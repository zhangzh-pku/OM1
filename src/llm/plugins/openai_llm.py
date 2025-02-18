import logging
import time
import typing as T
import re

import openai
from pydantic import BaseModel

from dataclasses import dataclass

from llm import LLM, LLMConfig

R = T.TypeVar("R", bound=BaseModel)

@dataclass
class History:
    """
    Container for timestamped interactions.

    Parameters
    ----------
    timestamp : float
        Unix timestamp of the interaction
    interaction : str
        The input/action pair
    """

    timestamp: float
    interaction: str

class OpenAILLM(LLM[R]):
    """
    An OpenAI-based Language Learning Model implementation.

    This class implements the LLM interface for OpenAI's GPT models, handling
    configuration, authentication, and async API communication.

    Parameters
    ----------
    output_model : Type[R]
        A Pydantic BaseModel subclass defining the expected response structure.
    config : LLMConfig, optional
        Configuration object containing API settings. If not provided, defaults
        will be used.
    """

    def __init__(self, output_model: T.Type[R], config: T.Optional[LLMConfig] = None):
        """
        Initialize the OpenAI LLM instance.

        Parameters
        ----------
        output_model : Type[R]
            Pydantic model class for response validation.
        config : LLMConfig, optional
            Configuration settings for the LLM.
        """
        super().__init__(output_model, config)
        logging.debug(f"Config OpenAI client with config: {config}")

        self.history_length = 0
        if config.history_length:
            self.history_length = config.history_length

        base_url = config.base_url or "https://api.openmind.org/api/core/openai"

        if config.api_key is None or config.api_key == "":
            raise ValueError("config file missing api_key")
        else:
            api_key = config.api_key

        # Messages buffer
        self.history: list[History] = []

        self.start_time = time.time()

        client_kwargs = {}
        client_kwargs["base_url"] = base_url
        client_kwargs["api_key"] = api_key

        logging.debug(f"Initializing OpenAI client with {client_kwargs}")
        self._client = openai.AsyncClient(**client_kwargs)

    def summarize_inputs(self, prompt: str) -> str | None:
        # extract the prompt inputs
        # import re
        # string = "// START\nWARNING: You are low on energy. SIT DOWN NOW.\n// END\n\n// START\nYou just saw a red dog.\n// END"
        # pattern = '// START\n(.*?)\n// END'
        # result = re.findall(pattern, string) 
        # print(result)

        pattern = '// START\n(.*?)\n// END'
        state_inputs = re.findall(pattern, prompt) 
        separator = " "
        state_inputs = separator.join(state_inputs)
        logging.debug(f"state_inputs: {state_inputs}")
        return state_inputs

    def add_to_history(self, summary: str, response):
        
        action_string = ""
        
        for command in response.commands:
            #logging.info(f"command: {command}")
            action_type = command.name
            #logging.info(f"parse: {command.arguments[0].value}")
            if action_type == "emotion":
                emotion = command.arguments[0].value
                #logging.info(f"emotion: {emotion}")
                action_string += f"You felt: {emotion}. "
            elif action_type == "speak":
                sentence = command.arguments[0].value
                #logging.info(f"sentence: {sentence}")
                action_string += f"You said: {sentence} "
            elif action_type == "move":
                motion = command.arguments[0].value
                #logging.info(f"movement: {movement}")
                action_string += f"You performed this motion: {motion}. "
                
        final_history = "Your inputs were: " + summary + " You decided: " + action_string

        elapsed_time = round(time.time() - self.start_time, 1)

        interaction = History(timestamp=elapsed_time, interaction=final_history)
        
        self.history.append(interaction)
        if len(self.history) > self.history_length:
            self.history.pop(0)

        logging.info(f"History:\n{self.history}")

    async def ask(self, prompt: str) -> R | None:
        """
        Send a prompt to the OpenAI API and get a structured response.

        Parameters
        ----------
        prompt : str
            The input prompt to send to the model.

        Returns
        -------
        R or None
            Parsed response matching the output_model structure, or None if
            parsing fails.
        """
        try:
            logging.debug(f"OpenAI LLM input: {prompt}")
            
            self.io_provider.llm_start_time = time.time()
            self.io_provider.set_llm_prompt(prompt)

            parsed_response = await self._client.beta.chat.completions.parse(
                model=(
                    "gpt-4o-mini" if self._config.model is None else self._config.model
                ),
                messages=[{"role": "user", "content": prompt}],
                response_format=self._output_model,
            )

            message_content = parsed_response.choices[0].message.content
            self.io_provider.llm_end_time = time.time()

            try:
                parsed_response = self._output_model.model_validate_json(
                    message_content
                )
                logging.debug(f"LLM output: {parsed_response}")

                summary = self.summarize_inputs(prompt)
                self.add_to_history(summary, parsed_response)

                return parsed_response
            except Exception as e:
                logging.error(f"Error parsing response: {e}")
                return None
        except Exception as e:
            logging.error(f"Error asking LLM: {e}")
            return None
