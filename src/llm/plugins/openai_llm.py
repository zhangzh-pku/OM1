import logging
import time
import typing as T
from dataclasses import dataclass

import openai
from pydantic import BaseModel

from llm import LLM, LLMConfig
from providers.llm_history_manager import LLMHistoryManager

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

        # Initialize history manager
        self.history_length = (
            config.history_length if config and config.history_length else 0
        )
        self.history_manager = LLMHistoryManager(self._client, self.history_length)

    @LLMHistoryManager.update_history()
    async def ask(
        self, prompt: str, messages: T.List[T.Dict[str, str]] = []
    ) -> R | None:
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

            print(messages)

            self.io_provider.llm_start_time = time.time()
            self.io_provider.set_llm_prompt(prompt)

            parsed_response = await self._client.beta.chat.completions.parse(
                model=(
                    "gpt-4o-mini" if self._config.model is None else self._config.model
                ),
                messages=[*messages, {"role": "user", "content": prompt}],
                response_format=self._output_model,
            )

            message_content = parsed_response.choices[0].message.content
            self.io_provider.llm_end_time = time.time()

            try:
                parsed_response = self._output_model.model_validate_json(
                    message_content
                )
                logging.debug(f"LLM output: {parsed_response}")
                return parsed_response
            except Exception as e:
                logging.error(f"Error parsing response: {e}")
                return None
        except Exception as e:
            logging.error(f"Error asking LLM: {e}")
            return None
