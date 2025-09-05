import logging
import time
import typing as T

from mistralai import Mistral
from pydantic import BaseModel

from llm import LLM, LLMConfig

R = T.TypeVar("R", bound=BaseModel)


class MistralLLM(LLM[R]):
    """
    A Mistral AI Language Learning Model implementation.

    This class implements the LLM interface for Mistral's models, handling
    configuration, authentication, and async API communication.

    Parameters
    ----------
    output_model : Type[R]
        A Pydantic BaseModel subclass defining the expected response structure.
    config : LLMConfig
        Configuration object containing API settings. If not provided, defaults
        will be used.
    """

    def __init__(self, output_model: T.Type[R], config: LLMConfig = LLMConfig()):
        """
        Initialize the Mistral LLM instance.

        Parameters
        ----------
        output_model : Type[R]
            Pydantic model class for response validation.
        config : LLMConfig, optional
            Configuration settings for the LLM.
        """
        super().__init__(output_model, config)

        if not config.api_key:
            raise ValueError("config file missing api_key for Mistral")
        if not config.model:
            self._config.model = "mistral-large-latest"

        # Initialize Mistral client
        self._client = Mistral(
            api_key=config.api_key,
            server_url=config.base_url if config.base_url else None,
        )

        # Initialize async client wrapper
        self._async_client = AsyncMistralWrapper(self._client)

    async def ask(
        self, prompt: str, messages: T.List[T.Dict[str, str]] = []
    ) -> R | None:
        """
        Send a prompt to the Mistral API and get a structured response.

        Parameters
        ----------
        prompt : str
            The input prompt to send to the model.
        messages : List[Dict[str, str]]
            List of message dictionaries to send to the model.

        Returns
        -------
        R or None
            Parsed response matching the output_model structure, or None if
            parsing fails.
        """
        try:
            logging.info(f"Mistral LLM input: {prompt}")
            logging.info(f"Mistral LLM messages: {messages}")

            self.io_provider.llm_start_time = time.time()

            # Save the input information for debugging
            self.io_provider.set_llm_prompt(prompt)

            # Prepare messages for Mistral API
            mistral_messages = []
            
            # Add conversation history
            for msg in messages:
                role = msg.get("role", "user")
                content = msg.get("content", "")
                # Mistral uses "assistant" instead of "system" for non-user messages
                if role == "system":
                    mistral_messages.append({"role": "system", "content": content})
                else:
                    mistral_messages.append({"role": role, "content": content})
            
            # Add current prompt as user message
            mistral_messages.append({"role": "user", "content": prompt})

            # Prepare response format schema for JSON mode
            schema_dict = self._output_model.model_json_schema()
            
            # Call Mistral API with JSON mode
            response = await self._async_client.chat_complete(
                model=self._config.model,
                messages=mistral_messages,
                response_format={
                    "type": "json_object",
                    "schema": schema_dict
                },
                timeout=self._config.timeout,
            )

            self.io_provider.llm_end_time = time.time()

            # Extract and parse the response
            if response and response.choices and len(response.choices) > 0:
                message_content = response.choices[0].message.content
                
                try:
                    # Parse the JSON response to our output model
                    parsed_response = self._output_model.model_validate_json(
                        message_content
                    )
                    logging.info(f"Mistral LLM output: {parsed_response}")
                    return parsed_response
                except Exception as e:
                    logging.error(f"Error parsing Mistral response: {e}")
                    logging.error(f"Response content: {message_content}")
                    return None
            else:
                logging.error("Empty response from Mistral API")
                return None

        except Exception as e:
            logging.error(f"Mistral API error: {e}")
            return None


class AsyncMistralWrapper:
    """
    Async wrapper for Mistral client to work with history manager.
    """
    
    def __init__(self, client: Mistral):
        self._client = client
    
    async def chat_complete(self, **kwargs):
        """
        Async wrapper for Mistral chat completion.
        
        Since Mistral SDK is synchronous, we wrap it to be async-compatible.
        """
        import asyncio
        
        # Remove timeout from kwargs as Mistral doesn't support it directly
        kwargs.pop("timeout", None)
        
        # Run the synchronous call in a thread pool
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(
            None,
            lambda: self._client.chat.complete(**kwargs)
        )