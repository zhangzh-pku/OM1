import json
import logging
import time
import typing as T

from pydantic import BaseModel

from llm import LLM, LLMConfig

R = T.TypeVar("R", bound=BaseModel)


class LlamaLLM(LLM[R]):
    """
    A Meta Llama Language Learning Model implementation.

    This class implements the LLM interface for Meta's Llama models, handling
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
        Initialize the Llama LLM instance.

        Parameters
        ----------
        output_model : Type[R]
            Pydantic model class for response validation.
        config : LLMConfig, optional
            Configuration settings for the LLM.
        """
        super().__init__(output_model, config)

        if not config.api_key:
            raise ValueError("config file missing api_key for Llama")
        if not config.model:
            self._config.model = "llama-3.2-3b-instruct"

        # Import llama_api here to handle optional dependency
        try:
            from llama_api import Client as LlamaClient
        except ImportError:
            raise ImportError(
                "llama-api-python is not installed. "
                "Please install it with: pip install llama-api-python"
            )

        # Initialize Llama client
        # Note: If base_url is provided, use it for self-hosted endpoints
        if config.base_url:
            self._client = LlamaClient(
                api_key=config.api_key,
                base_url=config.base_url
            )
        else:
            self._client = LlamaClient(api_key=config.api_key)

        # Wrap client for async compatibility
        self._async_client = AsyncLlamaWrapper(self._client)

    async def ask(
        self, prompt: str, messages: T.List[T.Dict[str, str]] = []
    ) -> R | None:
        """
        Send a prompt to the Llama API and get a structured response.

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
            logging.info(f"Llama LLM input: {prompt}")
            logging.info(f"Llama LLM messages: {messages}")

            self.io_provider.llm_start_time = time.time()

            # Save the input information for debugging
            self.io_provider.set_llm_prompt(prompt)

            # Prepare messages for Llama API
            llama_messages = []
            
            # Add system message if needed
            system_prompt = None
            for msg in messages:
                role = msg.get("role", "user")
                content = msg.get("content", "")
                
                if role == "system":
                    # Llama models often handle system messages differently
                    # Concatenate system messages as a prefix
                    if system_prompt:
                        system_prompt += "\n" + content
                    else:
                        system_prompt = content
                else:
                    llama_messages.append({
                        "role": role,
                        "content": content
                    })
            
            # Add current prompt
            current_message = prompt
            
            # If we have a system prompt, prepend it to the first user message
            if system_prompt:
                schema_info = self._get_schema_instruction()
                current_message = f"{system_prompt}\n\n{schema_info}\n\n{prompt}"
            else:
                schema_info = self._get_schema_instruction()
                current_message = f"{schema_info}\n\n{prompt}"
            
            llama_messages.append({
                "role": "user",
                "content": current_message
            })

            # Call Llama API
            response = await self._async_client.chat_complete(
                model=self._config.model,
                messages=llama_messages,
                max_tokens=self._config.extra_params.get("max_tokens", 4096),
                temperature=self._config.extra_params.get("temperature", 0.7),
                timeout=self._config.timeout,
            )

            self.io_provider.llm_end_time = time.time()

            # Extract and parse the response
            if response and hasattr(response, 'choices') and len(response.choices) > 0:
                message_content = response.choices[0].message.content
                
                # Try to extract JSON from the response
                json_content = self._extract_json(message_content)
                
                if json_content:
                    try:
                        # Parse the JSON response to our output model
                        parsed_response = self._output_model.model_validate_json(
                            json_content
                        )
                        logging.info(f"Llama LLM output: {parsed_response}")
                        return parsed_response
                    except Exception as e:
                        logging.error(f"Error parsing Llama response: {e}")
                        logging.error(f"JSON content: {json_content}")
                        return None
                else:
                    logging.error("Could not extract JSON from Llama response")
                    logging.error(f"Response content: {message_content}")
                    return None
            else:
                logging.error("Empty response from Llama API")
                return None

        except Exception as e:
            logging.error(f"Llama API error: {e}")
            return None

    def _get_schema_instruction(self) -> str:
        """
        Generate instruction for the model to output JSON in the required schema.
        
        Returns
        -------
        str
            Instruction string with schema information.
        """
        schema_dict = self._output_model.model_json_schema()
        return (
            f"You must respond with valid JSON that matches this schema:\n"
            f"{json.dumps(schema_dict, indent=2)}\n"
            f"Output only the JSON object, no additional text."
        )

    def _extract_json(self, text: str) -> str | None:
        """
        Extract JSON content from the model's response.
        
        Parameters
        ----------
        text : str
            The raw response text from the model.
            
        Returns
        -------
        str or None
            Extracted JSON string, or None if no valid JSON found.
        """
        # Try to find JSON content in the response
        text = text.strip()
        
        # If the entire response is JSON, return it
        if text.startswith('{') and text.endswith('}'):
            return text
        
        # Try to find JSON block in markdown code blocks
        import re
        json_pattern = r'```(?:json)?\s*(\{[\s\S]*?\})\s*```'
        matches = re.findall(json_pattern, text)
        if matches:
            return matches[0]
        
        # Try to find raw JSON object in text
        json_pattern = r'(\{[\s\S]*\})'
        matches = re.findall(json_pattern, text)
        if matches:
            # Try to validate the first match as JSON
            for match in matches:
                try:
                    json.loads(match)
                    return match
                except (json.JSONDecodeError, ValueError):
                    continue
        
        return None


class AsyncLlamaWrapper:
    """
    Async wrapper for Llama client to work with history manager.
    """
    
    def __init__(self, client):
        self._client = client
    
    async def chat_complete(self, **kwargs):
        """
        Async wrapper for Llama chat completion.
        
        Since Llama SDK might be synchronous, we wrap it to be async-compatible.
        """
        import asyncio
        
        # Remove timeout from kwargs if not supported
        kwargs.pop("timeout", None)
        
        # Run the synchronous call in a thread pool
        loop = asyncio.get_event_loop()
        
        # Check if the client has async support
        if hasattr(self._client, 'chat') and hasattr(self._client.chat, 'complete'):
            # Use the standard chat.complete interface
            return await loop.run_in_executor(
                None,
                lambda: self._client.chat.complete(**kwargs)
            )
        elif hasattr(self._client, 'completion'):
            # Alternative interface
            return await loop.run_in_executor(
                None,
                lambda: self._client.completion(**kwargs)
            )
        else:
            # Fallback to direct call
            return await loop.run_in_executor(
                None,
                lambda: self._client(**kwargs)
            )