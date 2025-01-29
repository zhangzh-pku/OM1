import logging
import os
import time
import typing as T

import openai
from pydantic import BaseModel

from llm import LLM, LLMConfig
from providers.io_provider import IOProvider

R = T.TypeVar("R", bound=BaseModel)


class GeminiLLM(LLM[R]):
    """
    Google Gemini LLM implementation using OpenAI-compatible API.

    Handles authentication and response parsing for Gemini endpoints.
    """

    def __init__(self, output_model: T.Type[R], config: T.Optional[LLMConfig] = None):
        """
        Initialize the DeepSeek LLM instance.

        Parameters
        ----------
        output_model : Type[R]
            Pydantic model class for response validation.
        config : LLMConfig, optional
            Configuration settings for the LLM.
        """
        super().__init__(output_model, config)

        # Configure authentication
        api_key = (
            os.getenv("GEMINI_API_KEY")
            or os.getenv("OPENMIND_API_KEY")
            or (config.api_key if config else None)
        )

        # Configure base URL
        base_url = config.base_url if config else None
        default_base_url = "https://generativelanguage.googleapis.com/v1beta/openai/"
        resolved_base_url = base_url or default_base_url

        # Set up client arguments
        client_kwargs = {
            "base_url": resolved_base_url,
            "api_key": api_key or "openmind-0x",
        }

        # Warn about rate limits if using public endpoint
        if not api_key and resolved_base_url == default_base_url:
            logging.warning("Gemini API key not found. The rate limit may be applied.")

        # Initialize OpenAI-compatible client
        self._client = openai.AsyncOpenAI(**client_kwargs)
        self.io_provider = IOProvider()

        # Debug logging
        logging.debug(f"Gemini API key configured: {bool(api_key)}")
        logging.debug(f"Using base URL: {resolved_base_url}")

    async def ask(self, prompt: str) -> R | None:
        """Execute LLM query and parse response"""
        try:
            self.io_provider.llm_start_time = time.time()
            self.io_provider.set_llm_prompt(prompt)
            response = self._execute_api_request(prompt)
            self.io_provider.llm_end_time = time.time()
            return self._parse_response(response)

        except Exception as error:
            logging.error(f"Gemini API error: {error}")
            return None

    def _execute_api_request(self, prompt: str):
        """Execute the actual API call to Gemini"""
        return self._client.chat.completions.create(
            model="gemini-2.0-flash-exp",
            messages=self._build_messages(prompt),
            response_format={"type": "json_object"},
        )

    def _build_messages(self, prompt: str) -> list[dict]:
        """Construct message payload for API request"""
        system_message = {
            "role": "system",
            "content": f"Respond with valid JSON matching this schema: {self._output_model.model_json_schema()}",
        }
        return [system_message, {"role": "user", "content": prompt}]

    def _parse_response(self, response) -> R | None:
        try:
            content = response.choices[0].message.content
            parsed = self._output_model.model_validate_json(content)
            logging.debug(f"Gemini output: {parsed}")
            return parsed
        except Exception as error:
            logging.error(f"Failed to parse Gemini response: {error}")
            return None
