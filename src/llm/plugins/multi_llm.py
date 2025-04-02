import logging
import time
import typing as T

import aiohttp
from pydantic import BaseModel

from llm import LLM, LLMConfig
from providers.llm_history_manager import LLMHistoryManager

R = T.TypeVar("R", bound=BaseModel)


class MultiLLM(LLM[R]):
    """
    Multi-Agent LLM implementation using the OpenMind robotic_team endpoint.

    This class provides access to a multi-agent system consisting of navigation and perception
    specialists, coordinated by a supervisor agent. The robotic team includes:

    1. Navigation Agent - expert in path planning, movement control, and spatial reasoning
    2. Perception Agent - expert in sensor data interpretation and environmental understanding
    3. Supervisor Agent - coordinates the team members to solve complex tasks

    The system automatically routes queries to the most appropriate agent and returns unified responses.

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
        Initialize the Multi-Agent LLM instance.

        Parameters
        ----------
        output_model : Type[R]
            Pydantic model class for response validation.
        config : LLMConfig, optional
            Configuration settings for the LLM.
        """
        super().__init__(output_model, config)

        if not config.api_key:
            raise ValueError("config file missing api_key")
        if not config.model:
            # Default model used for team agents (navigation and perception)
            self._config.model = "gemini-2.0-flash"

        self.base_url = config.base_url or "https://api.openmind.org/api/core"
        # Fixed endpoint for robotic_team agent
        self.endpoint = f"{self.base_url}/agent/robotic_team/runs"

        # Initialize history manager (compatible with other LLMs)
        self.history_manager = LLMHistoryManager(self._config, None)

    @LLMHistoryManager.update_history()
    async def ask(
        self, prompt: str, messages: T.List[T.Dict[str, str]] = None, *args, **kwargs
    ) -> R:
        """
        Send a prompt to the Multi-Agent system and get a structured response.

        The system will automatically:
        1. Analyze the query to determine the required expertise
        2. Route the query to the appropriate specialist agent
        3. Format and return the response

        Parameters
        ----------
        prompt : str
            The input prompt to send to the model.
        messages : List[Dict[str, str]], optional
            List of message dictionaries (for context/history).
            Note: The robotic_team endpoint doesn't use message history
            but we accept it for compatibility with other LLMs.

        Returns
        -------
        R
            Parsed response matching the output_model structure, or None if
            parsing fails.
        """
        if messages is None:
            messages = []

        try:
            logging.info(f"Multi-Agent LLM input: {prompt}")

            self.io_provider.llm_start_time = time.time()
            self.io_provider.set_llm_prompt(prompt)

            # Format the request payload for the robotic_team endpoint
            # Following the structure in digest copy.txt
            payload = {"model": self._config.model, "message": prompt}

            # Prepare authorization header with API key
            headers = {
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self._config.api_key}",
            }

            # Make HTTP request to the robotic_team endpoint
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    self.endpoint, json=payload, headers=headers
                ) as response:
                    if response.status != 200:
                        error_text = await response.text()
                        logging.error(
                            f"Multi-Agent API error: {response.status} - {error_text}"
                        )
                        return None

                    # Parse the response from the robotic_team endpoint
                    result = await response.json()
                    content = result.get("content", "")
                    self.io_provider.llm_end_time = time.time()

                    # Format the response according to the output model
                    try:
                        # Create a formatted response that matches the required output model
                        formatted_response = {
                            "content": content,
                            "model_used": self._config.model,
                            "agent_type": "robotic_team",
                        }

                        # Add any additional fields required by the output model
                        parsed_response = self._output_model.model_validate(
                            formatted_response
                        )
                        logging.info("Multi-Agent LLM output successfully parsed")
                        return parsed_response
                    except Exception as e:
                        logging.error(f"Error parsing Multi-Agent response: {e}")
                        return None

        except Exception as e:
            logging.error(f"Multi-Agent API request error: {e}")
            return None
