import logging
import typing as T

import aiohttp
from pydantic import BaseModel

from llm import LLM, LLMConfig

R = T.TypeVar("R")


class RoboticTeamRequest(BaseModel):
    """Request model for the robotic team endpoint"""

    message: str
    model: str


class RoboticTeamResponse(BaseModel):
    """Response model from the robotic team endpoint"""

    content: str


class MultiLLM(LLM[R]):
    """
    MultiLLM implementation that sends requests to the robotic team endpoint.
    """

    def __init__(self, output_model: T.Type[R], config: LLMConfig = LLMConfig()):
        super().__init__(output_model, config)

        if not config.api_key:
            raise ValueError("config file missing api_key")

        self.base_url = config.base_url
        self.api_key = config.api_key

        if not config.model:
            self._config.model = "gemini-2.0-flash"

        self.endpoint = "https://api.openmind.org/api/core/agent/robotic_team/runs"
        self.history_manager = None  # We don't use history management for this LLM

    async def ask(self, prompt: str, messages: T.List[T.Dict[str, str]] = []) -> R:
        """
        Send a prompt to the robotic team endpoint and return the response.

        Parameters
        ----------
        prompt : str
            The prompt to send to the endpoint
        messages : List[Dict[str, str]], optional
            Message history (not used by this endpoint)

        Returns
        -------
        R
            Response matching the output_model type specification, or None if the request fails
        """
        try:
            # Step 1: Validate request data
            request = RoboticTeamRequest(message=prompt, model=self._config.model)

            # Step 2: Send request to endpoint
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    self.endpoint,
                    json=request.model_dump(),
                    headers={"Authorization": f"Bearer {self._config.api_key}"},
                ) as response:
                    if response.status != 200:
                        error_text = await response.text()
                        logging.error(
                            f"API request failed with status {response.status}: {error_text}"
                        )
                        return None

                    # Step 3: Parse and validate response
                    try:
                        data = await response.json()
                        api_response = RoboticTeamResponse(**data)

                        # Step 4: Convert to output model format
                        return self._output_model(
                            content=api_response.content,
                            model_used=self._config.model,
                            agent_type="robotic_team",
                        )

                    except Exception as e:
                        logging.error(f"Error parsing API response: {str(e)}")
                        return None

        except Exception as e:
            logging.error(f"Error during API request: {str(e)}")
            return None
