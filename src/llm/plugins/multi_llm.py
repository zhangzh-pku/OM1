import logging
import typing as T

import aiohttp
from pydantic import BaseModel, Field

from llm import LLM, LLMConfig

R = T.TypeVar("R", bound=BaseModel)


class Command(BaseModel):
    """Executable action with its argument."""

    type: str = Field(..., description="The type of action")
    value: str = Field(..., description="The action argument")


class RoboticTeamRequest(BaseModel):
    """Request model for the robotic team endpoint"""

    message: str
    model: str


class RoboticTeamResponse(BaseModel):
    """Response model from the robotic team endpoint"""

    commands: list[Command] = Field(..., description="List of actions to execute")


class MultiLLM(LLM[R]):
    """
    MultiLLM implementation that sends requests to the robotic team endpoint.
    The endpoint returns a list of commands that can be executed by action plugins.
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
        Send a prompt to the robotic team endpoint and return a list of executable commands.

        Parameters
        ----------
        prompt : str
            The prompt to send to the endpoint
        messages : List[Dict[str, str]], optional
            Message history (not used by this endpoint)

        Returns
        -------
        R
            Response containing a list of commands to execute, or None if the request fails
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

                    # Step 3: Parse and validate response as a list of commands
                    try:
                        data = await response.json()
                        api_response = RoboticTeamResponse(**data)

                        # Step 4: Convert to output model format
                        # The output model should match CortexOutputModel structure
                        return self._output_model(commands=api_response.commands)

                    except Exception as e:
                        logging.error(f"Error parsing API response: {str(e)}")
                        return None

        except Exception as e:
            logging.error(f"Error during API request: {str(e)}")
            return None
