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

    content: str = Field(..., description="The response content from the model")


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
        """Send a prompt to the robotic team endpoint and return the response."""
        try:
            # Step 1: Prepare request data
            request = RoboticTeamRequest(message=prompt, model=self._config.model)

            async with aiohttp.ClientSession() as session:
                headers = {"Authorization": f"Bearer {self._config.api_key}"}

                # Step 2: Send request to endpoint
                async with session.post(
                    self.endpoint,
                    json=request.model_dump(),
                    headers=headers,
                ) as response:
                    if response.status != 200:
                        error_text = await response.text()
                        logging.error(
                            f"API request failed with status {response.status}: {error_text}"
                        )
                        return None

                    # Step 3: Get response content
                    response_json = await response.json()
                    logging.debug(f"Raw API response: {response_json}")

                    # Step 4: Validate the response structure
                    if (
                        not isinstance(response_json, dict)
                        or "content" not in response_json
                    ):
                        logging.error(f"Invalid response format: {response_json}")
                        return None

                    # Step 5: Parse the content field directly
                    # This contains markdown text from the TeamAgent, which we need to extract executable commands from
                    content = response_json["content"]
                    logging.debug(f"Raw content to parse: {content}")

                    # Extract commands from the content text based on formatting patterns
                    # For example, look for action patterns like "move forward" or "speak Hello"
                    commands = []

                    # Process action patterns by looking for common command indicators
                    # Only include lines that start with bullet points or dashes
                    action_lines = [
                        line.strip()
                        for line in content.split("\n")
                        if (
                            line.strip().startswith("- ")
                            or line.strip().startswith("* ")
                        )
                    ]
                    logging.debug(f"Extracted action lines: {action_lines}")

                    for line in action_lines:
                        # Remove leading markers
                        line = line.lstrip("- *").strip()

                        # Try to split on common separators
                        if ":" in line:
                            parts = line.split(":", 1)
                            cmd_type = parts[0].strip().lower()
                            cmd_value = parts[1].strip()
                            commands.append(Command(type=cmd_type, value=cmd_value))
                        elif " " in line:
                            # Use the first word as type and rest as value
                            parts = line.split(" ", 1)
                            cmd_type = parts[0].strip().lower()
                            cmd_value = parts[1].strip()
                            commands.append(Command(type=cmd_type, value=cmd_value))

                    # If no commands were found using patterns, create a single "response" command
                    if not commands:
                        commands.append(Command(type="response", value=content))

                    logging.debug(f"Extracted commands: {commands}")

                    # Create output model with the extracted commands
                    return self._output_model(commands=commands)

        except Exception as e:
            logging.error(f"Error during API request: {str(e)}")
            return None
