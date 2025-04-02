import logging
import typing as T

import aiohttp
from pydantic import BaseModel, Field

from llm import LLM, LLMConfig

R = T.TypeVar("R", bound=BaseModel)


class Command(BaseModel):
    """Executable action with its argument."""

    type: str = Field(..., description="The type of action")
    value: str = Field(None, description="The action argument")


class ApiCommand(BaseModel):
    """Command format returned by the robotic_team API."""

    command: str = Field(..., description="The command type")
    args: T.Optional[str] = Field(None, description="Optional command arguments")


class RoboticTeamRequest(BaseModel):
    """Request model for the robotic team endpoint"""

    message: str
    model: str


class RoboticTeamResponse(BaseModel):
    """Response model from the robotic team endpoint"""

    commands: list[ApiCommand] = Field(..., description="List of commands to execute")


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

                    try:
                        # Step 4: Parse response format
                        # The API returns either:
                        # 1. An object with a "commands" array containing command objects
                        # 2. A string in the "content" field (markdown format)
                        commands = []

                        if (
                            isinstance(response_json, dict)
                            and "commands" in response_json
                        ):
                            # Format: {"commands": [{"command": "wag tail"}, {"command": "speak", "args": "Woof!"}]}
                            api_commands = RoboticTeamResponse(**response_json).commands
                            logging.debug(f"API commands: {api_commands}")

                            for cmd in api_commands:
                                # Map API command format to our Command model
                                cmd_type = cmd.command.lower()
                                cmd_value = cmd.args if cmd.args is not None else ""
                                commands.append(Command(type=cmd_type, value=cmd_value))

                        elif (
                            isinstance(response_json, dict)
                            and "content" in response_json
                        ):
                            # Format: {"content": "markdown text with commands"}
                            content = response_json["content"]
                            logging.debug(f"Content field found: {content}")

                            # Extract commands from markdown bullet points
                            # Find lines that start with bullet points (* or -)
                            action_lines = [
                                line.strip()
                                for line in content.split("\n")
                                if (
                                    line.strip().startswith("- ")
                                    or line.strip().startswith("* ")
                                )
                            ]

                            for line in action_lines:
                                # Remove leading markers
                                line = line.lstrip("- *").strip()

                                # Remove any surrounding quotes
                                line = line.strip('"')

                                # Handle markdown bold markers
                                line = line.replace("**", "")

                                # Try to extract command and value
                                if "." in line and not line.startswith("http"):
                                    # Format: "Forward Scan. Obstacle detected. Type: Table."
                                    parts = line.split(".", 1)
                                    cmd_type = parts[0].strip().lower()
                                    cmd_value = (
                                        parts[1].strip() if len(parts) > 1 else ""
                                    )
                                    commands.append(
                                        Command(type=cmd_type, value=cmd_value)
                                    )
                                elif ":" in line:
                                    # Format: "Target Lock: Table"
                                    parts = line.split(":", 1)
                                    cmd_type = parts[0].strip().lower()
                                    cmd_value = parts[1].strip()
                                    commands.append(
                                        Command(type=cmd_type, value=cmd_value)
                                    )
                                elif " " in line:
                                    # Use the first word as type and rest as value
                                    parts = line.split(" ", 1)
                                    cmd_type = parts[0].strip().lower()
                                    cmd_value = (
                                        parts[1].strip() if len(parts) > 1 else ""
                                    )
                                    commands.append(
                                        Command(type=cmd_type, value=cmd_value)
                                    )
                                else:
                                    # Just use the whole line as the command type
                                    commands.append(
                                        Command(type=line.lower(), value="")
                                    )

                            # If no commands were extracted, use content as response
                            if not commands:
                                commands.append(Command(type="response", value=content))
                        else:
                            logging.error(
                                f"Unrecognized response format: {response_json}"
                            )
                            return None

                        logging.debug(f"Parsed commands: {commands}")

                        # Create output model with commands
                        return self._output_model(commands=commands)

                    except Exception as e:
                        logging.error(f"Error parsing response: {str(e)}")
                        return None

        except Exception as e:
            logging.error(f"Error during API request: {str(e)}")
            return None
