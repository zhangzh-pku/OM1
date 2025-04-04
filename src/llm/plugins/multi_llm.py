import logging
import typing as T

import aiohttp
from pydantic import BaseModel, Field

from llm import LLM, LLMConfig

R = T.TypeVar("R", bound=BaseModel)


class Command(BaseModel):
    """
    Executable action with its argument.

    Parameters
    ----------
    type : str
        Type of action to execute
    value : str
        The action argument, such as magnitude or sentence to speak
    """

    type: str = Field(..., description="The type of action")
    value: str = Field(..., description="The action argument")


class ApiCommand(BaseModel):
    """Command format returned by the robotic_team API."""

    command: str = Field(..., description="The command type")
    args: T.Optional[str] = Field(None, description="Optional command arguments")


class RoboticTeamRequest(BaseModel):
    """Request model for the robotic team endpoint"""

    message: str
    model: str
    response_model: T.Optional[dict] = Field(
        None, description="Pydantic model schema for structured output"
    )
    structured_outputs: bool = Field(
        False, description="Whether to use structured output format"
    )


class RoboticTeamResponse(BaseModel):
    """Response model from the robotic team endpoint"""

    content: T.Optional[str] = Field(
        None, description="Text content in markdown format"
    )
    structured_output: T.Optional[dict] = Field(
        None, description="Structured output data"
    )
    commands: T.Optional[list[ApiCommand]] = Field(
        None, description="List of commands to execute"
    )


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

        # Configure the API endpoint
        self.endpoint = "https://api.openmind.org/api/core/agent/robotic_team/runs"
        self.history_manager = None  # We don't use history management for this LLM

        # Log that we're using structured outputs
        logging.info(f"MultiLLM initialized with output model: {output_model.__name__}")
        logging.debug(f"Output model schema: {output_model.model_json_schema()}")

    async def ask(self, prompt: str, messages: T.List[T.Dict[str, str]] = []) -> R:
        """Send a prompt to the robotic team endpoint and return the response."""
        try:
            # Step 1: Prepare request data
            request = {
                "message": prompt,
                "model": self._config.model,
                "response_model": self._output_model.model_json_schema(),
                "structured_outputs": True,
            }

            async with aiohttp.ClientSession() as session:
                headers = {"Authorization": f"Bearer {self._config.api_key}"}

                logging.debug(f"Sending request to {self.endpoint}: {request}")
                # Step 2: Send request to endpoint
                async with session.post(
                    self.endpoint,
                    json=request,
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
                        # Handle different response formats

                        # Case 1: Structured output format with direct model data
                        if (
                            isinstance(response_json, dict)
                            and "structured_output" in response_json
                            and response_json["structured_output"]
                        ):
                            # This is the new structured output format
                            logging.debug("Using structured output from response")
                            structured_data = response_json["structured_output"]

                            # Parse and validate with output model
                            parsed_response = self._output_model.model_validate(
                                structured_data
                            )
                            logging.info(
                                f"MultiLLM structured output: {parsed_response}"
                            )
                            return parsed_response

                        # Case 2: OpenAI/Gemini format with pre-processed commands with 'type' field
                        elif (
                            isinstance(response_json, dict)
                            and "commands" in response_json
                            and response_json["commands"]
                            and isinstance(response_json["commands"][0], dict)
                            and "type" in response_json["commands"][0]
                        ):
                            # Already in the correct format, just validate with output model
                            logging.debug("Response has pre-processed commands")
                            parsed_response = self._output_model.model_validate(
                                response_json
                            )
                            logging.info(f"MultiLLM output: {parsed_response}")
                            return parsed_response

                        # Case 3: API format with 'command' and 'args' fields
                        elif (
                            isinstance(response_json, dict)
                            and "commands" in response_json
                            and response_json["commands"]
                            and isinstance(response_json["commands"][0], dict)
                            and "command" in response_json["commands"][0]
                        ):
                            # Convert from API format to our format
                            commands = []
                            for cmd in response_json["commands"]:
                                cmd_type = cmd.get("command", "").lower()
                                cmd_value = cmd.get("args", "")
                                if cmd_value is None:
                                    cmd_value = ""
                                commands.append({"type": cmd_type, "value": cmd_value})

                            # Create JSON compatible with our output model
                            output_json = {"commands": commands}

                            # Parse and validate with output model
                            try:
                                # Log the model schema and output_json for debugging
                                logging.debug(
                                    f"Output model schema: {self._output_model.model_json_schema()}"
                                )
                                logging.debug(f"Output JSON to validate: {output_json}")

                                parsed_response = self._output_model.model_validate(
                                    output_json
                                )
                                logging.info(f"MultiLLM output: {parsed_response}")
                                return parsed_response
                            except Exception as e:
                                logging.info(f"Direct validation failed: {e}")
                                logging.info(
                                    "Response needs transformation before using with output_model.py"
                                )

                                # If validation fails, create a response with a single command
                                # containing the entire content as a fallback
                                content_value = response_json.get(
                                    "content", "No content available"
                                )
                                output_json = {
                                    "commands": [
                                        {"type": "response", "value": content_value}
                                    ]
                                }

                                try:
                                    # Log the model schema and output_json for debugging
                                    logging.debug(
                                        f"Output model schema: {self._output_model.model_json_schema()}"
                                    )
                                    logging.debug(
                                        f"Output JSON to validate: {output_json}"
                                    )

                                    parsed_response = self._output_model.model_validate(
                                        output_json
                                    )
                                    logging.info("Using fallback response format")
                                    return parsed_response
                                except Exception as e2:
                                    logging.error(
                                        f"Fallback validation also failed: {e2}"
                                    )
                                    return None

                        # Case 4: Content field with markdown text
                        elif (
                            isinstance(response_json, dict)
                            and "content" in response_json
                        ):
                            content = response_json["content"]
                            logging.info("\nResponse has 'content' field")

                            # Extract commands from the markdown content
                            commands = self._extract_commands_from_markdown(content)

                            # Create JSON compatible with our output model
                            output_json = {"commands": commands}

                            try:
                                # Parse and validate with output model
                                parsed_response = self._output_model.model_validate(
                                    output_json
                                )
                                logging.info(f"MultiLLM output: {parsed_response}")
                                return parsed_response
                            except Exception as e:
                                logging.info(f"Direct validation failed: {e}")
                                logging.info(
                                    "Response needs transformation before using with output_model.py"
                                )

                                # If validation fails, create a response with a single command
                                # containing the entire content as a fallback
                                output_json = {
                                    "commands": [{"type": "response", "value": content}]
                                }

                                try:
                                    # Log the model schema and output_json for debugging
                                    logging.debug(
                                        f"Output model schema: {self._output_model.model_json_schema()}"
                                    )
                                    logging.debug(
                                        f"Output JSON to validate: {output_json}"
                                    )

                                    parsed_response = self._output_model.model_validate(
                                        output_json
                                    )
                                    logging.info("Using fallback response format")
                                    return parsed_response
                                except Exception as e2:
                                    logging.error(
                                        f"Fallback validation also failed: {e2}"
                                    )
                                    return None

                        else:
                            logging.error(
                                f"Unrecognized response format: {response_json}"
                            )
                            return None

                    except Exception as e:
                        logging.error(f"Error parsing response: {str(e)}")
                        return None

        except Exception as e:
            logging.error(f"Error during API request: {str(e)}")
            return None

    def _extract_commands_from_markdown(self, content: str) -> list[dict]:
        """Extract commands from markdown content"""
        commands = []
        logging.debug(f"Extracting commands from content: {content}")

        # Find lines that start with bullet points (* or -) or numbered items (1., 2., etc)
        action_lines = [
            line.strip()
            for line in content.split("\n")
            if (
                line.strip().startswith("- ")
                or line.strip().startswith("* ")
                or (
                    line.strip()
                    and line.strip()[0].isdigit()
                    and "." in line.strip().split(" ")[0]
                )
            )
        ]

        if action_lines:
            logging.debug(f"Found action lines: {action_lines}")
        else:
            logging.debug("No bullet points found, trying to parse paragraphs")
            # If no bullet points, try to identify commands in paragraphs
            paragraphs = [p.strip() for p in content.split("\n\n") if p.strip()]
            for paragraph in paragraphs:
                if ":" in paragraph and len(paragraph.split(":", 1)[0].split()) <= 3:
                    # Simple format like "Move forward: The robot should move ahead"
                    action_lines.append(paragraph)
                elif "." in paragraph and not paragraph.startswith("http"):
                    # Try to use first sentence as command
                    sentences = paragraph.split(". ")
                    if len(sentences) > 0 and len(sentences[0].split()) <= 5:
                        action_lines.append(sentences[0])

        # Process all identified action lines
        for line in action_lines:
            # Remove leading markers and numbering
            clean_line = line.lstrip("- *").strip()
            if clean_line[0].isdigit() and "." in clean_line.split(" ")[0]:
                clean_line = " ".join(clean_line.split(" ")[1:]).strip()

            # Remove any surrounding quotes
            clean_line = clean_line.strip("\"'")

            # Handle markdown bold markers
            clean_line = clean_line.replace("**", "").replace("__", "")

            # Try to extract command and value
            if ":" in clean_line:
                # Format: "Target Lock: Table" or "move: forward 2 meters"
                parts = clean_line.split(":", 1)
                cmd_type = parts[0].strip().lower()
                cmd_value = parts[1].strip()
                commands.append({"type": cmd_type, "value": cmd_value})
            elif "." in clean_line and not clean_line.startswith("http"):
                # Format: "Forward Scan. Obstacle detected."
                parts = clean_line.split(".", 1)
                cmd_type = parts[0].strip().lower()
                cmd_value = parts[1].strip() if len(parts) > 1 else ""
                commands.append({"type": cmd_type, "value": cmd_value})
            elif " " in clean_line:
                # Use the first word as type and rest as value
                parts = clean_line.split(" ", 1)
                cmd_type = parts[0].strip().lower()
                cmd_value = parts[1].strip() if len(parts) > 1 else ""
                commands.append({"type": cmd_type, "value": cmd_value})
            else:
                # Just use the whole line as the command type
                commands.append({"type": clean_line.lower(), "value": ""})

        # If no commands were extracted, use content as response
        if not commands:
            logging.info(
                "No commands extracted, using entire content as 'response' command"
            )
            commands.append({"type": "response", "value": content})

        logging.debug(f"Extracted commands: {commands}")
        return commands
