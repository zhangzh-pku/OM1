import logging
import time
import typing as T

import aiohttp
from pydantic import BaseModel, Field

from llm import LLM, LLMConfig

R = T.TypeVar("R", bound=BaseModel)


class ApiCommand(BaseModel):
    """Command format returned by the robotic_team API."""

    command: str = Field(..., description="The command type")
    args: T.Optional[str] = Field(None, description="Optional command arguments")


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
    The endpoint returns either structured output or a list of commands that
    can be converted to our output model format.

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
        Initialize the MultiLLM instance.

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

        self.api_key = config.api_key

        if not config.model:
            self._config.model = "gemini-2.0-flash"

        # Configure the API endpoint
        self.endpoint = "https://api.openmind.org/api/core/agent/robotic_team/runs"

        # We don't use LLMHistoryManager for this LLM
        self.history_manager = None

        logging.debug(
            f"MultiLLM initialized with output model: {output_model.__name__}"
        )
        logging.debug(f"Output model schema: {output_model.model_json_schema()}")

    async def ask(
        self, prompt: str, messages: T.List[T.Dict[str, str]] = []
    ) -> R | None:
        """
        Send a prompt to the robotic team endpoint and get a structured response.

        Parameters
        ----------
        prompt : str
            The input prompt to send to the model.
        messages : List[Dict[str, str]]
            Message history (not used in current implementation).

        Returns
        -------
        R or None
            Parsed response matching the output_model structure, or None if
            parsing fails.
        """
        try:
            logging.debug(f"MultiLLM input: {prompt}")

            self.io_provider.llm_start_time = time.time()
            self.io_provider.set_llm_prompt(prompt)

            # Prepare request data - always request structured output
            request = {
                "message": prompt,
                "response_model": self._output_model.model_json_schema(),
                "structured_outputs": True,
            }

            async with aiohttp.ClientSession() as session:
                headers = {"Authorization": f"Bearer {self._config.api_key}"}

                logging.debug(f"Sending request to {self.endpoint}")

                # Send request to endpoint
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

                    response_json = await response.json()
                    logging.debug(f"Raw API response: {response_json}")
                    self.io_provider.llm_end_time = time.time()

                    # Process the response based on what fields are present
                    try:
                        # Case 1: Direct structured output that matches our output model
                        if (
                            "structured_output" in response_json
                            and response_json["structured_output"]
                        ):
                            structured_data = response_json["structured_output"]

                            # Try to validate against our output model directly
                            try:
                                parsed_response = self._output_model.model_validate(
                                    structured_data
                                )
                                logging.debug(
                                    f"MultiLLM structured output: {parsed_response}"
                                )
                                return parsed_response
                            except Exception as e:
                                logging.debug(
                                    f"Could not directly validate structured output: {e}"
                                )
                                # If validation fails, we'll try converting it below

                        # Case 2: API format with command/args that needs converting to type/value
                        if "commands" in response_json and response_json["commands"]:
                            commands = []
                            for cmd in response_json["commands"]:
                                if isinstance(cmd, dict):
                                    if "command" in cmd:
                                        # Convert from API format to our format
                                        cmd_type = cmd.get("command", "").lower()
                                        cmd_value = cmd.get("args", "")
                                        if cmd_value is None:
                                            cmd_value = ""
                                        commands.append(
                                            {"type": cmd_type, "value": cmd_value}
                                        )
                                    elif "type" in cmd:
                                        # Already in our format
                                        commands.append(cmd)

                            if commands:
                                output_json = {"commands": commands}
                                try:
                                    parsed_response = self._output_model.model_validate(
                                        output_json
                                    )
                                    logging.debug(
                                        f"MultiLLM converted commands: {parsed_response}"
                                    )
                                    return parsed_response
                                except Exception as e:
                                    logging.error(
                                        f"Failed to validate converted commands: {e}"
                                    )

                        # Case 3: Content field only - use as a single "response" command
                        if "content" in response_json and response_json["content"]:
                            content = response_json["content"]
                            output_json = {
                                "commands": [{"type": "response", "value": content}]
                            }

                            try:
                                parsed_response = self._output_model.model_validate(
                                    output_json
                                )
                                logging.debug(
                                    f"MultiLLM content response: {parsed_response}"
                                )
                                return parsed_response
                            except Exception as e:
                                logging.error(
                                    f"Failed to validate content as response: {e}"
                                )

                        logging.error(
                            "Could not convert response to valid output format"
                        )
                        return None

                    except Exception as e:
                        logging.error(f"Error processing response: {str(e)}")
                        return None

        except Exception as e:
            logging.error(f"Error during API request: {str(e)}")
            return None
