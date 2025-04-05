import json
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
    This implementation converts the robotic team API responses to match the
    output structure of other LLM plugins.

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

                    # Process the response to match the output structure of other LLM plugins
                    try:
                        # Handle structured output format
                        if (
                            "structured_output" in response_json
                            and response_json["structured_output"]
                        ):
                            # The structured output should directly match our output model
                            structured_data = response_json["structured_output"]
                            try:
                                # Pass the structured output directly to our output model
                                parsed_response = (
                                    self._output_model.model_validate_json(
                                        json.dumps(structured_data)
                                    )
                                )
                                logging.debug(
                                    f"MultiLLM structured output: {parsed_response}"
                                )
                                return parsed_response
                            except Exception as e:
                                logging.debug(
                                    f"Could not validate structured output: {e}"
                                )

                        # Handle API command format
                        if "commands" in response_json and response_json["commands"]:
                            # For command-based responses, create JSON that matches our output model
                            commands_json = {}
                            commands = response_json["commands"]

                            # Extract the first command for simplicity
                            if len(commands) > 0 and isinstance(commands[0], dict):
                                first_command = commands[0]
                                if "command" in first_command:
                                    # Use command/args format
                                    field_name = self._get_output_model_field_name()
                                    if field_name:
                                        commands_json[field_name] = (
                                            f"{first_command.get('command')}: "
                                            f"{first_command.get('args', '')}"
                                        ).strip()

                            if commands_json:
                                try:
                                    parsed_response = (
                                        self._output_model.model_validate_json(
                                            json.dumps(commands_json)
                                        )
                                    )
                                    logging.debug(
                                        f"MultiLLM command response: {parsed_response}"
                                    )
                                    return parsed_response
                                except Exception as e:
                                    logging.error(
                                        f"Failed to validate command response: {e}"
                                    )

                        # Handle content-only format
                        if "content" in response_json and response_json["content"]:
                            content = response_json["content"]
                            field_name = self._get_output_model_field_name()

                            if field_name:
                                content_json = {field_name: content}
                                try:
                                    parsed_response = (
                                        self._output_model.model_validate_json(
                                            json.dumps(content_json)
                                        )
                                    )
                                    logging.debug(
                                        f"MultiLLM content response: {parsed_response}"
                                    )
                                    return parsed_response
                                except Exception as e:
                                    logging.error(
                                        f"Failed to validate content response: {e}"
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

    def _get_output_model_field_name(self) -> str:
        """
        Get the field name from the output model to use in the response.

        Returns
        -------
        str
            The name of the first field in the output model
        """
        try:
            # Get the first field name from the output model schema
            schema = self._output_model.model_json_schema()
            if "properties" in schema and schema["properties"]:
                return next(iter(schema["properties"].keys()))
            return "result"  # Default field name if we can't determine one
        except Exception:
            return "result"  # Default fallback
