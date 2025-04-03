from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from pydantic import BaseModel, Field, ValidationError

from llm import LLMConfig
from llm.plugins.multi_llm import (
    ApiCommand,
    Command,
    MultiLLM,
    RoboticTeamRequest,
    RoboticTeamResponse,
)


class DummyOutputModel(BaseModel):
    """Test output model matching the CortexOutputModel structure"""

    commands: list[Command] = Field(..., description="List of actions to execute")


@pytest.fixture
def config():
    return LLMConfig(
        base_url="https://api.openmind.org/api/core",
        api_key="test_api_key",
        model="gemini-2.0-flash",
    )


@pytest.fixture
def mock_response():
    """Mock API response with markdown content containing implicit commands"""
    return {
        "content": """
# Navigation Plan

Here's how to navigate around obstacles:

- move: forward 2 steps
- turn: left 90 degrees
- speak: I am moving around the obstacle
        """
    }


@pytest.fixture
def mock_unstructured_response():
    """Mock API response with content that doesn't have clear command patterns"""
    return {
        "content": "The robot should move forward, then turn left, then continue straight."
    }


@pytest.fixture
def mock_invalid_response():
    """Mock response with invalid format (missing content field)"""
    return {"wrong_field": "wrong value"}


@pytest.fixture
def mock_api_commands_response():
    """Mock API response with commands array (real format)"""
    return {
        "commands": [
            {"command": "wag tail"},
            {"command": "speak", "args": "Woof! I see a person and a TV!"},
            {"command": "emotion", "args": "smile"},
            {"command": "walk", "args": None},
        ]
    }


@pytest.fixture
def mock_real_api_response():
    """Mock API response with the actual format from the real API"""
    return {
        "content": """Okay, I'm analyzing the simulated sensor data now. Based on my current perception, here's what I "see" around me, translated into commands for a robot dog:

```
# Environmental Awareness Commands:

## Immediate Surroundings:

*   "**Forward Scan**. Obstacle detected. Type: Table. Distance: 1.5 meters."
*   "**Left Scan**. Obstacle detected. Type: Chair. Distance: 0.8 meters."
*   "**Right Scan**. Clear. No immediate obstacles."
*   "**Back Scan**. Obstacle detected. Type: Wall. Distance: 0.5 meters. **Caution: Impeding movement.**"

## Further Surroundings:

*   "**Forward Scan (Far)**. Obstacle detected. Type: Couch. Distance: 4 meters."
*   "**Right Scan (Far)**. Obstacle detected. Type: Plant. Distance: 5 meters."
*   "**Acoustic Analysis**. Sound detected. Source: Television. Approximate direction: Forward, Distance: 6 meters."

## Specific Object Commands:

*   "**Target Lock: Table**. Approach target cautiously. Stop within 0.3 meters."
*   "**Object Recognition: Plant**. Species: Ficus. Status: Healthy."
```
"""
    }


@pytest.fixture
def mock_openai_format_response():
    """Mock  response format with pre-processed commands"""
    return {
        "commands": [
            {"type": "move", "value": "sit"},
            {
                "type": "speak",
                "value": "Hi there! I'm Spot, your friendly dog! Woof woof!",
            },
            {"type": "emotion", "value": "joy"},
        ]
    }


@pytest.fixture
def llm(config):
    return MultiLLM(DummyOutputModel, config)


@pytest.mark.asyncio
async def test_init_with_config(llm, config):
    """Test that the MultiLLM initializes correctly with the given config"""
    assert llm.base_url == config.base_url
    assert llm.endpoint == "https://api.openmind.org/api/core/agent/robotic_team/runs"
    assert llm._config.model == config.model


@pytest.mark.asyncio
async def test_init_with_default_model():
    """Test that the MultiLLM uses the default model when not specified"""
    config = LLMConfig(api_key="test_key")
    llm = MultiLLM(DummyOutputModel, config)
    assert llm._config.model == "gemini-2.0-flash"


@pytest.mark.asyncio
async def test_init_empty_key():
    """Test that the MultiLLM raises an error when no API key is provided"""
    config = LLMConfig(base_url="test_url")
    with pytest.raises(ValueError, match="config file missing api_key"):
        MultiLLM(DummyOutputModel, config)


@pytest.mark.asyncio
async def test_ask_success(llm, mock_response):
    """Test successful API request and command extraction from markdown content"""
    with patch("aiohttp.ClientSession.post") as mock_post:
        mock_post.return_value.__aenter__.return_value.status = 200
        mock_post.return_value.__aenter__.return_value.json = AsyncMock(
            return_value=mock_response
        )

        result = await llm.ask("test prompt")
        assert result is not None
        assert len(result.commands) == 3
        assert result.commands[0].type == "move"
        assert result.commands[0].value == "forward 2 steps"
        assert result.commands[1].type == "turn"
        assert result.commands[1].value == "left 90 degrees"
        assert result.commands[2].type == "speak"
        assert result.commands[2].value == "I am moving around the obstacle"


@pytest.mark.asyncio
async def test_unstructured_content(llm, mock_unstructured_response):
    """Test handling of unstructured content without clear command patterns"""
    with patch("aiohttp.ClientSession.post") as mock_post:
        mock_post.return_value.__aenter__.return_value.status = 200
        mock_post.return_value.__aenter__.return_value.json = AsyncMock(
            return_value=mock_unstructured_response
        )

        result = await llm.ask("test prompt")
        assert result is not None
        assert len(result.commands) == 1
        assert result.commands[0].type == "response"
        assert "robot should move forward" in result.commands[0].value


@pytest.mark.asyncio
async def test_content_extraction():
    """Test extraction of commands from various content formats"""
    llm = MultiLLM(
        config=LLMConfig(api_key="test_key", model="test-model"),
        output_model=DummyOutputModel,
    )

    # Test with formatted action list using colons
    mock_response = {"content": "Actions:\n- move: forward\n- speak: hello"}
    with patch("aiohttp.ClientSession.post") as mock_post:
        mock_post.return_value.__aenter__.return_value.status = 200
        mock_post.return_value.__aenter__.return_value.json = AsyncMock(
            return_value=mock_response
        )

        result = await llm.ask("test prompt")
        assert result is not None
        assert len(result.commands) == 2
        assert result.commands[0].type == "move"
        assert result.commands[0].value == "forward"
        assert result.commands[1].type == "speak"
        assert result.commands[1].value == "hello"

    # Test with bulleted action list using spaces
    mock_response = {"content": "* move forward\n* turn left"}
    with patch("aiohttp.ClientSession.post") as mock_post:
        mock_post.return_value.__aenter__.return_value.status = 200
        mock_post.return_value.__aenter__.return_value.json = AsyncMock(
            return_value=mock_response
        )

        result = await llm.ask("test prompt")
        assert result is not None
        assert len(result.commands) == 2
        assert result.commands[0].type == "move"
        assert result.commands[0].value == "forward"
        assert result.commands[1].type == "turn"
        assert result.commands[1].value == "left"


@pytest.mark.asyncio
async def test_request_validation():
    """Test that the request model properly validates input"""
    # Valid request
    request = RoboticTeamRequest(message="test prompt", model="test-model")
    assert request.message == "test prompt"
    assert request.model == "test-model"

    # Invalid request - missing fields
    with pytest.raises(ValidationError):
        RoboticTeamRequest()

    with pytest.raises(ValidationError):
        RoboticTeamRequest(message="test prompt")

    with pytest.raises(ValidationError):
        RoboticTeamRequest(model="test-model")


@pytest.mark.asyncio
async def test_api_commands_format(llm, mock_api_commands_response):
    """Test handling of API response with commands array"""
    with patch("aiohttp.ClientSession.post") as mock_post:
        mock_post.return_value.__aenter__.return_value.status = 200
        mock_post.return_value.__aenter__.return_value.json = AsyncMock(
            return_value=mock_api_commands_response
        )

        result = await llm.ask("test prompt")
        assert result is not None
        assert len(result.commands) == 4

        # Verify command mapping
        assert result.commands[0].type == "wag tail"
        assert result.commands[0].value == ""

        assert result.commands[1].type == "speak"
        assert result.commands[1].value == "Woof! I see a person and a TV!"

        assert result.commands[2].type == "emotion"
        assert result.commands[2].value == "smile"

        assert result.commands[3].type == "walk"
        assert result.commands[3].value == ""


@pytest.mark.asyncio
async def test_response_validation():
    """Test that the response model properly validates input"""
    # Valid response with commands
    response = RoboticTeamResponse(
        commands=[ApiCommand(command="move"), ApiCommand(command="speak", args="hello")]
    )
    assert len(response.commands) == 2
    assert response.commands[0].command == "move"
    assert response.commands[1].args == "hello"

    # Invalid response - missing commands
    with pytest.raises(ValidationError):
        RoboticTeamResponse()

    # Invalid response - wrong structure
    with pytest.raises(ValidationError):
        RoboticTeamResponse(commands=[{"wrong": "structure"}])


@pytest.mark.asyncio
async def test_ask_invalid_response(llm):
    """Test handling of invalid response format"""
    # Mock the aiohttp ClientSession
    mock_session = MagicMock()
    mock_context = MagicMock()

    # Configure the mocks for an invalid response
    mock_session.__aenter__.return_value = mock_session
    mock_context.__aenter__.return_value = mock_context
    mock_context.status = 200
    mock_context.json = AsyncMock(return_value={"wrong_field": "wrong value"})
    mock_session.post = MagicMock(return_value=mock_context)

    # Test that invalid response is handled gracefully
    with patch("aiohttp.ClientSession", return_value=mock_session):
        result = await llm.ask("How do I navigate around obstacles?")
        assert result is None


@pytest.mark.asyncio
async def test_ask_api_error(llm):
    """Test handling of API errors"""
    # Mock the aiohttp ClientSession for an error response
    mock_session = MagicMock()
    mock_context = MagicMock()

    # Configure the mocks for an error
    mock_session.__aenter__.return_value = mock_session
    mock_context.__aenter__.return_value = mock_context
    mock_context.status = 500
    mock_context.text = AsyncMock(return_value="Internal Server Error")
    mock_session.post = MagicMock(return_value=mock_context)

    # Test error handling
    with patch("aiohttp.ClientSession", return_value=mock_session):
        result = await llm.ask("How do I navigate around obstacles?")
        assert result is None


@pytest.mark.asyncio
async def test_ask_network_exception(llm):
    """Test handling of network exceptions"""
    # Mock aiohttp to raise an exception during request
    with patch("aiohttp.ClientSession", side_effect=Exception("Connection error")):
        result = await llm.ask("How do I navigate around obstacles?")
        assert result is None


@pytest.mark.asyncio
async def test_ask_with_messages(llm, mock_response):
    """Test that the ask method handles message history correctly"""
    with patch("aiohttp.ClientSession.post") as mock_post:
        mock_post.return_value.__aenter__.return_value.status = 200
        mock_post.return_value.__aenter__.return_value.json = AsyncMock(
            return_value=mock_response
        )

        messages = [
            {"role": "user", "content": "previous message"},
            {"role": "assistant", "content": "previous response"},
        ]
        result = await llm.ask("test prompt", messages=messages)

        assert result is not None
        assert len(result.commands) == 3

        # Verify that message was included in request
        call_args = mock_post.call_args
        assert call_args is not None
        request_data = call_args[1]["json"]
        assert "message" in request_data
        assert request_data["message"] == "test prompt"


@pytest.mark.asyncio
async def test_real_api_response_format(llm, mock_real_api_response):
    """Test handling of the real API response format with markdown content"""
    with patch("aiohttp.ClientSession.post") as mock_post:
        mock_post.return_value.__aenter__.return_value.status = 200
        mock_post.return_value.__aenter__.return_value.json = AsyncMock(
            return_value=mock_real_api_response
        )

        result = await llm.ask("test prompt")
        assert result is not None
        assert len(result.commands) >= 5  # We should extract at least 5 commands

        # Verify some of the extracted commands
        forward_scan_commands = [
            cmd for cmd in result.commands if "forward scan" in cmd.type
        ]
        assert len(forward_scan_commands) > 0
        assert "obstacle detected" in forward_scan_commands[0].value.lower()

        target_lock_commands = [
            cmd for cmd in result.commands if "target lock" in cmd.type
        ]
        assert len(target_lock_commands) > 0
        assert "table" in target_lock_commands[0].type.lower()
        assert "approach" in target_lock_commands[0].value.lower()


@pytest.mark.asyncio
async def test_openai_format_compatibility(llm, mock_openai_format_response):
    """Test handling of pre-processed commands format for OpenAI compatibility"""
    with patch("aiohttp.ClientSession.post") as mock_post:
        mock_post.return_value.__aenter__.return_value.status = 200
        mock_post.return_value.__aenter__.return_value.json = AsyncMock(
            return_value=mock_openai_format_response
        )

        result = await llm.ask("test prompt")
        assert result is not None
        assert len(result.commands) == 3

        # Verify command values are preserved exactly as received
        assert result.commands[0].type == "move"
        assert result.commands[0].value == "sit"
        assert result.commands[1].type == "speak"
        assert "Hi there! I'm Spot" in result.commands[1].value
        assert result.commands[2].type == "emotion"
        assert result.commands[2].value == "joy"


# CLI test function for debugging real API responses
async def debug_real_api_response(api_key, prompt):
    """
    Debug function to test MultiLLM with real API calls

    Usage from command line:
    PYTHONPATH=/Users/ahmadkhan/OM1 python -c "import asyncio; from tests.llm.plugins.test_multi_llm import debug_real_api_response; asyncio.run(debug_real_api_response('your_api_key', 'your prompt'))"
    """
    import json
    import logging

    # Set up detailed logging
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )

    # Create config with API key
    config = LLMConfig(api_key=api_key, model="gemini-2.0-flash")
    llm = MultiLLM(DummyOutputModel, config)

    print(f"\nSending prompt: {prompt}")

    # Log raw request
    request_data = {"message": prompt, "model": "gemini-2.0-flash"}
    print(f"\nRequest data: {json.dumps(request_data, indent=2)}")

    # Send request
    result = await llm.ask(prompt)

    # Results
    if result:
        print("\nSuccess! Extracted commands:")
        for i, cmd in enumerate(result.commands):
            print(f"  {i+1}. {cmd.type}: {cmd.value}")
    else:
        print("\nError: Failed to get a valid response")
