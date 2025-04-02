from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from pydantic import BaseModel, ValidationError

from llm import LLMConfig
from llm.plugins.multi_llm import MultiLLM, RoboticTeamRequest, RoboticTeamResponse


# Test output model matching the format we return
class DummyOutputModel(BaseModel):
    content: str
    model_used: str
    agent_type: str


@pytest.fixture
def config():
    return LLMConfig(
        base_url="https://api.openmind.org/api/core",
        api_key="test_api_key",
        model="gemini-2.0-flash",
    )


@pytest.fixture
def mock_response():
    # Simulate the response from the robotic_team endpoint
    return {"content": "This is a response from the robotic team's specialist agent."}


@pytest.fixture
def llm(config):
    return MultiLLM(DummyOutputModel, config)


@pytest.mark.asyncio
async def test_init_with_config(llm, config):
    """Test that the MultiLLM initializes correctly with the given config"""
    assert llm.base_url == config.base_url
    assert llm.endpoint == f"{config.base_url}/agent/robotic_team/runs"
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
    """Test successful API call to the robotic_team endpoint"""
    # Mock the aiohttp ClientSession
    mock_session = MagicMock()
    mock_context = MagicMock()

    # Configure the mocks for a successful response
    mock_session.__aenter__.return_value = mock_session
    mock_context.__aenter__.return_value = mock_context
    mock_context.status = 200
    mock_context.json = AsyncMock(return_value=mock_response)
    mock_session.post = MagicMock(return_value=mock_context)

    # Test the ask method with a mocked HTTP session
    with patch("aiohttp.ClientSession", return_value=mock_session):
        result = await llm.ask("How do I navigate around obstacles?")

        # Verify the request payload
        mock_session.post.assert_called_once()
        args, kwargs = mock_session.post.call_args
        assert args[0] == llm.endpoint
        assert kwargs["json"]["model"] == llm._config.model
        assert kwargs["json"]["message"] == "How do I navigate around obstacles?"
        assert kwargs["headers"]["Authorization"] == f"Bearer {llm._config.api_key}"

        # Verify the response parsing
        assert isinstance(result, DummyOutputModel)
        assert result.content == mock_response["content"]
        assert result.model_used == llm._config.model
        assert result.agent_type == "robotic_team"


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
    """Test that the ask method works with message history (even though not used by endpoint)"""
    # Mock the aiohttp ClientSession
    mock_session = MagicMock()
    mock_context = MagicMock()

    # Configure the mocks
    mock_session.__aenter__.return_value = mock_session
    mock_context.__aenter__.return_value = mock_context
    mock_context.status = 200
    mock_context.json = AsyncMock(return_value=mock_response)
    mock_session.post = MagicMock(return_value=mock_context)

    # Messages that would be ignored by the endpoint but should not break our code
    messages = [
        {"role": "user", "content": "Previous message"},
        {"role": "assistant", "content": "Previous response"},
    ]

    # Test that messages are accepted but don't affect the request
    with patch("aiohttp.ClientSession", return_value=mock_session):
        result = await llm.ask("How do I navigate around obstacles?", messages)

        # Verify only the current prompt is sent in the payload
        args, kwargs = mock_session.post.call_args
        assert kwargs["json"]["message"] == "How do I navigate around obstacles?"
        assert "messages" not in kwargs["json"]

        # Verify normal response handling
        assert isinstance(result, DummyOutputModel)
        assert result.content == mock_response["content"]


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
async def test_response_validation():
    """Test that the response model properly validates input"""
    # Valid response
    response = RoboticTeamResponse(content="test content")
    assert response.content == "test content"

    # Invalid response - missing content
    with pytest.raises(ValidationError):
        RoboticTeamResponse()

    # Invalid response - wrong type
    with pytest.raises(ValidationError):
        RoboticTeamResponse(content=123)  # content should be string


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
