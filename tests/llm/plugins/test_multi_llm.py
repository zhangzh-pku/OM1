from unittest.mock import patch, AsyncMock, MagicMock
import json

import pytest
import aiohttp

from llm import LLMConfig
from llm.output_model import CortexOutputModel
from llm.plugins.multi_llm import MultiLLM

@pytest.fixture
def config():
    return LLMConfig(
        base_url="https://api.openmind.org/api/core",
        api_key="test_api_key",
        model="gemini-2.0-flash",
    )


@pytest.fixture
def mock_response():
    """Mock API response with markdown content"""
    return {
        "structured_output": '{"commands":[{"type":"move","value":"wag tail"},{"type":"move","value":"walk"},{"type":"speak","value":"Woof! Hello there! I see you!"},{"type":"emotion","value":"joy"}]}'
    }


@pytest.fixture
def mock_structured_output_response():
    """Mock API response with structured output matching our model"""
    return {
        "structured_output": json.dumps({
            "commands": [
                {"type": "move", "value": "wag tail"},
                {"type": "move", "value": "walk"},
                {"type": "speak", "value": "Hello!"},
                {"type": "emotion", "value": "happy"}
            ]
        })
    }


@pytest.fixture
def llm(config):
    return MultiLLM(CortexOutputModel, config)


@pytest.fixture
def mocked_session(request):
    """Provides a mocked aiohttp ClientSession with configurable post response."""
    # Get parameter from parametrize. Can be data directly or a fixture name string
    param = getattr(request, "param", {})

    if isinstance(param, str):
        # If it's a string, assume it's a fixture name and resolve it
        response_data = request.getfixturevalue(param)
    else:
        # Otherwise, assume it's the data itself (like in test_ask_invalid_response)
        response_data = param

    mock_context_manager = AsyncMock()
    mock_response_obj = AsyncMock()
    mock_response_obj.json = AsyncMock(return_value=response_data)
    mock_response_obj.status = 200 # Default status
    mock_context_manager.__aenter__.return_value = mock_response_obj

    mock_session = AsyncMock(spec=aiohttp.ClientSession)
    mock_session.post.return_value = mock_context_manager
    mock_session.close = AsyncMock()

    return mock_session


def test_init_with_config(llm, config):
    """Test initialization with provided configuration"""
    assert llm._config.api_key == config.api_key
    assert llm.endpoint == "https://api.openmind.org/api/core/agent/robotic_team/runs"
    assert llm._config.model == config.model
    assert llm.session is None


@pytest.mark.asyncio
async def test_init_with_default_model():
    """Test that the MultiLLM uses the default model when not specified"""
    config = LLMConfig(api_key="test_key")
    llm = MultiLLM(CortexOutputModel, config)

    assert llm._config.model == "gemini-2.0-flash"

    await llm.close()


def test_init_empty_key():
    """Test initialization with missing API key"""
    config = LLMConfig(base_url="test_url")
    with pytest.raises(ValueError, match="config file missing api_key"):
        MultiLLM(CortexOutputModel, config)


@pytest.mark.asyncio
async def test_context_manager():
    """Test async context manager functionality"""
    config = LLMConfig(api_key="test_key", model="test-model")
    
    async with MultiLLM(CortexOutputModel, config) as llm:
        assert llm.session is not None
        assert isinstance(llm.session, aiohttp.ClientSession)
    
    assert llm.session is None


@pytest.mark.asyncio
@pytest.mark.parametrize("mocked_session", ["mock_response"], indirect=True)
async def test_ask_structured_output(llm, mocked_session):
    """Test handling of structured output response format"""
    with patch('aiohttp.ClientSession', return_value=mocked_session):
        result = await llm.ask("test prompt")

        assert result is not None
        assert isinstance(result, CortexOutputModel)


@pytest.mark.asyncio
async def test_ask_api_error(llm):
    """Test handling of API errors"""
    # Configure mock session to raise an error on post
    mock_session = AsyncMock(spec=aiohttp.ClientSession)
    mock_context_manager = AsyncMock()
    mock_context_manager.__aenter__.side_effect = aiohttp.ClientError("API Error")
    mock_session.post.return_value = mock_context_manager
    mock_session.close = AsyncMock()

    with patch('aiohttp.ClientSession', return_value=mock_session):
        result = await llm.ask("test prompt")
        assert result is None


@pytest.mark.asyncio
@pytest.mark.parametrize("mocked_session", ["mock_structured_output_response"], indirect=True)
async def test_io_provider_timing(llm, mocked_session):
    """Test timing metrics collection"""
    # The fixture sets up the response using mock_structured_output_response name
    with patch('aiohttp.ClientSession', return_value=mocked_session):
        result = await llm.ask("test prompt")

        assert result is not None 
        assert llm.io_provider.llm_start_time is not None
        assert llm.io_provider.llm_end_time is not None
        assert llm.io_provider.llm_end_time >= llm.io_provider.llm_start_time
