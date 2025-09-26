from unittest.mock import AsyncMock, MagicMock, patch

import pytest
from pydantic import BaseModel

from llm import LLMConfig
from llm.plugins.mistral_llm import MistralLLM


# Test output model
class DummyOutputModel(BaseModel):
    test_field: str
    optional_field: str = "default"


@pytest.fixture
def config():
    return LLMConfig(
        base_url="https://api.mistral.ai",
        api_key="test-mistral-key",
        model="mistral-large-latest",
        timeout=30,
    )


@pytest.fixture
def mock_response():
    """Fixture providing a valid mock API response"""
    response = MagicMock()
    response.choices = [
        MagicMock(
            message=MagicMock(
                content='{"test_field": "success", "optional_field": "test"}'
            )
        )
    ]
    return response


@pytest.fixture
def llm(config):
    with patch("llm.plugins.mistral_llm.Mistral"):
        return MistralLLM(DummyOutputModel, config)


@pytest.mark.asyncio
async def test_init_with_config():
    """Test initialization with provided configuration"""
    config = LLMConfig(
        api_key="test-key",
        model="mistral-medium",
        base_url="https://custom.mistral.ai",
    )
    
    with patch("llm.plugins.mistral_llm.Mistral") as MockMistral:
        llm = MistralLLM(DummyOutputModel, config)
        
        # Verify Mistral client was initialized with correct params
        MockMistral.assert_called_once_with(
            api_key="test-key",
            server_url="https://custom.mistral.ai",
        )
        assert llm._config.model == "mistral-medium"


@pytest.mark.asyncio
async def test_init_default_model():
    """Test default model when not specified"""
    config = LLMConfig(api_key="test-key")
    
    with patch("llm.plugins.mistral_llm.Mistral"):
        llm = MistralLLM(DummyOutputModel, config)
        assert llm._config.model == "mistral-large-latest"


@pytest.mark.asyncio
async def test_init_missing_api_key():
    """Test error when API key is missing"""
    config = LLMConfig()
    
    with pytest.raises(ValueError, match="config file missing api_key for Mistral"):
        MistralLLM(DummyOutputModel, config)


@pytest.mark.asyncio
async def test_ask_success(llm):
    """Test successful API request and response parsing"""
    mock_response = MagicMock()
    mock_response.choices = [
        MagicMock(
            message=MagicMock(
                content='{"test_field": "success", "optional_field": "test_value"}'
            )
        )
    ]
    
    # Mock the async wrapper's chat_complete method
    llm._async_client.chat_complete = AsyncMock(return_value=mock_response)
    
    result = await llm.ask("test prompt", messages=[{"role": "system", "content": "Be helpful"}])
    
    assert isinstance(result, DummyOutputModel)
    assert result.test_field == "success"
    assert result.optional_field == "test_value"
    
    # Verify the API was called with correct parameters
    llm._async_client.chat_complete.assert_called_once()
    call_args = llm._async_client.chat_complete.call_args
    
    # Check messages were formatted correctly
    messages = call_args[1]["messages"]
    assert len(messages) == 2
    assert messages[0] == {"role": "system", "content": "Be helpful"}
    assert messages[1] == {"role": "user", "content": "test prompt"}


@pytest.mark.asyncio
async def test_ask_invalid_json(llm):
    """Test handling of invalid JSON response"""
    mock_response = MagicMock()
    mock_response.choices = [
        MagicMock(message=MagicMock(content="This is not valid JSON"))
    ]
    
    llm._async_client.chat_complete = AsyncMock(return_value=mock_response)
    
    result = await llm.ask("test prompt")
    assert result is None


@pytest.mark.asyncio
async def test_ask_api_error(llm):
    """Test error handling for API exceptions"""
    llm._async_client.chat_complete = AsyncMock(
        side_effect=Exception("Mistral API error")
    )
    
    result = await llm.ask("test prompt")
    assert result is None


@pytest.mark.asyncio
async def test_ask_empty_response(llm):
    """Test handling of empty response from API"""
    mock_response = MagicMock()
    mock_response.choices = []
    
    llm._async_client.chat_complete = AsyncMock(return_value=mock_response)
    
    result = await llm.ask("test prompt")
    assert result is None


@pytest.mark.asyncio
async def test_message_formatting(llm):
    """Test proper formatting of different message types"""
    mock_response = MagicMock()
    mock_response.choices = [
        MagicMock(message=MagicMock(content='{"test_field": "formatted"}'))
    ]
    
    llm._async_client.chat_complete = AsyncMock(return_value=mock_response)
    
    messages = [
        {"role": "system", "content": "System message"},
        {"role": "user", "content": "User message"},
        {"role": "assistant", "content": "Assistant response"},
    ]
    
    await llm.ask("current prompt", messages=messages)
    
    # Check that messages were properly formatted
    call_args = llm._async_client.chat_complete.call_args
    formatted_messages = call_args[1]["messages"]
    
    assert len(formatted_messages) == 4  # 3 history + 1 current
    assert formatted_messages[0]["role"] == "system"
    assert formatted_messages[1]["role"] == "user"
    assert formatted_messages[2]["role"] == "assistant"
    assert formatted_messages[3]["role"] == "user"
    assert formatted_messages[3]["content"] == "current prompt"


@pytest.mark.asyncio
async def test_io_provider_timing(llm):
    """Test timing metrics collection"""
    mock_response = MagicMock()
    mock_response.choices = [
        MagicMock(message=MagicMock(content='{"test_field": "timing_test"}'))
    ]
    
    llm._async_client.chat_complete = AsyncMock(return_value=mock_response)
    
    await llm.ask("test prompt")
    
    assert llm.io_provider.llm_start_time is not None
    assert llm.io_provider.llm_end_time is not None
    assert llm.io_provider.llm_end_time >= llm.io_provider.llm_start_time


@pytest.mark.asyncio
async def test_response_format_with_schema(llm):
    """Test that response format with schema is properly passed"""
    mock_response = MagicMock()
    mock_response.choices = [
        MagicMock(message=MagicMock(content='{"test_field": "schema_test"}'))
    ]
    
    llm._async_client.chat_complete = AsyncMock(return_value=mock_response)
    
    await llm.ask("test prompt")
    
    # Verify response_format was included with schema
    call_args = llm._async_client.chat_complete.call_args
    response_format = call_args[1]["response_format"]
    
    assert response_format["type"] == "json_object"
    assert "schema" in response_format
    assert response_format["schema"]["properties"]["test_field"]