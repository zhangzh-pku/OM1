from unittest.mock import AsyncMock, MagicMock, Mock, patch
import sys

import pytest
from pydantic import BaseModel

from llm import LLMConfig

# Mock llama_api module before importing LlamaLLM
sys.modules['llama_api'] = MagicMock()

from llm.plugins.llama_llm import LlamaLLM


# Test output model
class DummyOutputModel(BaseModel):
    test_field: str
    optional_field: str = "default"


@pytest.fixture
def config():
    return LLMConfig(
        api_key="test-llama-key",
        model="llama-3.2-3b-instruct",
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
    # The llama_api module is already mocked at module level
    return LlamaLLM(DummyOutputModel, config)


@pytest.mark.asyncio
async def test_init_with_config():
    """Test initialization with provided configuration"""
    config = LLMConfig(
        api_key="test-key",
        model="llama-3.1-8b",
    )
    
    llm = LlamaLLM(DummyOutputModel, config)
    assert llm._config.model == "llama-3.1-8b"


@pytest.mark.asyncio
async def test_init_with_base_url():
    """Test initialization with custom base URL"""
    config = LLMConfig(
        api_key="test-key",
        model="llama-3.2-1b",
        base_url="http://localhost:11434",
    )
    
    llm = LlamaLLM(DummyOutputModel, config)
    assert llm._config.base_url == "http://localhost:11434"


@pytest.mark.asyncio
async def test_init_default_model():
    """Test default model when not specified"""
    config = LLMConfig(api_key="test-key")
    
    llm = LlamaLLM(DummyOutputModel, config)
    assert llm._config.model == "llama-3.2-3b-instruct"


@pytest.mark.asyncio
async def test_init_missing_api_key():
    """Test error when API key is missing"""
    config = LLMConfig()
    
    with pytest.raises(ValueError, match="config file missing api_key for Llama"):
        LlamaLLM(DummyOutputModel, config)


@pytest.mark.asyncio
async def test_init_missing_llama_api():
    """Test error when llama-api-python is not installed"""
    config = LLMConfig(api_key="test-key")
    
    # Test ImportError handling in __init__
    with patch.object(sys.modules['llama_api'], 'Client', side_effect=ImportError):
        with pytest.raises(ImportError):
            LlamaLLM(DummyOutputModel, config)


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
    
    result = await llm.ask("test prompt")
    
    assert isinstance(result, DummyOutputModel)
    assert result.test_field == "success"
    assert result.optional_field == "test_value"


@pytest.mark.asyncio
async def test_ask_with_system_message(llm):
    """Test handling of system messages"""
    mock_response = MagicMock()
    mock_response.choices = [
        MagicMock(message=MagicMock(content='{"test_field": "system_test"}'))
    ]
    
    llm._async_client.chat_complete = AsyncMock(return_value=mock_response)
    
    messages = [
        {"role": "system", "content": "You are a helpful assistant"},
        {"role": "user", "content": "Previous question"},
    ]
    
    await llm.ask("current prompt", messages=messages)
    
    # Check that system message was prepended to user message
    call_args = llm._async_client.chat_complete.call_args
    sent_messages = call_args[1]["messages"]
    
    # Should have 2 messages: previous user message and current with system prepended
    assert len(sent_messages) == 2
    assert "You are a helpful assistant" in sent_messages[1]["content"]
    assert "current prompt" in sent_messages[1]["content"]


@pytest.mark.asyncio
async def test_ask_json_extraction_from_markdown(llm):
    """Test extraction of JSON from markdown code blocks"""
    mock_response = MagicMock()
    mock_response.choices = [
        MagicMock(
            message=MagicMock(
                content='Here is the JSON:\n```json\n{"test_field": "markdown_test"}\n```'
            )
        )
    ]
    
    llm._async_client.chat_complete = AsyncMock(return_value=mock_response)
    
    result = await llm.ask("test prompt")
    
    assert isinstance(result, DummyOutputModel)
    assert result.test_field == "markdown_test"


@pytest.mark.asyncio
async def test_ask_json_extraction_raw(llm):
    """Test extraction of raw JSON from response"""
    mock_response = MagicMock()
    mock_response.choices = [
        MagicMock(
            message=MagicMock(
                content='Some text before {"test_field": "raw_json"} and after'
            )
        )
    ]
    
    llm._async_client.chat_complete = AsyncMock(return_value=mock_response)
    
    result = await llm.ask("test prompt")
    
    assert isinstance(result, DummyOutputModel)
    assert result.test_field == "raw_json"


@pytest.mark.asyncio
async def test_ask_invalid_json(llm):
    """Test handling of invalid JSON response"""
    mock_response = MagicMock()
    mock_response.choices = [
        MagicMock(message=MagicMock(content="This is not JSON at all"))
    ]
    
    llm._async_client.chat_complete = AsyncMock(return_value=mock_response)
    
    result = await llm.ask("test prompt")
    assert result is None


@pytest.mark.asyncio
async def test_ask_api_error(llm):
    """Test error handling for API exceptions"""
    llm._async_client.chat_complete = AsyncMock(
        side_effect=Exception("Llama API error")
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
async def test_extra_params_passed(llm):
    """Test that extra parameters are passed to API"""
    mock_response = MagicMock()
    mock_response.choices = [
        MagicMock(message=MagicMock(content='{"test_field": "params_test"}'))
    ]
    
    llm._config.extra_params = {"temperature": 0.9, "max_tokens": 2048}
    llm._async_client.chat_complete = AsyncMock(return_value=mock_response)
    
    await llm.ask("test prompt")
    
    # Verify extra params were passed
    call_args = llm._async_client.chat_complete.call_args
    assert call_args[1]["temperature"] == 0.9
    assert call_args[1]["max_tokens"] == 2048


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
async def test_schema_instruction_included(llm):
    """Test that schema instruction is included in the prompt"""
    mock_response = MagicMock()
    mock_response.choices = [
        MagicMock(message=MagicMock(content='{"test_field": "schema_test"}'))
    ]
    
    llm._async_client.chat_complete = AsyncMock(return_value=mock_response)
    
    await llm.ask("test prompt")
    
    # Check that schema instruction was added
    call_args = llm._async_client.chat_complete.call_args
    messages = call_args[1]["messages"]
    
    # The schema should be in the user message
    assert "must respond with valid JSON" in messages[0]["content"]
    assert "test_field" in messages[0]["content"]  # Schema should contain our field