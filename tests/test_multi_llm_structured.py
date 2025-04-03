import asyncio
import json
import os
from unittest import mock

import pytest
from pydantic import BaseModel, Field

from llm import LLMConfig
from llm.output_model import Command, CortexOutputModel
from llm.plugins.multi_llm import MultiLLM


# Mock response for the API request
MOCK_STRUCTURED_RESPONSE = {
    "content": "Robot has performed the navigation task.",
    "structured_output": {
        "commands": [
            {"type": "move", "value": "forward 2 meters"},
            {"type": "turn", "value": "right 90 degrees"},
            {"type": "speak", "value": "Navigation complete"}
        ]
    }
}

# Mock response for non-structured format
MOCK_CONTENT_RESPONSE = {
    "content": "Robot has performed the navigation task.\n\n- move: forward 2 meters\n- turn: right 90 degrees\n- speak: Navigation complete"
}


class MockResponse:
    def __init__(self, data, status=200):
        self.data = data
        self.status = status

    async def json(self):
        return self.data

    async def text(self):
        return json.dumps(self.data)


class MockClientSession:
    def __init__(self, response):
        self.response = response
        self.post_args = None
        self.post_kwargs = None

    async def __aenter__(self):
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        pass

    # Return an object that acts as an async context manager
    def post(self, *args, **kwargs):
        self.post_args = args
        self.post_kwargs = kwargs
        
        # Create a context manager for post response
        class AsyncContextManager:
            def __init__(self, response):
                self.response = response

            async def __aenter__(self):
                return self.response
                
            async def __aexit__(self, exc_type, exc_val, exc_tb):
                pass
                
        return AsyncContextManager(self.response)


@pytest.mark.asyncio
async def test_multi_llm_structured_output():
    """Test that the MultiLLM plugin correctly handles structured outputs."""
    # Create a mock response
    mock_response = MockResponse(MOCK_STRUCTURED_RESPONSE)
    mock_session = MockClientSession(mock_response)

    # Create a MultiLLM instance
    config = LLMConfig(api_key="test_key", model="gemini-2.0-flash")
    multi_llm = MultiLLM(CortexOutputModel, config)

    # Mock the aiohttp.ClientSession to return our mock response
    with mock.patch("aiohttp.ClientSession", return_value=mock_session):
        result = await multi_llm.ask("Navigate to the kitchen")

    # Verify the request payload
    assert mock_session.post_kwargs is not None
    json_payload = mock_session.post_kwargs.get("json")
    assert json_payload is not None
    assert json_payload["message"] == "Navigate to the kitchen"
    assert json_payload["model"] == "gemini-2.0-flash"
    assert json_payload["structured_outputs"] is True
    assert "response_model" in json_payload

    # Verify the result
    assert result is not None
    assert isinstance(result, CortexOutputModel)
    assert len(result.commands) == 3
    assert result.commands[0].type == "move"
    assert result.commands[0].value == "forward 2 meters"
    assert result.commands[1].type == "turn"
    assert result.commands[1].value == "right 90 degrees"
    assert result.commands[2].type == "speak"
    assert result.commands[2].value == "Navigation complete"


@pytest.mark.asyncio
async def test_multi_llm_content_fallback():
    """Test that the MultiLLM plugin correctly handles content-only responses."""
    # Create a mock response
    mock_response = MockResponse(MOCK_CONTENT_RESPONSE)
    mock_session = MockClientSession(mock_response)

    # Create a MultiLLM instance
    config = LLMConfig(api_key="test_key", model="gemini-2.0-flash")
    multi_llm = MultiLLM(CortexOutputModel, config)

    # Mock the aiohttp.ClientSession to return our mock response
    with mock.patch("aiohttp.ClientSession", return_value=mock_session):
        result = await multi_llm.ask("Navigate to the kitchen")

    # Verify the result
    assert result is not None
    assert isinstance(result, CortexOutputModel)
    assert len(result.commands) > 0
    
    # Should extract commands from the markdown content
    commands_dict = {cmd.type: cmd.value for cmd in result.commands}
    assert "move" in commands_dict
    assert "turn" in commands_dict
    assert "speak" in commands_dict


if __name__ == "__main__":
    asyncio.run(test_multi_llm_structured_output())
    asyncio.run(test_multi_llm_content_fallback()) 