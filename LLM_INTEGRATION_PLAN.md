# LLM Integration Implementation Plan

## Overview
This document outlines the step-by-step implementation plan for adding Meta Llama and Mistral LLM support to the OM1 project.

## Project Structure
- **Plugin Directory**: `src/llm/plugins/`
- **Base Classes**: `src/llm/__init__.py` (LLM, LLMConfig)
- **Existing Implementations**: OpenAI, Gemini, DeepSeek, xAI

## Implementation Steps

### Step 1: Install Dependencies
Add the following packages to `pyproject.toml`:
- `mistralai` - Official Mistral SDK
- `llama-api-python` - Official Meta Llama SDK

### Step 2: Implement MistralLLM Plugin
Create `src/llm/plugins/mistral_llm.py` with:
- Class inheriting from `LLM[R]`
- Initialize Mistral client with API key and base URL
- Implement `ask()` method for chat completions
- Support streaming responses
- Handle system messages and tool calls

### Step 3: Implement LlamaLLM Plugin
Create `src/llm/plugins/llama_llm.py` with:
- Class inheriting from `LLM[R]`
- Initialize Llama client with API key
- Implement `ask()` method using official SDK
- Support streaming responses
- Handle system messages appropriately

### Step 4: Configuration Support
Update configuration to support:
- Provider selection via `type` field (e.g., "MistralLLM", "LlamaLLM")
- Custom `base_url` for self-hosted endpoints
- API key configuration
- Model selection (e.g., "mistral-large-latest", "llama-3.2-3b-instruct")

### Step 5: Create Tests
Implement comprehensive tests in `tests/llm/` for:
- Basic initialization
- API calls with mocked responses
- Error handling
- Streaming support
- Message formatting

### Step 6: Create Demo Script
Build demonstration script showing:
- Text generation with both providers
- Dialog/conversation example
- Switching between providers
- Custom configuration examples

## Technical Details

### MistralLLM Implementation
```python
# Key features:
- Use mistralai.Mistral client
- Support models: mistral-large-latest, mistral-medium, etc.
- Convert OM1 messages to Mistral format
- Parse responses to Pydantic models
```

### LlamaLLM Implementation
```python
# Key features:
- Use llama_api.Client
- Support models: llama-3.2-3b-instruct, etc.
- Handle Meta's specific message format
- Parse responses to Pydantic models
```

### Configuration Examples
```python
# Mistral configuration
config = LLMConfig(
    type="MistralLLM",
    api_key="your-mistral-api-key",
    model="mistral-large-latest",
    base_url="https://api.mistral.ai"  # Optional
)

# Meta Llama configuration
config = LLMConfig(
    type="LlamaLLM",
    api_key="your-llama-api-key",
    model="llama-3.2-3b-instruct"
)

# Local/self-hosted configuration
config = LLMConfig(
    type="MistralLLM",
    base_url="http://localhost:8080",
    api_key="local-key",
    model="mistral-7b"
)
```

## Testing Strategy
1. Unit tests for each plugin
2. Integration tests with mocked API responses
3. Manual testing with actual APIs (if keys available)
4. Performance benchmarks
5. Error recovery testing

## Deliverables
1. ✅ `mistral_llm.py` - Mistral plugin implementation
2. ✅ `llama_llm.py` - Meta Llama plugin implementation
3. ✅ Updated `pyproject.toml` with new dependencies
4. ✅ Test suite for both plugins
5. ✅ Demo script with examples
6. ✅ Configuration documentation

## Success Criteria
- Both plugins successfully integrate with OM1 runtime
- Tests pass with >90% coverage
- Demo script runs without errors
- Documentation is clear and complete
- PR meets project standards