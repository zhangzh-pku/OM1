#!/usr/bin/env python3
"""
Demo script for testing Mistral and Meta Llama LLM integrations.

This script demonstrates:
1. Text generation with both providers
2. Dialog/conversation example
3. Switching between providers
4. Custom configuration examples
"""

import asyncio
import os
import sys
from typing import List, Dict, Any
from pydantic import BaseModel
from dotenv import load_dotenv

# Add src to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from llm import LLMConfig, load_llm


# Define output models for different demo scenarios
class SimpleResponse(BaseModel):
    """Simple text response model"""
    answer: str
    confidence: float = 1.0


class ConversationResponse(BaseModel):
    """Conversation response with context awareness"""
    response: str
    intent: str
    requires_followup: bool = False


class CodeGeneration(BaseModel):
    """Code generation response"""
    language: str
    code: str
    explanation: str


async def demo_text_generation(provider_name: str, config: LLMConfig):
    """Demonstrate simple text generation"""
    print(f"\n{'='*60}")
    print(f"Demo: Text Generation with {provider_name}")
    print('='*60)
    
    try:
        # Load the LLM
        llm_class = load_llm(provider_name)
        llm = llm_class(SimpleResponse, config)
        
        # Test prompts
        prompts = [
            "What is the capital of France? Provide a brief answer.",
            "Explain quantum computing in one sentence.",
            "What's 15% of 240? Show just the number.",
        ]
        
        for prompt in prompts:
            print(f"\nPrompt: {prompt}")
            result = await llm.ask(prompt)
            if result:
                print(f"Response: {result.answer}")
                print(f"Confidence: {result.confidence}")
            else:
                print("Error: Failed to get response")
                
    except Exception as e:
        print(f"Error with {provider_name}: {e}")


async def demo_conversation(provider_name: str, config: LLMConfig):
    """Demonstrate conversation with context"""
    print(f"\n{'='*60}")
    print(f"Demo: Conversation with {provider_name}")
    print('='*60)
    
    try:
        # Load the LLM
        llm_class = load_llm(provider_name)
        llm = llm_class(ConversationResponse, config)
        
        # Conversation history
        messages = [
            {"role": "system", "content": "You are a helpful AI assistant."},
        ]
        
        # Conversation turns
        turns = [
            "I'm planning a trip to Japan. What should I know?",
            "What about the food there?",
            "How much should I budget per day?",
        ]
        
        for turn in turns:
            print(f"\nUser: {turn}")
            
            result = await llm.ask(turn, messages)
            if result:
                print(f"Assistant: {result.response}")
                print(f"Intent: {result.intent}")
                print(f"Requires followup: {result.requires_followup}")
                
                # Add to conversation history
                messages.append({"role": "user", "content": turn})
                messages.append({"role": "assistant", "content": result.response})
            else:
                print("Error: Failed to get response")
                break
                
    except Exception as e:
        print(f"Error with {provider_name}: {e}")


async def demo_code_generation(provider_name: str, config: LLMConfig):
    """Demonstrate code generation"""
    print(f"\n{'='*60}")
    print(f"Demo: Code Generation with {provider_name}")
    print('='*60)
    
    try:
        # Load the LLM
        llm_class = load_llm(provider_name)
        llm = llm_class(CodeGeneration, config)
        
        prompt = "Write a Python function to calculate the factorial of a number using recursion."
        
        print(f"\nPrompt: {prompt}")
        result = await llm.ask(prompt)
        
        if result:
            print(f"\nLanguage: {result.language}")
            print(f"Code:\n{result.code}")
            print(f"\nExplanation: {result.explanation}")
        else:
            print("Error: Failed to get response")
            
    except Exception as e:
        print(f"Error with {provider_name}: {e}")


async def demo_provider_switching():
    """Demonstrate switching between providers"""
    print(f"\n{'='*60}")
    print("Demo: Provider Switching")
    print('='*60)
    
    # Load environment variables
    load_dotenv()
    
    # Configure different providers
    providers = []
    
    # Mistral configuration
    if os.getenv("MISTRAL_API_KEY"):
        providers.append({
            "name": "MistralLLM",
            "config": LLMConfig(
                api_key=os.getenv("MISTRAL_API_KEY"),
                model="mistral-large-latest",
                timeout=30,
            )
        })
    
    # Llama configuration
    if os.getenv("LLAMA_API_KEY"):
        providers.append({
            "name": "LlamaLLM",
            "config": LLMConfig(
                api_key=os.getenv("LLAMA_API_KEY"),
                model="llama-3.2-3b-instruct",
                timeout=30,
            )
        })
    
    # OpenAI configuration (for comparison)
    if os.getenv("OPENAI_API_KEY"):
        providers.append({
            "name": "OpenAILLM",
            "config": LLMConfig(
                api_key=os.getenv("OPENAI_API_KEY"),
                model="gpt-4o-mini",
                timeout=30,
            )
        })
    
    if not providers:
        print("\nNo API keys found in environment variables.")
        print("Please set one or more of: MISTRAL_API_KEY, LLAMA_API_KEY, OPENAI_API_KEY")
        return
    
    # Test the same prompt with different providers
    prompt = "What are the three most important considerations when designing a REST API?"
    
    for provider_info in providers:
        print(f"\n--- Using {provider_info['name']} ---")
        
        try:
            llm_class = load_llm(provider_info['name'])
            llm = llm_class(SimpleResponse, provider_info['config'])
            
            result = await llm.ask(prompt)
            if result:
                print(f"Response: {result.answer}")
            else:
                print("Failed to get response")
                
        except Exception as e:
            print(f"Error: {e}")


async def demo_local_deployment():
    """Demonstrate configuration for local/self-hosted models"""
    print(f"\n{'='*60}")
    print("Demo: Local/Self-Hosted Configuration")
    print('='*60)
    
    # Example configurations for local deployments
    local_configs = [
        {
            "name": "Local Mistral (Ollama)",
            "provider": "MistralLLM",
            "config": LLMConfig(
                base_url="http://localhost:11434/v1",  # Ollama with OpenAI compatibility
                api_key="dummy-key",  # Local deployments often don't need real keys
                model="mistral:latest",
                timeout=60,
            )
        },
        {
            "name": "Local Llama (vLLM)",
            "provider": "LlamaLLM",
            "config": LLMConfig(
                base_url="http://localhost:8000",  # vLLM server
                api_key="dummy-key",
                model="meta-llama/Llama-3.2-3B-Instruct",
                timeout=60,
            )
        },
    ]
    
    print("\nExample configurations for local deployments:")
    for config_info in local_configs:
        print(f"\n{config_info['name']}:")
        print(f"  Provider: {config_info['provider']}")
        print(f"  Base URL: {config_info['config'].base_url}")
        print(f"  Model: {config_info['config'].model}")
        
    print("\nNote: To test these, ensure your local server is running.")


async def main():
    """Main demo function"""
    print("="*60)
    print("OM1 LLM Provider Integration Demo")
    print("="*60)
    
    # Load environment variables
    load_dotenv()
    
    # Check available providers
    available_providers = []
    
    if os.getenv("MISTRAL_API_KEY"):
        available_providers.append(("MistralLLM", LLMConfig(
            api_key=os.getenv("MISTRAL_API_KEY"),
            model="mistral-large-latest",
        )))
    
    if os.getenv("LLAMA_API_KEY"):
        available_providers.append(("LlamaLLM", LLMConfig(
            api_key=os.getenv("LLAMA_API_KEY"),
            model="llama-3.2-3b-instruct",
        )))
    
    if not available_providers:
        print("\nNo API keys found. Running in demo mode with mock configurations.")
        print("\nTo run actual tests, create a .env file with:")
        print("MISTRAL_API_KEY=your-mistral-key")
        print("LLAMA_API_KEY=your-llama-key")
        print("\nShowing configuration examples instead...")
        
        # Show configuration examples
        await demo_local_deployment()
        return
    
    # Run demos for available providers
    for provider_name, config in available_providers:
        print(f"\n\n{'#'*60}")
        print(f"# Testing {provider_name}")
        print('#'*60)
        
        # Run different demo scenarios
        await demo_text_generation(provider_name, config)
        await demo_conversation(provider_name, config)
        await demo_code_generation(provider_name, config)
    
    # Demo provider switching
    await demo_provider_switching()
    
    print("\n" + "="*60)
    print("Demo completed successfully!")
    print("="*60)


if __name__ == "__main__":
    asyncio.run(main())