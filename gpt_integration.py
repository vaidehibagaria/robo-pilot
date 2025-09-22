#!/usr/bin/env python3
"""
GPT Integration Module - Handles OpenAI API communication for the RCM framework
"""

import os
import json
import logging
from typing import Dict, Any, List, Optional
from dataclasses import dataclass
import asyncio
from openai import AsyncOpenAI
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)

@dataclass
class GPTConfig:
    """Configuration for GPT integration"""
    api_key: str
    model: str = "gpt-4-turbo-preview"
    max_tokens: int = 1000
    temperature: float = 0.1
    timeout: int = 30

class GPTIntegration:
    """Handles communication with OpenAI GPT API"""
    
    def __init__(self, config: Optional[GPTConfig] = None):
        """Initialize GPT integration with configuration"""
        if config:
            self.config = config
        else:
            # Load from environment variables
            api_key = os.getenv("OPENAI_API_KEY")
            if not api_key:
                raise ValueError("OPENAI_API_KEY environment variable is required")
            
            self.config = GPTConfig(
                api_key=api_key,
                model=os.getenv("OPENAI_MODEL", "gpt-4-turbo-preview"),
                max_tokens=int(os.getenv("OPENAI_MAX_TOKENS", "1000")),
                temperature=float(os.getenv("OPENAI_TEMPERATURE", "0.1")),
                timeout=int(os.getenv("OPENAI_TIMEOUT", "30"))
            )
        
        self.client = AsyncOpenAI(api_key=self.config.api_key)
        logger.info(f"Initialized GPT integration with model: {self.config.model}")
    
    async def chat_completion(
        self,
        messages: List[Dict[str, str]],
        tools: Optional[List[Dict[str, Any]]] = None,
        tool_choice: str = "auto"
    ) -> Dict[str, Any]:
        """
        Send chat completion request to OpenAI API
        
        Args:
            messages: List of message objects with 'role' and 'content'
            tools: Optional list of tool definitions
            tool_choice: Tool choice strategy ("auto", "none", or specific tool)
            
        Returns:
            Dict containing the response from OpenAI
        """
        try:
            # Prepare request parameters
            request_params = {
                "model": self.config.model,
                "messages": messages,
                "max_tokens": self.config.max_tokens,
                "temperature": self.config.temperature,
                "timeout": self.config.timeout
            }
            
            # Add tools if provided
            if tools:
                request_params["tools"] = tools
                request_params["tool_choice"] = tool_choice
            
            logger.debug(f"Sending request to OpenAI: {json.dumps(request_params, indent=2)}")
            
            # Make API call
            response = await self.client.chat.completions.create(**request_params)
            
            # Convert response to dict for easier handling
            response_dict = {
                "id": response.id,
                "model": response.model,
                "choices": []
            }
            
            for choice in response.choices:
                choice_dict = {
                    "index": choice.index,
                    "message": {
                        "role": choice.message.role,
                        "content": choice.message.content
                    },
                    "finish_reason": choice.finish_reason
                }
                
                # Add tool calls if present
                if hasattr(choice.message, 'tool_calls') and choice.message.tool_calls:
                    choice_dict["message"]["tool_calls"] = []
                    for tool_call in choice.message.tool_calls:
                        tool_call_dict = {
                            "id": tool_call.id,
                            "type": tool_call.type,
                            "function": {
                                "name": tool_call.function.name,
                                "arguments": tool_call.function.arguments
                            }
                        }
                        choice_dict["message"]["tool_calls"].append(tool_call_dict)
                
                response_dict["choices"].append(choice_dict)
            
            logger.debug(f"Received response from OpenAI: {json.dumps(response_dict, indent=2)}")
            return response_dict
            
        except Exception as e:
            logger.error(f"Error in GPT API call: {e}")
            raise
    
    async def process_robot_command(
        self,
        system_prompt: str,
        user_message: str,
        available_tools: List[Dict[str, Any]],
        conversation_history: Optional[List[Dict[str, str]]] = None
    ) -> Dict[str, Any]:
        """
        Process a robot command using GPT with tool calling
        
        Args:
            system_prompt: System prompt describing robot capabilities
            user_message: User's command/message
            available_tools: List of available robot tools
            conversation_history: Optional previous conversation context
            
        Returns:
            Dict containing GPT response with potential tool calls
        """
        # Build message history
        messages = [{"role": "system", "content": system_prompt}]
        
        # Add conversation history if provided
        if conversation_history:
            messages.extend(conversation_history)
        
        # Add current user message
        messages.append({"role": "user", "content": user_message})
        
        # Make API call with tools
        response = await self.chat_completion(
            messages=messages,
            tools=available_tools,
            tool_choice="auto"
        )
        
        return response
    
    def validate_api_key(self) -> bool:
        """
        Validate that the API key is properly configured
        
        Returns:
            True if API key appears valid, False otherwise
        """
        if not self.config.api_key:
            return False
        
        # Basic validation - OpenAI keys start with 'sk-'
        if not self.config.api_key.startswith('sk-'):
            logger.warning("API key doesn't match expected OpenAI format")
            return False
        
        return True
    
    async def test_connection(self) -> bool:
        """
        Test connection to OpenAI API
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            response = await self.chat_completion(
                messages=[{"role": "user", "content": "Hello, this is a connection test."}]
            )
            return response is not None and "choices" in response
        except Exception as e:
            logger.error(f"Connection test failed: {e}")
            return False

# Utility functions for easy integration
def create_gpt_integration(api_key: Optional[str] = None) -> GPTIntegration:
    """
    Create a GPT integration instance
    
    Args:
        api_key: Optional API key, will use environment variable if not provided
        
    Returns:
        GPTIntegration instance
    """
    if api_key:
        config = GPTConfig(api_key=api_key)
        return GPTIntegration(config)
    else:
        return GPTIntegration()

def format_tools_for_openai(tool_schemas: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """
    Format tool schemas for OpenAI API
    
    Args:
        tool_schemas: Tool schemas from RCMToolGenerator
        
    Returns:
        Formatted tools list for OpenAI
    """
    return tool_schemas  # Tool schemas are already in OpenAI format

# Example usage
async def main():
    """Example usage of GPT integration"""
    try:
        # Create integration
        gpt = create_gpt_integration()
        
        # Test connection
        if await gpt.test_connection():
            print("✓ GPT API connection successful")
        else:
            print("✗ GPT API connection failed")
            return
        
        # Example robot command processing
        system_prompt = """I am a robot assistant with the following capabilities:
- Mobile base with differential drive
- LIDAR sensor
- Can move forward, turn, and stop

Available tools: drive_straight, rotate_in_place, stop, read_lidar_base_scan"""
        
        tools = [
            {
                "type": "function",
                "function": {
                    "name": "drive_straight",
                    "description": "Drive the robot straight forward or backward",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "distance_m": {"type": "number", "description": "Distance to drive in meters"},
                            "speed_mps": {"type": "number", "description": "Speed in meters per second"}
                        },
                        "required": ["distance_m"]
                    }
                }
            }
        ]
        
        response = await gpt.process_robot_command(
            system_prompt=system_prompt,
            user_message="Move forward 2 meters",
            available_tools=tools
        )
        
        print(f"GPT Response: {json.dumps(response, indent=2)}")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    asyncio.run(main())
