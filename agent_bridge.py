#!/usr/bin/env python3
"""
Agent Bridge - Connects LLMs to robot capabilities through RCM
"""

import json
import logging
import asyncio
import httpx
from typing import Dict, Any, List, Optional
from dataclasses import dataclass

from tool_generator import RCMToolGenerator
from gpt_integration import GPTIntegration, create_gpt_integration

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class LLMMessage:
    """Represents a message to/from LLM"""
    role: str  # "system", "user", "assistant", "tool"
    content: str
    tool_calls: Optional[List[Dict]] = None
    tool_call_id: Optional[str] = None

class AgentBridge:
    """Bridge between LLM and robot capabilities"""
    
    def __init__(self, rcm_server_url: str = "http://localhost:8000", api_key: Optional[str] = None):
        self.rcm_server_url = rcm_server_url
        self.tool_generator = RCMToolGenerator()
        self.robot_id: Optional[str] = None
        self.conversation_history: List[LLMMessage] = []
        
        # Initialize GPT integration
        try:
            self.gpt = create_gpt_integration(api_key)
            self.use_gpt = True
            logger.info("GPT integration initialized successfully")
        except Exception as e:
            logger.warning(f"GPT integration failed to initialize: {e}. Falling back to mock responses.")
            self.gpt = None
            self.use_gpt = False
        
    async def connect_robot(self, robot_id: str):
        """Connect to a robot and load its capabilities"""
        self.robot_id = robot_id
        
        try:
            async with httpx.AsyncClient() as client:
                response = await client.get(f"{self.rcm_server_url}/robots/{robot_id}/rcm")
                if response.status_code == 200:
                    rcm = response.json()
                    self.tool_generator.load_rcm(rcm)
                    logger.info(f"Connected to robot {robot_id} with {len(self.tool_generator.tools)} tools")
                    return True
                else:
                    logger.error(f"Failed to get RCM for robot {robot_id}: {response.status_code}")
                    return False
        except Exception as e:
            logger.error(f"Error connecting to robot {robot_id}: {e}")
            return False
    
    def get_system_prompt(self) -> str:
        """Generate system prompt with robot capabilities"""
        if not self.robot_id or not self.tool_generator.robot_rcm:
            return "I am a robot assistant, but no robot capabilities are currently loaded."
        
        rcm = self.tool_generator.robot_rcm
        robot_name = rcm.get("metadata", {}).get("robot_name", self.robot_id)
        
        # Build capability description
        capabilities = []
        
        # Locomotion
        locomotion = rcm.get("locomotion", {})
        if locomotion.get("type") != "static":
            capabilities.append(f"- Mobile base with {locomotion.get('type', 'unknown')} drive")
        
        # Manipulation
        end_effectors = rcm.get("end_effectors", [])
        if end_effectors:
            capabilities.append(f"- {len(end_effectors)} end effector(s)")
        
        # Gripper
        if rcm.get("has_gripper"):
            gripper_type = rcm.get("gripper_type", "unknown")
            capabilities.append(f"- {gripper_type} gripper")
        
        # Sensors
        sensors = rcm.get("sensors", [])
        if sensors:
            sensor_types = list(set(s.get("type") for s in sensors))
            capabilities.append(f"- Sensors: {', '.join(sensor_types)}")
        
        # Joints
        joints = rcm.get("joints", {})
        movable_joints = [j for j, data in joints.items() if data.get("type") in ["revolute", "prismatic", "continuous"]]
        if movable_joints:
            capabilities.append(f"- {len(movable_joints)} controllable joint(s)")
        
        # Safety constraints
        constraints = []
        dynamics = rcm.get("dynamics", {})
        if dynamics.get("mass_kg"):
            constraints.append(f"Robot mass: {dynamics['mass_kg']:.1f} kg")
        
        # Build tool list
        tool_names = list(self.tool_generator.tools.keys())
        
        prompt = f"""I am controlling {robot_name}, a robot with the following capabilities:

{chr(10).join(capabilities)}

SAFETY CONSTRAINTS:
{chr(10).join(constraints) if constraints else "- Standard safety protocols apply"}

AVAILABLE TOOLS:
{', '.join(tool_names)}

IMPORTANT TOOL USAGE RULES:
- For MOVEMENT commands like "move to [object]", "go to [object]", "pick up [object]" → Use move_to_object tool
- For INFORMATION queries like "what is [object]", "show me [object]", "describe [object]" → Use resolve_object_description tool
- The move_to_object tool handles object resolution automatically - no need to call resolve_object_description first
- Always use the most direct tool for the requested action

I can ONLY use the tools listed above. I cannot perform actions that require capabilities this robot doesn't have. For example:
- If the robot has no gripper, I cannot grasp objects
- If the robot is stationary, I cannot move the base
- I must respect all joint limits, velocity limits, and payload constraints

I will always validate that requested actions are within the robot's capabilities before attempting them."""
        
        return prompt
    
    def get_tool_schemas(self) -> List[Dict[str, Any]]:
        """Get tool schemas for LLM"""
        return self.tool_generator.get_tool_schemas()
    
    async def process_message(self, user_message: str) -> str:
        """Process a user message and return response"""
        if not self.robot_id:
            return "Please connect to a robot first using connect_robot(robot_id)."
        
        # Add user message to history
        self.conversation_history.append(LLMMessage(role="user", content=user_message))
        
        if self.use_gpt and self.gpt:
            response = await self._generate_gpt_response(user_message)
        else:
            # Fallback to simple rule-based responses
            response = await self._generate_mock_response(user_message)
        
        # Add assistant response to history
        self.conversation_history.append(LLMMessage(role="assistant", content=response))
        
        return response
    
    async def _generate_gpt_response(self, user_message: str) -> str:
        """Generate response using GPT API"""
        try:
            # Get system prompt and available tools
            system_prompt = self.get_system_prompt()
            available_tools = self.get_tool_schemas()
            
            # Convert conversation history to format expected by GPT
            conversation_context = []
            for msg in self.conversation_history[:-1]:  # Exclude the current message we just added
                if msg.role in ["user", "assistant"]:
                    conversation_context.append({"role": msg.role, "content": msg.content})
            
            # Process with GPT
            response = await self.gpt.process_robot_command(
                system_prompt=system_prompt,
                user_message=user_message,
                available_tools=available_tools,
                conversation_history=conversation_context
            )
            
            # Handle response
            if response and "choices" in response and len(response["choices"]) > 0:
                choice = response["choices"][0]
                message = choice["message"]
                
                # Check if GPT wants to call tools
                if "tool_calls" in message and message["tool_calls"]:
                    return await self._handle_tool_calls(message["tool_calls"], message.get("content", ""))
                else:
                    return message.get("content", "I'm processing your request...")
            else:
                logger.error("Invalid response from GPT API")
                return "I encountered an error processing your request. Please try again."
                
        except Exception as e:
            logger.error(f"Error generating GPT response: {e}")
            return f"I encountered an error: {str(e)}. Falling back to basic responses."
    
    async def _handle_tool_calls(self, tool_calls: List[Dict], assistant_message: str = "") -> str:
        """Handle tool calls from GPT"""
        results = []
        
        for tool_call in tool_calls:
            function_name = tool_call["function"]["name"]
            try:
                # Parse arguments
                arguments = json.loads(tool_call["function"]["arguments"])
                
                # Execute tool
                result = self.tool_generator.execute_tool(function_name, **arguments)
                
                if result.get("status") == "success":
                    results.append(f"✓ {function_name}: {result.get('message', 'Executed successfully')}")
                else:
                    results.append(f"✗ {function_name}: {result.get('message', 'Failed')}")
                    
            except Exception as e:
                logger.error(f"Error executing tool {function_name}: {e}")
                results.append(f"✗ {function_name}: Error - {str(e)}")
        
        # Combine assistant message with tool results
        response_parts = []
        if assistant_message:
            response_parts.append(assistant_message)
        if results:
            response_parts.append("Tool execution results:")
            response_parts.extend(results)
        
        return "\n".join(response_parts) if response_parts else "Commands executed."
    
    async def _generate_mock_response(self, user_message: str) -> str:
        """Generate response (mock LLM for demo)"""
        user_lower = user_message.lower()
        
        # Simple command parsing
        if "move forward" in user_lower or "drive forward" in user_lower:
            return await self._handle_move_command("forward", user_message)
        elif "turn" in user_lower or "rotate" in user_lower:
            return await self._handle_turn_command(user_message)
        elif "stop" in user_lower:
            return await self._handle_stop_command()
        elif "status" in user_lower or "capabilities" in user_lower:
            return self._handle_status_request()
        elif "joints" in user_lower:
            return await self._handle_joint_command(user_message)
        elif "sensor" in user_lower or "scan" in user_lower:
            return await self._handle_sensor_command(user_message)
        else:
            return f"I understand you want me to: {user_message}. Let me check what tools I have available and how to help you safely."
    
    async def _handle_move_command(self, direction: str, full_message: str) -> str:
        """Handle movement commands"""
        if "drive_straight" not in self.tool_generator.tools:
            return "I don't have the capability to drive straight. This robot may not be mobile."
        
        # Extract distance if mentioned
        distance = 1.0  # default
        words = full_message.split()
        for i, word in enumerate(words):
            if word in ["meter", "meters", "m"] and i > 0:
                try:
                    distance = float(words[i-1])
                except ValueError:
                    pass
        
        # Execute the tool
        result = self.tool_generator.execute_tool("drive_straight", distance_m=distance, speed_mps=0.5)
        
        if result.get("status") == "success":
            return f"Moving forward {distance} meters. Command executed successfully."
        else:
            return f"Failed to move forward: {result.get('message', 'Unknown error')}"
    
    async def _handle_turn_command(self, full_message: str) -> str:
        """Handle turn commands"""
        if "rotate_in_place" not in self.tool_generator.tools:
            return "I don't have the capability to rotate. This robot may not be mobile."
        
        # Extract angle if mentioned
        angle = 1.57  # default 90 degrees
        if "left" in full_message.lower():
            angle = 1.57  # 90 degrees left
        elif "right" in full_message.lower():
            angle = -1.57  # 90 degrees right
        
        result = self.tool_generator.execute_tool("rotate_in_place", yaw_rad=angle, yaw_rate_rps=0.5)
        
        if result.get("status") == "success":
            direction = "left" if angle > 0 else "right"
            return f"Turning {direction}. Command executed successfully."
        else:
            return f"Failed to turn: {result.get('message', 'Unknown error')}"
    
    async def _handle_stop_command(self) -> str:
        """Handle stop commands"""
        if "stop" not in self.tool_generator.tools:
            return "I don't have a stop command available."
        
        result = self.tool_generator.execute_tool("stop")
        
        if result.get("status") == "success":
            return "Stopped successfully."
        else:
            return f"Failed to stop: {result.get('message', 'Unknown error')}"
    
    def _handle_status_request(self) -> str:
        """Handle status/capability requests"""
        if not self.tool_generator.robot_rcm:
            return "No robot capabilities loaded."
        
        rcm = self.tool_generator.robot_rcm
        robot_name = rcm.get("metadata", {}).get("robot_name", self.robot_id)
        
        status = [f"Robot: {robot_name}"]
        
        # Locomotion status
        locomotion = rcm.get("locomotion", {})
        status.append(f"Locomotion: {locomotion.get('type', 'unknown')}")
        
        # Tool status
        status.append(f"Available tools: {', '.join(self.tool_generator.tools.keys())}")
        
        # Joint status
        joints = rcm.get("joints", {})
        movable_joints = [j for j, data in joints.items() if data.get("type") in ["revolute", "prismatic", "continuous"]]
        if movable_joints:
            status.append(f"Controllable joints: {len(movable_joints)}")
        
        return "\n".join(status)
    
    async def _handle_joint_command(self, full_message: str) -> str:
        """Handle joint control commands"""
        if "control_joints" not in self.tool_generator.tools:
            return "I don't have joint control capabilities."
        
        # This is a simplified parser - in production you'd want more sophisticated NLP
        return "Joint control is available. Please specify joint positions explicitly, e.g., 'set joint1 to 0.5 radians'."
    
    async def _handle_sensor_command(self, full_message: str) -> str:
        """Handle sensor reading commands"""
        sensor_tools = [name for name in self.tool_generator.tools.keys() if name.startswith("read_")]
        
        if not sensor_tools:
            return "I don't have any sensors available."
        
        # Execute first available sensor tool as demo
        sensor_tool = sensor_tools[0]
        result = self.tool_generator.execute_tool(sensor_tool)
        
        if result.get("status") == "success":
            return f"Sensor reading from {sensor_tool}: {result.get('data', {})}"
        else:
            return f"Failed to read sensor: {result.get('message', 'Unknown error')}"
    
    async def execute_tool_call(self, tool_name: str, **kwargs) -> Dict[str, Any]:
        """Execute a tool call from LLM"""
        return self.tool_generator.execute_tool(tool_name, **kwargs)

# Example usage and testing
async def main():
    bridge = AgentBridge()
    
    # Connect to robot (assumes RCM server is running)
    success = await bridge.connect_robot("turtlebot")
    if not success:
        print("Failed to connect to robot. Make sure RCM server is running.")
        return
    
    print("System Prompt:")
    print(bridge.get_system_prompt())
    print("\n" + "="*50 + "\n")
    
    # Test conversation
    test_messages = [
        "What are your capabilities?",
        "Move forward 2 meters",
        "Turn left",
        "Stop",
        "Read the lidar sensor"
    ]
    
    for msg in test_messages:
        print(f"User: {msg}")
        response = await bridge.process_message(msg)
        print(f"Robot: {response}")
        print()

if __name__ == "__main__":
    asyncio.run(main())
