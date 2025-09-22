#!/usr/bin/env python3
"""
Main runner for the RCM-based robot control framework
"""

import asyncio
import json
import logging
import argparse
from pathlib import Path
import threading
import time

from rcm_server import RCMServer
from agent_bridge import AgentBridge

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RCMFramework:
    """Main framework coordinator"""
    
    def __init__(self, rcm_path: str = None, robot_id: str = "default", server_port: int = 8000, api_key: str = None):
        self.rcm_path = rcm_path
        self.robot_id = robot_id
        self.server_port = server_port
        self.api_key = api_key
        self.server = None
        self.bridge = None
        self.server_thread = None
    
    def start_server(self):
        """Start RCM server in background thread"""
        def run_server():
            self.server = RCMServer()
            
            # Load RCM if provided
            if self.rcm_path and Path(self.rcm_path).exists():
                self.server.load_rcm_from_file(self.robot_id, self.rcm_path)
                logger.info(f"Loaded RCM for {self.robot_id} from {self.rcm_path}")
            
            # Run server
            self.server.run(host="0.0.0.0", port=self.server_port)
        
        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.start()
        
        # Wait for server to start
        time.sleep(2)
        logger.info(f"RCM server started on port {self.server_port}")
    
    async def start_bridge(self):
        """Start agent bridge"""
        self.bridge = AgentBridge(f"http://localhost:{self.server_port}", self.api_key)
        
        # Connect to robot
        success = await self.bridge.connect_robot(self.robot_id)
        if success:
            logger.info(f"Agent bridge connected to robot {self.robot_id}")
            return True
        else:
            logger.error(f"Failed to connect agent bridge to robot {self.robot_id}")
            return False
    
    async def run_interactive_session(self):
        """Run interactive chat session"""
        if not self.bridge:
            logger.error("Bridge not initialized")
            return
        
        print("\n" + "="*60)
        print("RCM Robot Control Framework")
        print("="*60)
        print(f"Connected to robot: {self.robot_id}")
        print("\nSystem capabilities:")
        print(self.bridge.get_system_prompt())
        print("\n" + "="*60)
        print("Enter commands (type 'quit' to exit, 'help' for help):")
        print()
        
        while True:
            try:
                user_input = input("You: ").strip()
                
                if user_input.lower() in ['quit', 'exit', 'q']:
                    break
                elif user_input.lower() == 'help':
                    self.show_help()
                    continue
                elif user_input.lower() == 'tools':
                    self.show_tools()
                    continue
                elif user_input.lower() == 'status':
                    response = await self.bridge.process_message("What is your status?")
                    print(f"Robot: {response}")
                    continue
                elif user_input.lower() == 'position':
                    self.show_position()
                    continue
                elif user_input.lower() == 'path':
                    self.show_path()
                    continue
                elif user_input.lower() == 'reset':
                    self.reset_position()
                    continue
                elif not user_input:
                    continue
                
                # Process message
                response = await self.bridge.process_message(user_input)
                print(f"Robot: {response}")
                print()
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                logger.error(f"Error processing message: {e}")
                print(f"Error: {e}")
        
        print("\nGoodbye!")
    
    def show_help(self):
        """Show help message"""
        help_text = """
Available commands:
- Move forward [distance] meters
- Turn left/right
- Stop
- Read sensor
- What are your capabilities?
- Status

Special commands:
- help: Show this help
- tools: Show available tools
- status: Show robot status
- position: Show current robot position
- path: Show robot's movement path
- reset: Reset robot to origin (0,0)
- quit/exit/q: Exit the program
"""
        print(help_text)
    
    def show_tools(self):
        """Show available tools"""
        if not self.bridge or not self.bridge.tool_generator:
            print("No tools available")
            return
        
        tools = self.bridge.tool_generator.tools
        print(f"\nAvailable tools ({len(tools)}):")
        for name, tool in tools.items():
            print(f"- {name}: {tool.description}")
        print()
    
    def show_position(self):
        """Show current robot position"""
        if not self.bridge or not self.bridge.tool_generator:
            print("No robot state available")
            return
        
        status = self.bridge.tool_generator.state_tracker.get_status()
        pose = status["current_pose"]
        print(f"\n🤖 Robot Position:")
        print(f"  Location: ({pose['x']:.2f}, {pose['y']:.2f}) meters")
        print(f"  Heading: {pose['theta_deg']:.1f}° ({pose['theta']:.3f} rad)")
        print(f"  Total distance traveled: {status['total_distance_traveled']:.2f}m")
        print(f"  Total rotation: {status['total_rotation_deg']:.1f}°")
        print()
    
    def show_path(self):
        """Show robot's movement path"""
        if not self.bridge or not self.bridge.tool_generator:
            print("No robot state available")
            return
        
        path_viz = self.bridge.tool_generator.state_tracker.get_path_visualization()
        print(f"\n🗺️  {path_viz}")
        print()
    
    def reset_position(self):
        """Reset robot position to origin"""
        if not self.bridge or not self.bridge.tool_generator:
            print("No robot state available")
            return
        
        result = self.bridge.tool_generator.state_tracker.reset_position()
        print(f"\n🔄 Robot position reset to origin (0, 0)")
        print()

async def run_demo_conversation():
    """Run a demo conversation"""
    framework = RCMFramework(rcm_path="turtlebot_rcm.json", robot_id="turtlebot")
    
    # Start server
    framework.start_server()
    
    # Start bridge
    success = await framework.start_bridge()
    if not success:
        print("Failed to start framework")
        return
    
    # Demo conversation
    demo_messages = [
        "What are your capabilities?",
        "Move forward 1 meter",
        "Turn left 90 degrees", 
        "Stop",
        "What sensors do you have?"
    ]
    
    print("Running demo conversation...")
    print("="*50)
    
    for msg in demo_messages:
        print(f"\nUser: {msg}")
        response = await framework.bridge.process_message(msg)
        print(f"Robot: {response}")
        time.sleep(1)

def main():
    parser = argparse.ArgumentParser(description="RCM Robot Control Framework")
    parser.add_argument("--json", help="Path to generated RCM JSON file", required=True)
    parser.add_argument("--port", type=int, default=8000, help="Server port")
    parser.add_argument("--demo", action="store_true", help="Run demo conversation")
    parser.add_argument("--server-only", action="store_true", help="Run server only")
    parser.add_argument("--api-key", help="OpenAI API key (can also be set via OPENAI_API_KEY env var)")
    
    args = parser.parse_args()
    
    if args.demo:
        asyncio.run(run_demo_conversation())
        return
    
    framework = RCMFramework(
        rcm_path=args.json,
        robot_id="robot",  # Generic robot ID
        server_port=args.port,
        api_key=args.api_key
    )
    
    # Start server
    framework.start_server()
    
    if args.server_only:
        print(f"RCM server running on port {args.port}")
        print("Press Ctrl+C to stop")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nShutting down...")
        return
    
    # Start interactive session
    async def run_interactive():
        success = await framework.start_bridge()
        if success:
            await framework.run_interactive_session()
        else:
            print("Failed to start framework")
    
    try:
        asyncio.run(run_interactive())
    except KeyboardInterrupt:
        print("\nShutting down...")

if __name__ == "__main__":
    main()
