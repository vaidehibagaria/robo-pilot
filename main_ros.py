#!/usr/bin/env python3
"""
Main runner for the RCM-based robot control framework with ROS/Gazebo support
"""

import asyncio
import json
import logging
import argparse
from pathlib import Path
import threading
import time
import os
import subprocess

from rcm_server import RCMServer
from agent_bridge import AgentBridge

# Import both tool generators
from tool_generator import RCMToolGenerator  # Original fake simulator
from ros_tool_generator import ROSToolGenerator  # New ROS integration

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RCMFramework:
    """Main framework coordinator with ROS/Gazebo support"""
    
    def __init__(self, rcm_path: str = None, robot_id: str = "default", 
                 server_port: int = 8000, api_key: str = None, 
                 use_ros: bool = False, robot_namespace: str = ""):
        self.rcm_path = rcm_path
        self.robot_id = robot_id
        self.server_port = server_port
        self.api_key = api_key
        self.use_ros = use_ros
        self.robot_namespace = robot_namespace
        
        self.server = None
        self.bridge = None
        self.server_thread = None
        
        logger.info(f"RCM Framework initialized with ROS={'enabled' if use_ros else 'disabled'}")
    
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
        """Start agent bridge with appropriate tool generator"""
        # Create custom agent bridge with ROS or fake tool generator
        self.bridge = ROSAgentBridge(
            f"http://localhost:{self.server_port}", 
            self.api_key, 
            use_ros=self.use_ros,
            robot_namespace=self.robot_namespace
        )
        
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
        if self.use_ros:
            print("🤖 ROS/Gazebo Integration ENABLED")
        else:
            print("🎮 Using Fake Simulator")
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
                elif user_input.lower() == 'ros_status':
                    self.show_ros_status()
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
        help_text = f"""
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
- ros_status: Show ROS connection status (if using ROS)
- quit/exit/q: Exit the program

Current mode: {'ROS/Gazebo' if self.use_ros else 'Fake Simulator'}
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
        
        if hasattr(self.bridge.tool_generator, 'state_tracker') and self.bridge.tool_generator.state_tracker:
            status = self.bridge.tool_generator.state_tracker.get_status()
            pose = status["current_pose"]
            print(f"\n🤖 Robot Position:")
            print(f"  Location: ({pose['x']:.2f}, {pose['y']:.2f}) meters")
            print(f"  Heading: {pose['theta_deg']:.1f}° ({pose['theta']:.3f} rad)")
            print(f"  Total distance traveled: {status['total_distance_traveled']:.2f}m")
            print(f"  Total rotation: {status.get('total_rotation_deg', 0):.1f}°")
        else:
            print("Position tracking not available")
        print()
    
    def show_path(self):
        """Show robot's movement path"""
        if not self.bridge or not self.bridge.tool_generator:
            print("No robot state available")
            return
        
        if hasattr(self.bridge.tool_generator, 'state_tracker') and self.bridge.tool_generator.state_tracker:
            path_viz = self.bridge.tool_generator.state_tracker.get_path_visualization()
            print(f"\n🗺️  {path_viz}")
        else:
            print("Path tracking not available")
        print()
    
    def reset_position(self):
        """Reset robot position to origin"""
        if not self.bridge or not self.bridge.tool_generator:
            print("No robot state available")
            return
        
        if hasattr(self.bridge.tool_generator, 'state_tracker') and self.bridge.tool_generator.state_tracker:
            result = self.bridge.tool_generator.state_tracker.reset_position()
            print(f"\n🔄 Robot position reset to origin (0, 0)")
        else:
            print("Position reset not available")
        print()
    
    def show_ros_status(self):
        """Show ROS connection status"""
        if not self.use_ros:
            print("ROS integration is disabled. Use --ros to enable.")
            return
        
        if not self.bridge or not hasattr(self.bridge.tool_generator, '_ros_initialized'):
            print("ROS tool generator not available")
            return
        
        ros_initialized = self.bridge.tool_generator._ros_initialized
        print(f"\n🤖 ROS Status:")
        print(f"  ROS Bridge: {'Connected' if ros_initialized else 'Disconnected'}")
        print(f"  Robot Namespace: '{self.robot_namespace}' (empty = default)")
        
        if ros_initialized and self.bridge.tool_generator.ros_bridge:
            pose = self.bridge.tool_generator.ros_bridge.current_pose.to_dict()
            print(f"  Current Pose: ({pose['x']:.2f}, {pose['y']:.2f}, {pose['theta_deg']:.1f}°)")
        
        print()


class ROSAgentBridge(AgentBridge):
    """Extended AgentBridge that can use ROS or fake tool generator"""
    
    def __init__(self, rcm_server_url: str = "http://localhost:8000", 
                 api_key: str = None, use_ros: bool = False, 
                 robot_namespace: str = ""):
        # Initialize parent class
        super().__init__(rcm_server_url, api_key)
        
        # Replace tool generator with ROS or fake version
        if use_ros:
            self.tool_generator = ROSToolGenerator(robot_namespace)
            logger.info("Using ROS Tool Generator")
        else:
            self.tool_generator = RCMToolGenerator()
            logger.info("Using Fake Simulator Tool Generator")


def check_ros_environment():
    """Check if ROS environment is properly set up"""
    # First try to detect ROS_DISTRO
    ros_distro = os.environ.get('ROS_DISTRO')
    
    # If not set, try to source ROS and check
    if not ros_distro:
        try:
            # Try to source ROS and get the environment
            result = subprocess.run(
                ['bash', '-c', 'source /opt/ros/humble/setup.bash && echo $ROS_DISTRO'],
                capture_output=True, text=True, timeout=5
            )
            if result.returncode == 0 and result.stdout.strip():
                ros_distro = result.stdout.strip()
                os.environ['ROS_DISTRO'] = ros_distro  # Set it for this process
                logger.info(f"ROS {ros_distro} environment detected via sourcing")
            else:
                logger.warning("Could not detect ROS environment")
                return False
        except Exception as e:
            logger.warning(f"Error checking ROS environment: {e}")
            return False
    
    # Test if ros2 command is available
    try:
        # Test with sourced environment
        result = subprocess.run(
            ['bash', '-c', 'source /opt/ros/humble/setup.bash && ros2 --help'],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0:
            logger.info(f"ROS {ros_distro} environment detected and working")
            return True
        else:
            logger.warning(f"ROS {ros_distro} detected but ros2 command failed")
            return False
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        logger.warning(f"ros2 command not found or timed out: {e}")
        return False


async def run_demo_conversation(use_ros: bool = False):
    """Run a demo conversation"""
    if use_ros and not check_ros_environment():
        logger.error("ROS environment not properly configured")
        return
    
    framework = RCMFramework(
        rcm_path="turtlebot_rcm.json", 
        robot_id="turtlebot",
        use_ros=use_ros
    )
    
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
        time.sleep(2)


def main():
    parser = argparse.ArgumentParser(description="RCM Robot Control Framework with ROS/Gazebo Support")
    parser.add_argument("--json", help="Path to generated RCM JSON file", required=True)
    parser.add_argument("--port", type=int, default=8000, help="Server port")
    parser.add_argument("--demo", action="store_true", help="Run demo conversation")
    parser.add_argument("--server-only", action="store_true", help="Run server only")
    parser.add_argument("--api-key", help="OpenAI API key (can also be set via OPENAI_API_KEY env var)")
    parser.add_argument("--ros", action="store_true", help="Enable ROS/Gazebo integration")
    parser.add_argument("--namespace", default="", help="Robot namespace for ROS topics")
    parser.add_argument("--check-ros", action="store_true", help="Check ROS environment and exit")
    
    args = parser.parse_args()
    
    if args.check_ros:
        if check_ros_environment():
            print("✅ ROS environment is properly configured")
        else:
            print("❌ ROS environment is not configured")
            print("Please source your ROS setup: source /opt/ros/humble/setup.bash")
        return
    
    if args.ros and not check_ros_environment():
        logger.error("ROS integration requested but environment not configured")
        print("Please source your ROS setup: source /opt/ros/humble/setup.bash")
        print("Or run without --ros flag to use fake simulator")
        return
    
    if args.demo:
        asyncio.run(run_demo_conversation(use_ros=args.ros))
        return
    
    framework = RCMFramework(
        rcm_path=args.json,
        robot_id="robot",  # Generic robot ID
        server_port=args.port,
        api_key=args.api_key,
        use_ros=args.ros,
        robot_namespace=args.namespace
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
    finally:
        # Cleanup ROS if used
        if args.ros and framework.bridge and hasattr(framework.bridge.tool_generator, 'shutdown_ros'):
            framework.bridge.tool_generator.shutdown_ros()


if __name__ == "__main__":
    main()
