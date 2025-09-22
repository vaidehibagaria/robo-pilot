#!/usr/bin/env python3
"""
Dynamic Tool Generator - Creates LLM tools based on RCM capabilities
"""

import json
import logging
from typing import Dict, Any, List, Callable, Optional
from dataclasses import dataclass
import inspect

from robot_state_tracker import RobotStateTracker, RobotPose

logger = logging.getLogger(__name__)

@dataclass
class ToolDefinition:
    """Represents a dynamically generated tool"""
    name: str
    description: str
    parameters: Dict[str, Any]
    function: Callable
    constraints: Dict[str, Any]
    primitive_id: str

class RCMToolGenerator:
    """Generates LLM-compatible tools from RCM primitives"""
    
    def __init__(self):
        self.tools: Dict[str, ToolDefinition] = {}
        self.robot_rcm: Optional[Dict[str, Any]] = None
        self.state_tracker = RobotStateTracker()  # Initialize state tracker
    
    def load_rcm(self, rcm: Dict[str, Any]):
        """Load RCM and generate tools"""
        self.robot_rcm = rcm
        self.tools.clear()
        self._generate_tools_from_rcm()
        logger.info(f"Generated {len(self.tools)} tools from RCM")
    
    def _generate_tools_from_rcm(self):
        """Generate tools based on RCM primitives and capabilities"""
        if not self.robot_rcm:
            return
        
        # Generate tools from primitives
        primitives = self.robot_rcm.get("primitives", [])
        for primitive in primitives:
            tool = self._create_tool_from_primitive(primitive)
            if tool:
                self.tools[tool.name] = tool
        
        # Generate joint control tools if robot has joints
        joints = self.robot_rcm.get("joints", {})
        if joints:
            joint_tool = self._create_joint_control_tool(joints)
            if joint_tool:
                self.tools[joint_tool.name] = joint_tool
        
        # Generate sensor reading tools
        sensors = self.robot_rcm.get("sensors", [])
        for sensor in sensors:
            sensor_tool = self._create_sensor_tool(sensor)
            if sensor_tool:
                self.tools[sensor_tool.name] = sensor_tool
    
    def _create_tool_from_primitive(self, primitive: Dict[str, Any]) -> Optional[ToolDefinition]:
        """Create a tool from an RCM primitive"""
        primitive_id = primitive.get("primitive_id")
        signature = primitive.get("signature", "")
        
        if not primitive_id or not signature:
            return None
        
        # Parse signature to extract parameters
        params = self._parse_signature(signature)
        
        # Create constraints from preconditions
        constraints = {
            "preconditions": primitive.get("preconditions", []),
            "confidence": primitive.get("confidence", 0.5)
        }
        
        # Add specific constraints based on primitive type
        if primitive_id == "move_base":
            constraints.update(self._get_locomotion_constraints())
        elif primitive_id in ["drive_straight", "rotate_in_place", "set_base_twist"]:
            constraints.update(self._get_base_motion_constraints())
        elif primitive_id == "move_arm_to_pose":
            constraints.update(self._get_arm_constraints())
        elif primitive_id == "grasp_object":
            constraints.update(self._get_gripper_constraints())
        
        # Create the actual function
        def tool_function(**kwargs):
            return self._execute_primitive(primitive_id, kwargs, constraints)
        
        # Sanitize primitive name for OpenAI API
        sanitized_name = self._sanitize_tool_name(primitive_id)
        
        return ToolDefinition(
            name=sanitized_name,
            description=f"Execute {primitive_id}: {signature}",
            parameters=params,
            function=tool_function,
            constraints=constraints,
            primitive_id=primitive_id
        )
    
    def _create_joint_control_tool(self, joints: Dict[str, Any]) -> Optional[ToolDefinition]:
        """Create joint control tool with limits"""
        movable_joints = {}
        for joint_name, joint_data in joints.items():
            if joint_data.get("type") in ["revolute", "prismatic", "continuous"]:
                limits = joint_data.get("limits", {})
                movable_joints[joint_name] = {
                    "type": joint_data.get("type"),
                    "lower": limits.get("lower"),
                    "upper": limits.get("upper"),
                    "velocity": limits.get("velocity"),
                    "effort": limits.get("effort")
                }
        
        if not movable_joints:
            return None
        
        # Create parameters for joint positions
        params = {
            "type": "object",
            "properties": {},
            "required": []
        }
        
        for joint_name, joint_info in movable_joints.items():
            param_def = {"type": "number", "description": f"Target position for {joint_name}"}
            if joint_info["lower"] is not None and joint_info["upper"] is not None:
                param_def["minimum"] = joint_info["lower"]
                param_def["maximum"] = joint_info["upper"]
            params["properties"][joint_name] = param_def
        
        def joint_control_function(**kwargs):
            return self._execute_joint_control(kwargs, movable_joints)
        
        return ToolDefinition(
            name="control_joints",
            description="Control robot joints within safe limits",
            parameters=params,
            function=joint_control_function,
            constraints={"joint_limits": movable_joints},
            primitive_id="control_joints"
        )
    
    def _create_sensor_tool(self, sensor: Dict[str, Any]) -> Optional[ToolDefinition]:
        """Create sensor reading tool"""
        sensor_type = sensor.get("type")
        link = sensor.get("link")
        
        if not sensor_type or not link:
            return None
        
        # Sanitize tool name - remove invalid characters for OpenAI API
        sanitized_link = self._sanitize_tool_name(link)
        sanitized_sensor_type = self._sanitize_tool_name(sensor_type)
        
        def sensor_function():
            return self._read_sensor(sensor)
        
        return ToolDefinition(
            name=f"read_{sanitized_sensor_type}_{sanitized_link}",
            description=f"Read {sensor_type} sensor data from {link}",
            parameters={"type": "object", "properties": {}, "required": []},
            function=sensor_function,
            constraints={"sensor_type": sensor_type},
            primitive_id=f"read_sensor_{sanitized_link}"
        )
    
    def _sanitize_tool_name(self, name: str) -> str:
        """Sanitize tool name to match OpenAI API requirements: ^[a-zA-Z0-9_-]+$"""
        import re
        # Replace invalid characters with underscores
        sanitized = re.sub(r'[^a-zA-Z0-9_-]', '_', name)
        # Remove consecutive underscores
        sanitized = re.sub(r'_+', '_', sanitized)
        # Remove leading/trailing underscores
        sanitized = sanitized.strip('_')
        # Ensure it's not empty
        if not sanitized:
            sanitized = "sensor"
        return sanitized
    
    def _parse_signature(self, signature: str) -> Dict[str, Any]:
        """Parse function signature to extract parameters"""
        # Simple parser for signatures like "move_base(goal:Pose)->Status"
        if "(" not in signature:
            return {"type": "object", "properties": {}, "required": []}
        
        params_str = signature.split("(")[1].split(")")[0]
        if not params_str.strip():
            return {"type": "object", "properties": {}, "required": []}
        
        properties = {}
        required = []
        
        for param in params_str.split(","):
            param = param.strip()
            if ":" in param:
                name, type_hint = param.split(":", 1)
                name = name.strip()
                type_hint = type_hint.strip()
                
                # Map types
                if type_hint in ["float", "double"]:
                    param_type = "number"
                elif type_hint in ["int", "integer"]:
                    param_type = "integer"
                elif type_hint in ["str", "string"]:
                    param_type = "string"
                elif type_hint in ["bool", "boolean"]:
                    param_type = "boolean"
                else:
                    param_type = "object"  # Complex types like Pose
                
                properties[name] = {"type": param_type, "description": f"Parameter {name} of type {type_hint}"}
                required.append(name)
        
        return {
            "type": "object",
            "properties": properties,
            "required": required
        }
    
    def _get_locomotion_constraints(self) -> Dict[str, Any]:
        """Get locomotion-specific constraints"""
        constraints = {}
        locomotion = self.robot_rcm.get("locomotion", {})
        constraints["locomotion_type"] = locomotion.get("type", "unknown")
        
        # Add footprint constraints if available
        footprint = self.robot_rcm.get("footprint")
        if footprint:
            constraints["footprint_radius_m"] = footprint.get("radius_m", 0.5)
        
        return constraints
    
    def _get_base_motion_constraints(self) -> Dict[str, Any]:
        """Get base motion constraints"""
        constraints = {"max_linear_velocity": 1.0, "max_angular_velocity": 1.0}  # Default safe values
        
        # Look for velocity limits in wheel joints
        joints = self.robot_rcm.get("joints", {})
        for joint_name, joint_data in joints.items():
            if "wheel" in joint_name.lower():
                limits = joint_data.get("limits", {})
                if limits.get("velocity"):
                    # Rough conversion from wheel velocity to base velocity
                    constraints["max_linear_velocity"] = min(constraints["max_linear_velocity"], limits["velocity"] * 0.1)
        
        return constraints
    
    def _get_arm_constraints(self) -> Dict[str, Any]:
        """Get arm manipulation constraints"""
        constraints = {}
        workspaces = self.robot_rcm.get("workspaces", {})
        if workspaces:
            constraints["workspaces"] = workspaces
        
        # Add joint limits for arm joints
        joints = self.robot_rcm.get("joints", {})
        arm_joints = {}
        for joint_name, joint_data in joints.items():
            if not any(kw in joint_name.lower() for kw in ["wheel", "caster", "base"]):
                limits = joint_data.get("limits", {})
                if any(v is not None for v in limits.values()):
                    arm_joints[joint_name] = limits
        
        if arm_joints:
            constraints["arm_joint_limits"] = arm_joints
        
        return constraints
    
    def _get_gripper_constraints(self) -> Dict[str, Any]:
        """Get gripper constraints"""
        constraints = {
            "has_gripper": self.robot_rcm.get("has_gripper", False),
            "gripper_type": self.robot_rcm.get("gripper_type", "none")
        }
        
        # Add payload capacity if available
        dynamics = self.robot_rcm.get("dynamics", {})
        if dynamics.get("mass_kg"):
            constraints["max_payload_kg"] = dynamics["mass_kg"] * 0.1  # 10% of robot mass
        
        return constraints
    
    def _execute_primitive(self, primitive_id: str, params: Dict[str, Any], constraints: Dict[str, Any]) -> Dict[str, Any]:
        """Execute a primitive with safety checks and state tracking"""
        logger.info(f"Executing primitive {primitive_id} with params: {params}")
        
        # Validate against constraints
        validation_result = self._validate_parameters(params, constraints)
        if not validation_result["valid"]:
            return {"status": "error", "message": validation_result["message"]}
        
        # Execute with state tracking
        if primitive_id == "drive_straight":
            distance_m = params.get("distance_m", 0.0)
            speed_mps = params.get("speed_mps", 0.5)
            result = self.state_tracker.drive_straight(distance_m, speed_mps)
            result["message"] = f"Drove {distance_m}m to position ({result['new_pose']['x']:.2f}, {result['new_pose']['y']:.2f})"
            return result
            
        elif primitive_id == "rotate_in_place":
            yaw_rad = params.get("yaw_rad", 0.0)
            yaw_rate_rps = params.get("yaw_rate_rps", 0.5)
            result = self.state_tracker.rotate_in_place(yaw_rad, yaw_rate_rps)
            result["message"] = f"Rotated {result['rotation_deg']:.1f}° to heading {result['new_pose']['theta_deg']:.1f}°"
            return result
            
        elif primitive_id == "set_base_twist":
            vx = params.get("vx", 0.0)
            vy = params.get("vy", 0.0)
            wz = params.get("wz", 0.0)
            duration = params.get("duration", 1.0)
            result = self.state_tracker.set_base_twist(vx, vy, wz, duration)
            result["message"] = f"Applied velocity for {duration}s, moved to ({result['new_pose']['x']:.2f}, {result['new_pose']['y']:.2f})"
            return result
            
        elif primitive_id == "stop":
            result = self.state_tracker.stop()
            return result
            
        else:
            # For other primitives, return basic success without state change
            return {
                "status": "success",
                "primitive_id": primitive_id,
                "parameters": params,
                "message": f"Executed {primitive_id} successfully",
                "current_pose": self.state_tracker.current_pose.to_dict()
            }
    
    def _execute_joint_control(self, joint_positions: Dict[str, float], joint_limits: Dict[str, Any]) -> Dict[str, Any]:
        """Execute joint control with limit checking"""
        logger.info(f"Controlling joints: {joint_positions}")
        
        # Validate joint limits
        for joint_name, position in joint_positions.items():
            if joint_name in joint_limits:
                limits = joint_limits[joint_name]
                if limits["lower"] is not None and position < limits["lower"]:
                    return {"status": "error", "message": f"Joint {joint_name} position {position} below lower limit {limits['lower']}"}
                if limits["upper"] is not None and position > limits["upper"]:
                    return {"status": "error", "message": f"Joint {joint_name} position {position} above upper limit {limits['upper']}"}
        
        return {
            "status": "success",
            "joint_positions": joint_positions,
            "message": "Joint control executed successfully"
        }
    
    def _read_sensor(self, sensor: Dict[str, Any]) -> Dict[str, Any]:
        """Read sensor data"""
        logger.info(f"Reading sensor: {sensor}")
        
        # Mock sensor data based on type
        sensor_type = sensor.get("type")
        if sensor_type == "lidar":
            return {"status": "success", "data": {"ranges": [1.0] * 360, "angle_min": 0, "angle_max": 6.28}}
        elif sensor_type == "camera_rgbd":
            return {"status": "success", "data": {"width": 640, "height": 480, "has_depth": True}}
        elif sensor_type == "imu":
            return {"status": "success", "data": {"orientation": [0, 0, 0, 1], "angular_velocity": [0, 0, 0]}}
        else:
            return {"status": "success", "data": {"raw": "sensor_data"}}
    
    def _validate_parameters(self, params: Dict[str, Any], constraints: Dict[str, Any]) -> Dict[str, Any]:
        """Validate parameters against constraints"""
        # Basic validation - can be extended
        preconditions = constraints.get("preconditions", [])
        
        # Check battery level constraint
        if "battery_level>0.2" in preconditions:
            # In real implementation, check actual battery level
            pass
        
        return {"valid": True, "message": "Parameters valid"}
    
    def get_tool_schemas(self) -> List[Dict[str, Any]]:
        """Get OpenAI-compatible tool schemas"""
        schemas = []
        for tool in self.tools.values():
            schema = {
                "type": "function",
                "function": {
                    "name": tool.name,
                    "description": tool.description,
                    "parameters": tool.parameters
                }
            }
            schemas.append(schema)
        return schemas
    
    def execute_tool(self, tool_name: str, **kwargs) -> Dict[str, Any]:
        """Execute a tool by name"""
        if tool_name not in self.tools:
            return {"status": "error", "message": f"Tool {tool_name} not found"}
        
        tool = self.tools[tool_name]
        try:
            result = tool.function(**kwargs)
            return result
        except Exception as e:
            logger.error(f"Error executing tool {tool_name}: {e}")
            return {"status": "error", "message": str(e)}
