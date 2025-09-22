#!/usr/bin/env python3
"""
ROS Tool Generator - Creates LLM tools based on RCM capabilities using ROS
"""

import json
import logging
from typing import Dict, Any, List, Callable, Optional
from dataclasses import dataclass
import inspect
import threading
import time

from ros_bridge import ROSBridgeManager, ROSBridge
from llm_object_resolver import LLMObjectResolver

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

class ROSToolGenerator:
    """Generates LLM-compatible tools from RCM primitives using ROS"""
    
    def __init__(self, robot_namespace: str = ""):
        self.tools: Dict[str, ToolDefinition] = {}
        self.robot_rcm: Optional[Dict[str, Any]] = None
        self.robot_namespace = robot_namespace
        
        # Initialize ROS bridge manager
        self.ros_manager = ROSBridgeManager(robot_namespace)
        self.ros_bridge: Optional[ROSBridge] = None
        
        # Initialize LLM object resolver
        self.llm_resolver = LLMObjectResolver()
        self._ros_initialized = False
        
    def initialize_ros(self) -> bool:
        """Initialize ROS connection"""
        if self._ros_initialized:
            return True
            
        try:
            logger.info("Initializing ROS bridge...")
            self.ros_manager.start()
            self.ros_bridge = self.ros_manager.get_bridge()
            
            if self.ros_bridge is None:
                logger.error("Failed to get ROS bridge instance")
                return False
            
            self._ros_initialized = True
            logger.info("ROS bridge initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize ROS bridge: {e}")
            return False
    
    def shutdown_ros(self):
        """Shutdown ROS connection"""
        if self.ros_manager:
            self.ros_manager.stop()
        self._ros_initialized = False
        logger.info("ROS bridge shutdown")
    
    def load_rcm(self, rcm: Dict[str, Any]):
        """Load RCM and generate tools"""
        self.robot_rcm = rcm
        self.tools.clear()
        
        # Initialize ROS if not already done
        if not self._ros_initialized:
            if not self.initialize_ros():
                logger.error("Failed to initialize ROS, falling back to mock tools")
                # Could fall back to original tool generator here
                return
        
        self._generate_tools_from_rcm()
        # Configure controllers/servo based on ros_interfaces if available
        try:
            ros_if = (rcm or {}).get("ros_interfaces", {})
            controllers = ros_if.get("controllers")
            servo_topics = ros_if.get("servo")
            if self.ros_bridge and (controllers or servo_topics):
                self.ros_bridge.configure_controllers(controllers or [], servo_topics or {})
        except Exception as e:
            logger.warning(f"Failed to configure controllers from RCM: {e}")
        logger.info(f"Generated {len(self.tools)} ROS-enabled tools from RCM")
    
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
            # Also provide relative joint offsets tool
            offset_tool = self._create_joint_offset_tool(joints)
            if offset_tool:
                self.tools[offset_tool.name] = offset_tool
        
        # Generate sensor reading tools
        sensors = self.robot_rcm.get("sensors", [])
        for sensor in sensors:
            sensor_tool = self._create_sensor_tool(sensor)
            if sensor_tool:
                self.tools[sensor_tool.name] = sensor_tool
        
        # Add additional manipulator/utility tools conditionally (no servo tools for joint moves)
        self._maybe_add_sample_workspace_tool()
        self._maybe_add_moveit_tools()
        self._maybe_add_relative_movement_tools()
        self._maybe_add_env_tools()
        self._maybe_add_env_object_tools()
        self._maybe_add_move_to_object_tool()

    # ======== Helpers to derive MoveIt defaults from current RCM (robot-agnostic) ========
    def _rcm_srdf_groups(self) -> List[Dict[str, Any]]:
        try:
            return (self.robot_rcm or {}).get('kinematics', {}).get('srdf', {}).get('groups', []) or []
        except Exception:
            return []

    def _rcm_moveit_groups(self) -> List[str]:
        try:
            return (self.robot_rcm or {}).get('ros_interfaces', {}).get('moveit', {}).get('groups', []) or []
        except Exception:
            return []

    def _rcm_end_effectors(self) -> List[str]:
        try:
            return list((self.robot_rcm or {}).get('end_effectors', []) or [])
        except Exception:
            return []

    def _rcm_root_link(self) -> Optional[str]:
        try:
            return (self.robot_rcm or {}).get('kinematics', {}).get('root_link')
        except Exception:
            return None

    def _guess_moveit_group(self) -> Optional[str]:
        # Prefer groups enumerated by MoveIt interfaces; else SRDF groups; prefer names containing 'arm'/'manipulator'
        moveit_groups = self._rcm_moveit_groups()
        if moveit_groups:
            # choose the most arm-like group name if available
            arm_like = [g for g in moveit_groups if any(k in g.lower() for k in ['arm', 'manipulator'])]
            return (arm_like[0] if arm_like else moveit_groups[0])
        srdf_groups = self._rcm_srdf_groups()
        if srdf_groups:
            arm_like = [g.get('name') for g in srdf_groups if isinstance(g, dict) and any(k in (g.get('name','').lower()) for k in ['arm','manipulator'])]
            if arm_like:
                return arm_like[0]
            name0 = srdf_groups[0].get('name') if isinstance(srdf_groups[0], dict) else None
            if name0:
                return name0
        return None

    def _guess_ee_link(self, group_name: Optional[str]) -> Optional[str]:
        # Try SRDF chains tip_link for the selected group
        if group_name:
            for g in self._rcm_srdf_groups():
                if isinstance(g, dict) and g.get('name') == group_name:
                    chains = g.get('chains') or []
                    if chains and isinstance(chains[0], dict):
                        tip = chains[0].get('tip_link')
                        if tip:
                            return tip
                    # If explicit links are listed, try picking a likely EE
                    links = g.get('links') or []
                    if links:
                        hand_like = [l for l in links if any(k in l.lower() for k in ['tool', 'wrist', 'hand', 'finger', 'ee'])]
                        return (hand_like[-1] if hand_like else links[-1])
        # Fall back to RCM end_effectors list
        ees = self._rcm_end_effectors()
        if ees:
            return ees[0]
        return None

    def _guess_frame_id(self) -> str:
        return self._rcm_root_link() or 'base_link'

    def _resolve_moveit_defaults(self, overrides: Dict[str, Any] = None) -> Dict[str, str]:
        overrides = overrides or {}
        group = overrides.get('group_name') or self._guess_moveit_group()
        ee = overrides.get('ee_link') or self._guess_ee_link(group)
        frame = overrides.get('frame_id') or self._guess_frame_id()
        # Final fallbacks remain generic but should rarely be used
        group = group or 'manipulator'
        ee = ee or 'tool0'
        frame = frame or 'base_link'
        return {'group_name': group, 'ee_link': ee, 'frame_id': frame}
    
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

    def _create_joint_offset_tool(self, joints: Dict[str, Any]) -> Optional[ToolDefinition]:
        """Create relative joint offsets tool (adds deltas to current joint positions)."""
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

        params = {
            "type": "object",
            "properties": {},
            "required": []
        }
        for joint_name in movable_joints.keys():
            params["properties"][joint_name] = {"type": "number", "description": f"Delta for {joint_name} (radians/meters)"}

        def joint_offset_function(**kwargs):
            return self._execute_joint_offsets(kwargs, movable_joints)

        return ToolDefinition(
            name="offset_joints",
            description="Offset joints by specified deltas from current positions (clamped to limits)",
            parameters=params,
            function=joint_offset_function,
            constraints={"joint_limits": movable_joints},
            primitive_id="offset_joints"
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
        """Execute a primitive with safety checks using ROS"""
        logger.info(f"Executing ROS primitive {primitive_id} with params: {params}")
        
        # Check if ROS bridge is available
        if not self._ros_initialized or not self.ros_bridge:
            return {"status": "error", "message": "ROS bridge not initialized"}
        
        # Validate against constraints
        validation_result = self._validate_parameters(params, constraints)
        if not validation_result["valid"]:
            return {"status": "error", "message": validation_result["message"]}
        
        # Execute with ROS bridge
        try:
            if primitive_id == "drive_straight":
                distance_m = float(params.get("distance_m", 0.0))
                speed_mps = float(params.get("speed_mps", 0.5))
                return self.ros_bridge.drive_straight(distance_m, speed_mps)
                
            elif primitive_id == "rotate_in_place":
                yaw_rad = float(params.get("yaw_rad", 0.0))
                yaw_rate_rps = float(params.get("yaw_rate_rps", 0.5))
                return self.ros_bridge.rotate_in_place(yaw_rad, yaw_rate_rps)
                
            elif primitive_id == "set_base_twist":
                vx = float(params.get("vx", 0.0))
                vy = float(params.get("vy", 0.0))
                wz = float(params.get("wz", 0.0))
                duration = float(params.get("duration", 1.0))
                return self.ros_bridge.set_base_twist(vx, vy, wz, duration)
                
            elif primitive_id == "stop":
                return self.ros_bridge.stop()
                
            else:
                # For other primitives, return basic success
                return {
                    "status": "success",
                    "primitive_id": primitive_id,
                    "parameters": params,
                    "message": f"Executed {primitive_id} successfully (ROS)",
                    "current_pose": self.ros_bridge.current_pose.to_dict() if self.ros_bridge else None
                }
                
        except Exception as e:
            logger.error(f"Error executing ROS primitive {primitive_id}: {e}")
            return {"status": "error", "message": f"ROS execution error: {str(e)}"}
    
    def _execute_joint_control(self, joint_positions: Dict[str, float], joint_limits: Dict[str, Any]) -> Dict[str, Any]:
        """Execute joint control with limit checking and ROS controller publishing."""
        logger.info(f"Controlling joints via ROS: {joint_positions}")

        # Validate joint limits
        for joint_name, position in joint_positions.items():
            if joint_name in joint_limits:
                limits = joint_limits[joint_name]
                if limits["lower"] is not None and position < limits["lower"]:
                    return {"status": "error", "message": f"Joint {joint_name} position {position} below lower limit {limits['lower']}"}
                if limits["upper"] is not None and position > limits["upper"]:
                    return {"status": "error", "message": f"Joint {joint_name} position {position} above upper limit {limits['upper']}"}

        if not self.ros_bridge:
            return {"status": "error", "message": "ROS bridge not initialized"}

        # Determine available controller from ros_interfaces
        ros_if = (self.robot_rcm or {}).get('ros_interfaces', {})
        controllers = ros_if.get('controllers', [])
        traj_ctrl = next((c for c in controllers if c.get('type') == 'trajectory' and c.get('topics', {}).get('command')), None)
        fwd_ctrl = next((c for c in controllers if c.get('type') == 'position' and c.get('topics', {}).get('command')), None)

        # Create ordered joint list; prefer /joint_states ordering if available
        joint_names = list(joint_positions.keys())
        if self.ros_bridge.latest_joint_state and self.ros_bridge.latest_joint_state.name:
            order_map = {n: i for i, n in enumerate(self.ros_bridge.latest_joint_state.name)}
            joint_names = sorted(joint_positions.keys(), key=lambda n: order_map.get(n, 1e9))
        positions = [float(joint_positions[n]) for n in joint_names]

        # Publish via trajectory controller if available; else forward position controller
        if traj_ctrl:
            result = self.ros_bridge.publish_joint_trajectory(joint_names, positions, duration=2.0)
            if result.get('status') == 'success':
                return {"status": "success", "joint_positions": dict(zip(joint_names, positions)), "message": "JointTrajectory command sent"}
            return result
        elif fwd_ctrl:
            result = self.ros_bridge.publish_forward_position(positions)
            if result.get('status') == 'success':
                return {"status": "success", "joint_positions": dict(zip(joint_names, positions)), "message": "Forward position command sent"}
            return result
        else:
            return {"status": "error", "message": "No suitable joint controller found in ros_interfaces"}

    def _execute_joint_offsets(self, joint_deltas: Dict[str, float], joint_limits: Dict[str, Any]) -> Dict[str, Any]:
        """Apply deltas to current joint positions and send as absolute control via controller."""
        if not self.ros_bridge or not self.ros_bridge.latest_joint_state:
            return {"status": "error", "message": "Current joint state not available"}

        current_names = list(self.ros_bridge.latest_joint_state.name or [])
        current_pos = list(self.ros_bridge.latest_joint_state.position or [])
        name_to_pos = {n: p for n, p in zip(current_names, current_pos)}

        target_positions: Dict[str, float] = {}
        for joint_name, delta in joint_deltas.items():
            if joint_name not in name_to_pos:
                continue
            target = float(name_to_pos[joint_name]) + float(delta)
            # Clamp to limits
            limits = joint_limits.get(joint_name, {})
            lower = limits.get("lower")
            upper = limits.get("upper")
            if lower is not None:
                target = max(lower, target)
            if upper is not None:
                target = min(upper, target)
            target_positions[joint_name] = target

        if not target_positions:
            return {"status": "error", "message": "No valid joints provided for offset"}

        return self._execute_joint_control(target_positions, joint_limits)

    def _maybe_add_servo_tools(self):
        ros_if = (self.robot_rcm or {}).get('ros_interfaces', {})
        servo_topics = ros_if.get('servo', {})
        if not servo_topics:
            return

        # delta_joint servo tool
        def servo_delta_joint_function(**kwargs):
            joint_names = kwargs.get('joint_names', [])
            deltas = kwargs.get('deltas', [])
            duration = float(kwargs.get('duration', 1.0))
            rate_hz = float(kwargs.get('rate_hz', 50.0))
            if not self.ros_bridge:
                return {"status": "error", "message": "ROS bridge not initialized"}
            return self.ros_bridge.publish_servo_delta_joint(joint_names, deltas, duration, rate_hz)

        self.tools['servo_delta_joint'] = ToolDefinition(
            name='servo_delta_joint',
            description='Apply delta joint commands using MoveIt Servo',
            parameters={
                'type': 'object',
                'properties': {
                    'joint_names': {'type': 'array', 'items': {'type': 'string'}},
                    'deltas': {'type': 'array', 'items': {'type': 'number'}},
                    'duration': {'type': 'number'},
                    'rate_hz': {'type': 'number'}
                },
                'required': ['joint_names', 'deltas']
            },
            function=servo_delta_joint_function,
            constraints={},
            primitive_id='servo_delta_joint'
        )

        # delta_twist servo tool
        def servo_delta_twist_function(**kwargs):
            linear = kwargs.get('linear', [0,0,0])
            angular = kwargs.get('angular', [0,0,0])
            frame_id = kwargs.get('frame_id', 'base_link')
            duration = float(kwargs.get('duration', 1.0))
            rate_hz = float(kwargs.get('rate_hz', 50.0))
            if not self.ros_bridge:
                return {"status": "error", "message": "ROS bridge not initialized"}
            return self.ros_bridge.publish_servo_delta_twist(linear, angular, frame_id, duration, rate_hz)

        self.tools['servo_delta_twist'] = ToolDefinition(
            name='servo_delta_twist',
            description='Apply delta Cartesian twist using MoveIt Servo',
            parameters={
                'type': 'object',
                'properties': {
                    'linear': {'type': 'array', 'items': {'type': 'number'}},
                    'angular': {'type': 'array', 'items': {'type': 'number'}},
                    'frame_id': {'type': 'string'},
                    'duration': {'type': 'number'},
                    'rate_hz': {'type': 'number'}
                },
                'required': []
            },
            function=servo_delta_twist_function,
            constraints={},
            primitive_id='servo_delta_twist'
        )

    def _maybe_add_sample_workspace_tool(self):
        workspaces = (self.robot_rcm or {}).get('workspaces', {})
        if not workspaces:
            return

        def sample_workspace_pose_function(**kwargs):
            import random
            ee = kwargs.get('end_effector')
            if not ee or ee not in workspaces:
                # Default to first available end-effector if parameter is missing or invalid
                if workspaces:
                    ee = next(iter(workspaces.keys()))
                else:
                    return {"status": "error", "message": "No workspaces available"}
            aabb = workspaces.get(ee)
            if not aabb:
                available_ees = list(workspaces.keys())
                return {"status": "error", "message": f"No workspace for end effector '{ee}'. Available: {available_ees}"}
            aabb_min = aabb.get('aabb_min', [0,0,0])
            aabb_max = aabb.get('aabb_max', [0,0,0])
            pos = [random.uniform(aabb_min[i], aabb_max[i]) for i in range(3)]
            pose = {"position": {"x": pos[0], "y": pos[1], "z": pos[2]}, "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}
            return {
                "status": "success", 
                "message": f"Sampled pose for {ee}: position({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})",
                "pose": pose, 
                "end_effector": ee,
                "sampled_position": {"x": pos[0], "y": pos[1], "z": pos[2]}
            }

        self.tools['sample_workspace_pose'] = ToolDefinition(
            name='sample_workspace_pose',
            description='Sample a random reachable pose within the end effector workspace. Returns pose data that can be used with move_to_pose or plan_to_pose tools.',
            parameters={
                'type': 'object',
                'properties': {
                    'end_effector': {'type': 'string'}
                },
                'required': []
            },
            function=sample_workspace_pose_function,
            constraints={},
            primitive_id='sample_workspace_pose'
        )

        # Also add a combined sample+move tool for convenience
        def sample_and_move_function(**kwargs):
            # First sample a pose
            sample_result = sample_workspace_pose_function(**kwargs)
            if sample_result.get("status") != "success":
                return sample_result
            
            # Then move to the sampled pose
            pose_data = sample_result.get("pose")
            if not pose_data:
                return {"status": "error", "message": "No pose data from sampling"}
            
            # Use the MoveIt planning function
            if not self.ros_bridge:
                return {"status": "error", "message": "ROS bridge not initialized"}
            
            # Convert pose to the format expected by MoveIt
            target_pose = {}
            if 'position' in pose_data:
                pos = pose_data['position']
                target_pose.update({'x': pos.get('x', 0.0), 'y': pos.get('y', 0.0), 'z': pos.get('z', 0.0)})
            if 'orientation' in pose_data:
                ori = pose_data['orientation']
                target_pose.update({
                    'qx': ori.get('x', 0.0), 'qy': ori.get('y', 0.0), 
                    'qz': ori.get('z', 0.0), 'qw': ori.get('w', 1.0)
                })
            else:
                target_pose.update({'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0})
            
            # Resolve robot-specific defaults from RCM (can be overridden by kwargs)
            defaults = self._resolve_moveit_defaults({
                'group_name': kwargs.get('group_name'),
                'ee_link': kwargs.get('ee_link'),
                'frame_id': kwargs.get('frame_id')
            })
            vel = float(kwargs.get('velocity_scale', 0.2))
            acc = float(kwargs.get('acceleration_scale', 0.2))
            # Execute the motion with derived defaults
            move_result = self.ros_bridge.plan_and_execute_pose_goal(
                target_pose, defaults['group_name'], defaults['ee_link'], defaults['frame_id'], vel, acc
            )
            
            # Combine results
            return {
                "status": move_result.get("status"),
                "message": f"Sampled and moved to pose: {sample_result.get('message', '')}. {move_result.get('message', '')}",
                "sampled_pose": sample_result.get("pose"),
                "move_result": move_result
            }

        self.tools['sample_and_move_to_pose'] = ToolDefinition(
            name='sample_and_move_to_pose',
            description='Sample a random reachable pose and immediately move to it using MoveIt. Combines sampling and motion in one step. Uses tool0 end-effector by default.',
            parameters={
                'type': 'object',
                'properties': {
                    'end_effector': {
                        'type': 'string', 
                        'description': 'End-effector name (optional, defaults to tool0)',
                        'default': 'tool0'
                    }
                },
                'required': []
            },
            function=sample_and_move_function,
            constraints={},
            primitive_id='sample_and_move_to_pose'
        )

    def _maybe_add_moveit_tools(self):
        """Add MoveIt planning/execution tools if ros_interfaces.moveit exists."""
        ros_if = (self.robot_rcm or {}).get('ros_interfaces', {})
        moveit_if = ros_if.get('moveit', {})
        if not moveit_if:
            return

        # plan_to_pose tool (uses MoveIt action client via ROS bridge)
        def plan_to_pose_function(**kwargs):
            if not self.ros_bridge:
                return {"status": "error", "message": "ROS bridge not initialized"}
            pose_data = kwargs.get('pose') or {}
            # Derive defaults from RCM, allow overrides
            defaults = self._resolve_moveit_defaults({
                'group_name': kwargs.get('group_name'),
                'ee_link': kwargs.get('ee_link'),
                'frame_id': kwargs.get('frame_id')
            })
            group = defaults['group_name']
            ee_link = defaults['ee_link']
            frame_id = defaults['frame_id']
            vel = float(kwargs.get('velocity_scale', 0.2))
            acc = float(kwargs.get('acceleration_scale', 0.2))
            
            # Convert pose structure to flat dict expected by ros_bridge
            target_pose = {}
            if 'position' in pose_data:
                pos = pose_data['position']
                target_pose.update({'x': pos.get('x', 0.0), 'y': pos.get('y', 0.0), 'z': pos.get('z', 0.0)})
            if 'orientation' in pose_data:
                ori = pose_data['orientation']
                target_pose.update({
                    'qx': ori.get('x', 0.0), 'qy': ori.get('y', 0.0), 
                    'qz': ori.get('z', 0.0), 'qw': ori.get('w', 1.0)
                })
            else:
                # Default orientation (no rotation)
                target_pose.update({'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0})
            
            return self.ros_bridge.plan_and_execute_pose_goal(
                target_pose, group, ee_link, frame_id, vel, acc
            )

        self.tools['plan_to_pose'] = ToolDefinition(
            name='plan_to_pose',
            description='Plan and execute to a target pose using MoveIt',
            parameters={
                'type': 'object',
                'properties': {
                    'pose': {
                        'type': 'object',
                        'properties': {
                            'position': {
                                'type': 'object',
                                'properties': {'x': {'type': 'number'}, 'y': {'type': 'number'}, 'z': {'type': 'number'}},
                                'required': ['x','y','z']
                            },
                            'orientation': {
                                'type': 'object',
                                'properties': {'x': {'type': 'number'}, 'y': {'type': 'number'}, 'z': {'type': 'number'}, 'w': {'type': 'number'}},
                                'required': ['w']
                            }
                        },
                        'required': ['position']
                    },
                    'group_name': {'type': 'string'},
                    'ee_link': {'type': 'string'},
                    'frame_id': {'type': 'string'},
                    'velocity_scale': {'type': 'number'},
                    'acceleration_scale': {'type': 'number'}
                },
                'required': ['pose']
            },
            function=plan_to_pose_function,
            constraints={},
            primitive_id='plan_to_pose'
        )

        # Alias: move_to_pose (same behavior)
        self.tools['move_to_pose'] = ToolDefinition(
            name='move_to_pose',
            description='Move to a target pose using MoveIt (plans then executes)',
            parameters={
                'type': 'object',
                'properties': {
                    'pose': {
                        'type': 'object',
                        'properties': {
                            'position': {
                                'type': 'object',
                                'properties': {'x': {'type': 'number'}, 'y': {'type': 'number'}, 'z': {'type': 'number'}},
                                'required': ['x','y','z']
                            },
                            'orientation': {
                                'type': 'object',
                                'properties': {'x': {'type': 'number'}, 'y': {'type': 'number'}, 'z': {'type': 'number'}, 'w': {'type': 'number'}},
                                'required': ['w']
                            }
                        },
                        'required': ['position']
                    },
                    'group_name': {'type': 'string'},
                    'ee_link': {'type': 'string'},
                    'frame_id': {'type': 'string'},
                    'velocity_scale': {'type': 'number'},
                    'acceleration_scale': {'type': 'number'}
                },
                'required': ['pose']
            },
            function=plan_to_pose_function,
            constraints={},
            primitive_id='move_to_pose'
        )

    def _maybe_add_relative_movement_tools(self):
        """Add relative movement tools if MoveIt and end-effectors are available."""
        ros_if = (self.robot_rcm or {}).get('ros_interfaces', {})
        moveit_if = ros_if.get('moveit', {})
        end_effectors = (self.robot_rcm or {}).get('end_effectors', [])
        
        if not moveit_if or not end_effectors:
            return

        # Get current pose tool
        def get_current_pose_function(**kwargs):
            if not self.ros_bridge:
                return {"status": "error", "message": "ROS bridge not initialized"}
            defaults = self._resolve_moveit_defaults({
                'ee_link': kwargs.get('link_name'),
                'frame_id': kwargs.get('frame_id')
            })
            link_name = defaults['ee_link']
            frame_id = defaults['frame_id']
            return self.ros_bridge.get_end_effector_pose(link_name, frame_id)

        self.tools['get_current_pose'] = ToolDefinition(
            name='get_current_pose',
            description='Get current end-effector pose (position and orientation)',
            parameters={
                'type': 'object',
                'properties': {
                    'link_name': {'type': 'string', 'description': 'End-effector link name', 'default': 'tool0'},
                    'frame_id': {'type': 'string', 'description': 'Reference frame', 'default': 'base_link'}
                },
                'required': []
            },
            function=get_current_pose_function,
            constraints={},
            primitive_id='get_current_pose'
        )

        # Move relative to current pose tool
        def move_relative_function(**kwargs):
            if not self.ros_bridge:
                return {"status": "error", "message": "ROS bridge not initialized"}
            
            offset = kwargs.get('offset', {})
            defaults = self._resolve_moveit_defaults({
                'ee_link': kwargs.get('link_name'),
                'frame_id': kwargs.get('frame_id'),
                'group_name': kwargs.get('group_name')
            })
            link_name = defaults['ee_link']
            frame_id = defaults['frame_id']
            group_name = defaults['group_name']
            velocity_scale = kwargs.get('velocity_scale', 0.2)
            acceleration_scale = kwargs.get('acceleration_scale', 0.2)
            
            return self.ros_bridge.move_relative_to_current_pose(
                offset, link_name, frame_id,
                group_name=group_name,
                max_velocity_scaling=velocity_scale,
                max_acceleration_scaling=acceleration_scale
            )

        self.tools['move_relative'] = ToolDefinition(
            name='move_relative',
            description='Move end-effector relative to its current pose by specified offsets (x, y, z in meters)',
            parameters={
                'type': 'object',
                'properties': {
                    'offset': {
                        'type': 'object',
                        'properties': {
                            'x': {'type': 'number', 'description': 'X offset in meters'},
                            'y': {'type': 'number', 'description': 'Y offset in meters'},
                            'z': {'type': 'number', 'description': 'Z offset in meters'}
                        },
                        'required': []
                    },
                    'link_name': {'type': 'string', 'description': 'End-effector link name', 'default': 'tool0'},
                    'frame_id': {'type': 'string', 'description': 'Reference frame', 'default': 'base_link'},
                    'group_name': {'type': 'string', 'description': 'MoveIt planning group', 'default': 'ur_manipulator'},
                    'velocity_scale': {'type': 'number', 'description': 'Velocity scaling factor (0-1)', 'default': 0.2},
                    'acceleration_scale': {'type': 'number', 'description': 'Acceleration scaling factor (0-1)', 'default': 0.2}
                },
                'required': ['offset']
            },
            function=move_relative_function,
            constraints={},
            primitive_id='move_relative'
        )

    def _maybe_add_env_tools(self):
        """Add environment-aware perception and localization tools based on available topics."""
        if not self.ros_bridge:
            return
        # Detect availability from RCM
        sensors = (self.robot_rcm or {}).get('sensors', []) or []
        ros_if = (self.robot_rcm or {}).get('ros_interfaces', {}) or {}
        has_lidar = any(isinstance(s, dict) and s.get('type') == 'lidar' for s in sensors)
        has_tf = bool(ros_if.get('tf'))
        # LIDAR-based tools
        if has_lidar:
            def is_obstacle_ahead_function(**kwargs):
                thr = float(kwargs.get('threshold_m', 0.5))
                fov = float(kwargs.get('fov_deg', 60.0))
                return self.ros_bridge.is_obstacle_ahead(threshold_m=thr, fov_deg=fov)

            self.tools['is_obstacle_ahead'] = ToolDefinition(
                name='is_obstacle_ahead',
                description='Check if an obstacle exists within forward FOV and threshold distance using LIDAR.',
                parameters={
                    'type': 'object',
                    'properties': {
                        'threshold_m': {'type': 'number', 'description': 'Distance threshold in meters', 'default': 0.5},
                        'fov_deg': {'type': 'number', 'description': 'Field of view around forward (degrees)', 'default': 60.0}
                    },
                    'required': []
                },
                function=is_obstacle_ahead_function,
                constraints={},
                primitive_id='is_obstacle_ahead'
            )

            def obstacle_report_function(**kwargs):
                bins = int(kwargs.get('bins', 12))
                return self.ros_bridge.obstacle_report(bins=bins)

            self.tools['obstacle_report'] = ToolDefinition(
                name='obstacle_report',
                description='Sectorize 360° LIDAR into N bins and report min distance per bin, closest obstacle, free sectors.',
                parameters={
                    'type': 'object',
                    'properties': {
                        'bins': {'type': 'integer', 'description': 'Number of angular sectors', 'default': 12}
                    },
                    'required': []
                },
                function=obstacle_report_function,
                constraints={},
                primitive_id='obstacle_report'
            )

            def free_direction_function(**kwargs):
                fov = float(kwargs.get('fov_deg', 30.0))
                clear = float(kwargs.get('min_clear_m', 1.0))
                return self.ros_bridge.free_direction(fov_deg=fov, min_clear_m=clear)

            self.tools['free_direction'] = ToolDefinition(
                name='free_direction',
                description='Suggest a clear forward-facing yaw sector to move into, based on LIDAR.',
                parameters={
                    'type': 'object',
                    'properties': {
                        'fov_deg': {'type': 'number', 'description': 'Sector width in degrees', 'default': 30.0},
                        'min_clear_m': {'type': 'number', 'description': 'Minimum required clearance in meters', 'default': 1.0}
                    },
                    'required': []
                },
                function=free_direction_function,
                constraints={},
                primitive_id='free_direction'
            )

        # Localization tools
        def get_pose_function(**kwargs):
            frame = str(kwargs.get('frame', 'map'))
            return self.ros_bridge.get_pose_tool(frame=frame)

        self.tools['get_pose'] = ToolDefinition(
            name='get_pose',
            description='Get robot pose in map/odom frame using TF (prefers map; falls back to odom or AMCL).',
            parameters={
                'type': 'object',
                'properties': {
                    'frame': {'type': 'string', 'description': "Frame: 'map' or 'odom'", 'default': 'map'}
                },
                'required': []
            },
            function=get_pose_function,
            constraints={},
            primitive_id='get_pose'
        )

        def where_am_i_function(**kwargs):
            return self.ros_bridge.where_am_i()

        self.tools['where_am_i'] = ToolDefinition(
            name='where_am_i',
            description='Natural-language summary of robot position and heading in map/odom.',
            parameters={'type': 'object', 'properties': {}, 'required': []},
            function=where_am_i_function,
            constraints={},
            primitive_id='where_am_i'
        )

        def describe_environment_function(**kwargs):
            return self.ros_bridge.describe_environment()

        if has_lidar or has_tf:
            self.tools['describe_environment'] = ToolDefinition(
                name='describe_environment',
                description='Concise environment summary (obstacles if LIDAR available) and current pose.',
                parameters={'type': 'object', 'properties': {}, 'required': []},
                function=describe_environment_function,
                constraints={},
                primitive_id='describe_environment'
            )

    def _maybe_add_env_object_tools(self):
        """Expose environment objects discovered in the RCM (planning scene)."""
        env = (self.robot_rcm or {}).get('environment', {}) or {}
        objects = env.get('objects', []) or []
        if not objects:
            return

        def _primitive_type_name(pt: int) -> str:
            # MoveIt primitive_type: 1=BOX,2=SPHERE,3=CYLINDER,4=CONE per shape_msgs/SolidPrimitive
            mapping = {1: 'BOX', 2: 'SPHERE', 3: 'CYLINDER', 4: 'CONE'}
            return mapping.get(int(pt), str(pt))

        def get_environment_objects_function(**kwargs):
            simplified = []
            for obj in objects:
                entry = {
                    'id': obj.get('id'),
                    'frame_id': obj.get('frame_id'),
                    'shapes': []
                }
                for shp in obj.get('shapes', []) or []:
                    entry['shapes'].append({
                        'type': _primitive_type_name(shp.get('primitive_type')),
                        'dimensions': shp.get('dimensions'),
                        'pose': shp.get('pose')
                    })
                simplified.append(entry)
            # Compose a concise message
            parts = []
            for e in simplified[:5]:
                try:
                    p = (e.get('shapes') or [{}])[0].get('pose', {})
                    pos = p.get('position') or []
                    parts.append(f"{e.get('id')} at ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) in {e.get('frame_id')}")
                except Exception:
                    parts.append(f"{e.get('id')}")
            msg = f"Found {len(simplified)} environment object(s)" + (": " + "; ".join(parts) if parts else "")
            return {'status': 'success', 'objects': simplified, 'message': msg}

        self.tools['get_environment_objects'] = ToolDefinition(
            name='get_environment_objects',
            description='List environment objects from the RCM planning scene with poses in their frame_id.',
            parameters={'type': 'object', 'properties': {}, 'required': []},
            function=get_environment_objects_function,
            constraints={},
            primitive_id='get_environment_objects'
        )
        
        # Add semantic tools
        self._add_semantic_tools()

    def _maybe_add_move_to_object_tool(self):
        env = (self.robot_rcm or {}).get('environment', {}) or {}
        objects = env.get('objects', []) or []
        ros_if = (self.robot_rcm or {}).get('ros_interfaces', {}) or {}
        moveit_if = ros_if.get('moveit', {}) or {}
        if not objects or not moveit_if:
            return

        def move_to_object_function(**kwargs):
            if not self.ros_bridge:
                return {"status": "error", "message": "ROS bridge not initialized"}
            
            # Support both object_id and natural language description
            object_id = kwargs.get('object_id')
            description = kwargs.get('description')
            approach_offset = float(kwargs.get('approach_offset_m', 0.05))
            
            logger.info(f"move_to_object called with object_id={object_id}, description={description}")
            
            # Validate that at least one parameter is provided
            if not object_id and not description:
                return {"status": "error", "message": "Provide either object_id or description"}
            
            # If description is provided, resolve it to object_id
            if description and not object_id:
                # Use the LLM resolver directly
                resolve_result = self.llm_resolver.resolve_object_description(description, objects)
                if resolve_result.get('status') != 'success':
                    return resolve_result
                object_id = resolve_result.get('object_id')
                if not object_id:
                    return {"status": "error", "message": "Could not resolve description to object ID"}
            
            # Find object by ID
            obj = next((o for o in objects if o.get('id') == object_id), None)
            if not obj:
                return {"status": "error", "message": f"Object '{object_id}' not found in environment"}
            
            logger.info(f"Found object: {obj.get('id')} at position {obj.get('shapes', [{}])[0].get('pose', {}).get('position', [])}")
            # Use first shape pose as target
            shape = (obj.get('shapes') or [{}])[0]
            pose = shape.get('pose', {})
            frame = obj.get('frame_id', 'world')
            # Compute face pose (top surface) rather than center
            dims = shape.get('dimensions') or []
            prim_type = int(shape.get('primitive_type', 1))
            face_up_offset = 0.0
            if prim_type == 1 and len(dims) >= 3:  # BOX: [x,y,z]
                face_up_offset = float(dims[2]) / 2.0
            elif prim_type == 2 and len(dims) >= 1:  # SPHERE: [radius]
                face_up_offset = float(dims[0])
            elif prim_type == 3 and len(dims) >= 2:  # CYLINDER: [height, radius]
                face_up_offset = float(dims[0]) / 2.0
            # Build a face pose in object frame by elevating along +Z
            pose_face = {
                'position': [
                    (pose.get('position') or [0,0,0])[0],
                    (pose.get('position') or [0,0,0])[1],
                    (pose.get('position') or [0,0,0])[2] + face_up_offset
                ],
                'orientation_xyzw': pose.get('orientation_xyzw') or [0,0,0,1]
            }

            # Transform to planning frame (use root_link or map/odom)
            planning_frame = (self.robot_rcm or {}).get('kinematics', {}).get('root_link', 'base_link')
            tf_res = self.ros_bridge.transform_pose(pose_face, frame, planning_frame)
            if tf_res.get('status') != 'success':
                return tf_res

            # Compute an approach position that stays outside the object volume
            # Direction from base toward object in XY plane
            import math
            bx, by = 0.0, 0.0
            ox, oy, oz = tf_res['x'], tf_res['y'], tf_res['z']
            vx, vy = ox - bx, oy - by
            norm = math.hypot(vx, vy)
            ux, uy = (vx / norm, vy / norm) if norm > 1e-6 else (0.0, 1.0)
            # Back off distance from object face: half extent in XY plus user approach offset
            if prim_type == 1 and len(dims) >= 2:  # BOX: dims[0]=x, dims[1]=y in object frame; use max as conservative
                back = 0.5 * float(max(dims[0], dims[1])) + max(0.0, approach_offset)
            elif prim_type == 2 and len(dims) >= 1:  # SPHERE
                back = float(dims[0]) + max(0.0, approach_offset)
            elif prim_type == 3 and len(dims) >= 2:  # CYLINDER dims[1]=radius
                back = float(dims[1]) + max(0.0, approach_offset)
            else:
                back = 0.1 + max(0.0, approach_offset)
            # Approach above the top face with small clearance, and backed off along -u
            clearance_z = 0.02
            target = {
                'x': ox - ux * back,
                'y': oy - uy * back,
                'z': oz + clearance_z,
                'qx': tf_res['qx'], 'qy': tf_res['qy'], 'qz': tf_res['qz'], 'qw': tf_res['qw']
            }

            # Resolve defaults and plan
            defaults = self._resolve_moveit_defaults({})
            group = defaults['group_name']
            ee_link = defaults['ee_link']
            frame_id = defaults['frame_id']

            # Pre-check workspace if available (warn only; do not hard-block)
            workspace = (self.robot_rcm or {}).get('workspaces', {}).get(ee_link)
            if workspace and isinstance(workspace, dict):
                aabb_min = workspace.get('aabb_min') or []
                aabb_max = workspace.get('aabb_max') or []
                if len(aabb_min) >= 3 and len(aabb_max) >= 3:
                    px, py, pz = target['x'], target['y'], target['z']
                    outside = not (aabb_min[0] <= px <= aabb_max[0] and aabb_min[1] <= py <= aabb_max[1] and aabb_min[2] <= pz <= aabb_max[2])
                    workspace_warning = {
                        'outside_workspace_aabb': outside,
                        'workspace_aabb': {'min': aabb_min, 'max': aabb_max}
                    }
                else:
                    workspace_warning = None
            else:
                workspace_warning = None

            # Try a series of approach offsets to improve feasibility
            attempts = []
            for extra in [0.0, 0.02, 0.05, 0.1, -0.02, -0.05]:
                t = dict(target)
                t['z'] = target['z'] + extra
                # Also try small XY sidesteps perpendicular to approach
                perp_x, perp_y = -uy, ux
                for xy_slip in [0.0, 0.03, -0.03]:
                    txy = dict(t)
                    txy['x'] = t['x'] + perp_x * xy_slip
                    txy['y'] = t['y'] + perp_y * xy_slip
                    result = self.ros_bridge.plan_and_execute_pose_goal(
                        txy,
                        group,
                        ee_link,
                        frame_id,
                        0.2,
                        0.2,
                        position_tolerance_m=0.03,
                        orientation_tolerance_rad=0.1745,
                        allow_position_only=True,
                        precheck_ik=False
                    )
                    logger.info(f"MoveIt attempt result: {result}")
                    if result.get('status') == 'success':
                        out = {"status": "success", "message": f"Moved to {object_id}", "target": txy}
                        if workspace_warning:
                            out['workspace_warning'] = workspace_warning
                        return out
                    attempts.append({'z_offset': extra, 'xy_slip': xy_slip, 'result': result.get('message', 'plan_failed')})

            # All attempts failed, report best reason
            return {
                'status': 'error',
                'message': 'Failed to reach object after multiple approach offsets',
                'attempts': attempts,
                'target_base': target,
                'object_id': object_id
            }

        self.tools['move_to_object'] = ToolDefinition(
            name='move_to_object',
            description='Move the robot arm to an object. Use this tool when asked to "move to [object]" or "go to [object]". Supports natural language descriptions like "small box on table", "red tote", etc. This tool handles object resolution automatically - no need to call resolve_object_description first.',
            parameters={
                'type': 'object',
                'properties': {
                    'object_id': {'type': 'string', 'description': 'ID of the environment object from get_environment_objects'},
                    'description': {'type': 'string', 'description': 'Natural language description of the object (e.g., "small box on table", "red tote")'},
                    'approach_offset_m': {'type': 'number', 'description': 'Z offset (meters) to approach above the object', 'default': 0.05}
                }
            },
            function=move_to_object_function,
            constraints={},
            primitive_id='move_to_object'
        )
    
    def _read_sensor(self, sensor: Dict[str, Any]) -> Dict[str, Any]:
        """Read sensor data via ROS"""
        logger.info(f"Reading sensor via ROS: {sensor}")
        
        if not self._ros_initialized or not self.ros_bridge:
            return {"status": "error", "message": "ROS bridge not initialized"}
        
        try:
            sensor_type = sensor.get("type")
            
            if sensor_type == "lidar":
                return self.ros_bridge.read_lidar_scan()
            elif sensor_type == "imu":
                return self.ros_bridge.read_imu_data()
            else:
                # For other sensor types, return basic info
                return {
                    "status": "success", 
                    "data": {"sensor_type": sensor_type, "message": "Sensor data via ROS"},
                    "message": f"Read {sensor_type} sensor via ROS"
                }
                
        except Exception as e:
            logger.error(f"Error reading sensor {sensor}: {e}")
            return {"status": "error", "message": f"Sensor read error: {str(e)}"}
    
    def _validate_parameters(self, params: Dict[str, Any], constraints: Dict[str, Any]) -> Dict[str, Any]:
        """Validate parameters against constraints"""
        # Basic validation - can be extended
        preconditions = constraints.get("preconditions", [])
        
        # Check battery level constraint
        if "battery_level>0.2" in preconditions:
            # In real implementation, check actual battery level via ROS
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
    
    # State tracking methods (delegated to ROS bridge)
    @property
    def state_tracker(self):
        """Property to maintain compatibility with existing code"""
        return self.ros_bridge
    
    def get_status(self) -> Dict[str, Any]:
        """Get current robot status"""
        if self.ros_bridge:
            return self.ros_bridge.get_status()
        else:
            return {"status": "error", "message": "ROS bridge not available"}
    
    def get_path_visualization(self) -> str:
        """Get path visualization"""
        if self.ros_bridge:
            return self.ros_bridge.get_path_visualization()
        else:
            return "ROS bridge not available"
    
    def reset_position(self, new_pose=None):
        """Reset robot position"""
        if self.ros_bridge:
            return self.ros_bridge.reset_position(new_pose)
        else:
            return {"status": "error", "message": "ROS bridge not available"}

    def _add_semantic_tools(self):
        """Add semantic environment tools based on RCM environment data"""
        env = (self.robot_rcm or {}).get('environment', {}) or {}
        objects = env.get('objects', []) or []
        
        if not objects:
            return
        
        def get_semantic_map_function(**kwargs):
            """Get semantic map of environment with object classes, affordances, and zones"""
            zone_filter = kwargs.get('zone_filter')
            class_filter = kwargs.get('class_filter')
            
            filtered_objects = objects
            if zone_filter:
                filtered_objects = [obj for obj in filtered_objects if obj.get('zone') == zone_filter]
            if class_filter:
                filtered_objects = [obj for obj in filtered_objects if obj.get('class') == class_filter]
            
            # Extract semantic information
            semantic_objects = []
            for obj in filtered_objects:
                semantic_obj = {
                    'id': obj.get('id'),
                    'class': obj.get('class', 'unknown'),
                    'affordances': obj.get('affordances', []),
                    'zone': obj.get('zone', 'general'),
                    'properties': obj.get('properties', {}),
                    'pose': obj.get('shapes', [{}])[0].get('pose', {}) if obj.get('shapes') else {}
                }
                semantic_objects.append(semantic_obj)
            
            # Get unique zones and classes
            zones = list(set(obj.get('zone', 'general') for obj in objects))
            classes = list(set(obj.get('class', 'unknown') for obj in objects))
            
            # Create detailed message with object information
            if semantic_objects:
                details = []
                for obj in semantic_objects[:10]:  # Show first 10 objects
                    pos = obj.get('pose', {}).get('position', [])
                    pos_str = f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})" if len(pos) >= 3 else "unknown position"
                    details.append(f"- {obj['id']} ({obj['class']}) at {pos_str} - zone: {obj['zone']}, affordances: {obj['affordances']}")
                
                message = f"Semantic map: {len(semantic_objects)} objects, {len(classes)} classes, {len(zones)} zones\n\nObjects:\n" + "\n".join(details)
                if len(semantic_objects) > 10:
                    message += f"\n... and {len(semantic_objects) - 10} more objects"
            else:
                message = "No objects found in the environment"
            
            return {
                'status': 'success',
                'objects': semantic_objects,
                'summary': {
                    'total_objects': len(semantic_objects),
                    'object_classes': classes,
                    'zones': zones
                },
                'message': message
            }
        
        def find_objects_by_semantics_function(**kwargs):
            """Find objects by semantic properties (class, affordance, zone)"""
            class_name = kwargs.get('class')
            affordance = kwargs.get('affordance')
            zone = kwargs.get('zone')
            
            matches = []
            for obj in objects:
                # Check class match
                if class_name and obj.get('class') != class_name:
                    continue
                # Check affordance match
                if affordance and affordance not in obj.get('affordances', []):
                    continue
                # Check zone match
                if zone and obj.get('zone') != zone:
                    continue
                
                matches.append({
                    'id': obj['id'],
                    'class': obj.get('class', 'unknown'),
                    'affordances': obj.get('affordances', []),
                    'zone': obj.get('zone', 'general'),
                    'properties': obj.get('properties', {}),
                    'pose': obj.get('shapes', [{}])[0].get('pose', {}) if obj.get('shapes') else {}
                })
            
            # Create detailed message with object information
            if matches:
                details = []
                for match in matches:
                    pos = match.get('pose', {}).get('position', [])
                    pos_str = f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})" if len(pos) >= 3 else "unknown position"
                    details.append(f"- {match['id']} ({match['class']}) at {pos_str} - affordances: {match['affordances']}")
                message = f"Found {len(matches)} objects matching criteria:\n" + "\n".join(details)
            else:
                message = "No objects found matching the specified criteria"
            
            return {
                'status': 'success',
                'matches': matches,
                'count': len(matches),
                'message': message
            }
        
        def suggest_place_locations_function(**kwargs):
            """Suggest where to place an object based on its class and constraints"""
            object_class = kwargs.get('object_class', 'unknown')
            preferred_zone = kwargs.get('preferred_zone')
            avoid_collision = kwargs.get('avoid_collision', True)
            
            # Find placeable surfaces
            placeable_objects = [obj for obj in objects if 'placeable' in obj.get('affordances', [])]
            
            suggestions = []
            for obj in placeable_objects:
                # Skip if zone preference doesn't match
                if preferred_zone and obj.get('zone') != preferred_zone:
                    continue
                
                # Get object dimensions and pose
                if obj.get('shapes'):
                    shape = obj['shapes'][0]
                    dimensions = shape.get('dimensions', [])
                    pose = shape.get('pose', {})
                    position = pose.get('position', [0, 0, 0])
                    
                    # Calculate suggested placement positions
                    if len(dimensions) >= 3 and len(position) >= 3:
                        width, height, depth = dimensions[0], dimensions[1], dimensions[2]
                        x, y, z = position[0], position[1], position[2]
                        
                        # Suggest positions on the surface
                        surface_z = z + height/2 + 0.05  # 5cm above surface
                        
                        # Generate grid of placement positions
                        for dx in [-width/4, 0, width/4]:
                            for dy in [-depth/4, 0, depth/4]:
                                if dx == 0 and dy == 0:
                                    continue  # Skip center
                                
                                suggestions.append({
                                    'position': [x + dx, y + dy, surface_z],
                                    'surface_id': obj['id'],
                                    'surface_class': obj.get('class', 'unknown'),
                                    'zone': obj.get('zone', 'general'),
                                    'reason': f"On {obj.get('class', 'surface')} {obj['id']}"
                                })
            
            # Create detailed message with placement suggestions
            if suggestions:
                details = []
                for i, suggestion in enumerate(suggestions[:5]):  # Show first 5 suggestions
                    pos = suggestion['position']
                    pos_str = f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
                    details.append(f"{i+1}. {pos_str} - {suggestion['reason']}")
                
                message = f"Found {len(suggestions)} placement suggestions for {object_class}:\n" + "\n".join(details)
                if len(suggestions) > 5:
                    message += f"\n... and {len(suggestions) - 5} more suggestions"
            else:
                message = f"No placement suggestions found for {object_class}"
            
            return {
                'status': 'success',
                'suggestions': suggestions[:10],  # Limit to 10 suggestions
                'count': len(suggestions),
                'message': message
            }
        
        def resolve_object_description_function(**kwargs):
            """Resolve natural language descriptions to specific object IDs using LLM reasoning"""
            description = kwargs.get('description', '')
            
            if not description:
                return {
                    'status': 'error',
                    'message': 'No description provided'
                }
            
            # Use LLM resolver for intelligent reasoning
            result = self.llm_resolver.resolve_object_description(description, objects)
            
            # If successful, format the response to match the expected structure
            if result['status'] == 'success':
                # Add position information to the message
                obj = result['object']
                pos = obj.get('shapes', [{}])[0].get('pose', {}).get('position', [])
                pos_str = f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})" if len(pos) >= 3 else "unknown position"
                
                # Format alternatives
                alternatives_msg = ""
                if result.get('alternatives'):
                    alternatives_msg = "\nOther possibilities:"
                    for alt in result['alternatives']:
                        alternatives_msg += f"\n- {alt['object_id']} (confidence: {alt['confidence']:.2f}) - {alt['reasoning']}"
                
                # Create enhanced message
                enhanced_message = f"Best match: {result['object_id']} at {pos_str} (confidence: {result['confidence']:.2f})\n{result['reasoning']}{alternatives_msg}"
                
                return {
                    'status': 'success',
                    'best_match': result['object'],
                    'object_id': result['object_id'],
                    'confidence': result['confidence'],
                    'reasoning': result['reasoning'],
                    'alternatives': result.get('alternatives', []),
                    'analysis': result.get('analysis', {}),
                    'all_matches': [result['object']] + [alt.get('object', {}) for alt in result.get('alternatives', [])],
                    'message': enhanced_message
                }
            else:
                return result
        
        # Add the semantic tools
        self.tools['resolve_object_description'] = ToolDefinition(
            name='resolve_object_description',
            description='Get information about objects by description (e.g., "small box on table" -> object details). Use this for queries like "what is the small box" or "show me objects". For movement, use move_to_object instead.',
            parameters={
                'type': 'object',
                'properties': {
                    'description': {'type': 'string', 'description': 'Natural language description of the object'}
                },
                'required': ['description']
            },
            function=resolve_object_description_function,
            constraints={},
            primitive_id='resolve_object_description'
        )
        
        self.tools['get_semantic_map'] = ToolDefinition(
            name='get_semantic_map',
            description='Get a semantic map of the environment with object classes, affordances, and zones',
            parameters={
                'type': 'object',
                'properties': {
                    'zone_filter': {'type': 'string', 'description': 'Filter by zone (optional)'},
                    'class_filter': {'type': 'string', 'description': 'Filter by object class (optional)'}
                }
            },
            function=get_semantic_map_function,
            constraints={},
            primitive_id='get_semantic_map'
        )
        
        self.tools['find_objects_by_semantics'] = ToolDefinition(
            name='find_objects_by_semantics',
            description='Find objects by semantic properties (class, affordance, zone)',
            parameters={
                'type': 'object',
                'properties': {
                    'class': {'type': 'string', 'description': 'Object class (e.g., tote, workstation)'},
                    'affordance': {'type': 'string', 'description': 'Required affordance (e.g., pickable)'},
                    'zone': {'type': 'string', 'description': 'Zone name (e.g., workstation_1)'}
                }
            },
            function=find_objects_by_semantics_function,
            constraints={},
            primitive_id='find_objects_by_semantics'
        )
        
        self.tools['suggest_place_locations'] = ToolDefinition(
            name='suggest_place_locations',
            description='Suggest where to place an object based on its class and constraints',
            parameters={
                'type': 'object',
                'properties': {
                    'object_class': {'type': 'string', 'description': 'Class of object to place'},
                    'preferred_zone': {'type': 'string', 'description': 'Preferred zone (optional)'},
                    'avoid_collision': {'type': 'boolean', 'description': 'Avoid existing objects', 'default': True}
                }
            },
            function=suggest_place_locations_function,
            constraints={},
            primitive_id='suggest_place_locations'
        )
        
        # Add a simple "what is" tool for testing
        def what_is_object_function(**kwargs):
            """Get information about an object by description or ID"""
            description = kwargs.get('description', '')
            object_id = kwargs.get('object_id', '')
            
            if not description and not object_id:
                return {"status": "error", "message": "Provide either description or object_id"}
            
            # If description provided, resolve it first
            if description:
                resolve_result = resolve_object_description_function(description=description)
                if resolve_result.get('status') != 'success':
                    return resolve_result
                object_id = resolve_result.get('object_id')
            
            # Find the object
            obj = next((o for o in objects if o.get('id') == object_id), None)
            if not obj:
                return {"status": "error", "message": f"Object '{object_id}' not found"}
            
            # Format detailed information
            pos = obj.get('shapes', [{}])[0].get('pose', {}).get('position', [])
            pos_str = f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})" if len(pos) >= 3 else "unknown position"
            
            message = f"Object: {obj['id']}\n"
            message += f"Class: {obj.get('class', 'unknown')}\n"
            message += f"Position: {pos_str}\n"
            message += f"Zone: {obj.get('zone', 'general')}\n"
            message += f"Affordances: {obj.get('affordances', [])}\n"
            message += f"Properties: {obj.get('properties', {})}"
            
            return {
                'status': 'success',
                'object': obj,
                'message': message
            }
        
        self.tools['what_is_object'] = ToolDefinition(
            name='what_is_object',
            description='Get detailed information about an object by description or ID (provide either description or object_id)',
            parameters={
                'type': 'object',
                'properties': {
                    'description': {'type': 'string', 'description': 'Natural language description of the object'},
                    'object_id': {'type': 'string', 'description': 'Exact object ID'}
                }
            },
            function=what_is_object_function,
            constraints={},
            primitive_id='what_is_object'
        )


# Compatibility function for easy migration
def create_tool_generator(use_ros: bool = True, robot_namespace: str = "") -> 'ROSToolGenerator':
    """Create appropriate tool generator based on configuration"""
    if use_ros:
        return ROSToolGenerator(robot_namespace)
    else:
        # Fall back to original tool generator
        from tool_generator import RCMToolGenerator
        return RCMToolGenerator()


# Example usage
def main():
    """Test the ROS tool generator"""
    import asyncio
    
    async def test_ros_tools():
        # Load example RCM
        with open("turtlebot_rcm.json", "r") as f:
            rcm = json.load(f)
        
        # Create ROS tool generator
        tool_gen = ROSToolGenerator()
        
        # Load RCM and generate tools
        tool_gen.load_rcm(rcm)
        
        print(f"Generated {len(tool_gen.tools)} tools:")
        for name, tool in tool_gen.tools.items():
            print(f"  - {name}: {tool.description}")
        
        # Test a movement command
        print("\nTesting drive_straight...")
        result = tool_gen.execute_tool("drive_straight", distance_m=1.0, speed_mps=0.2)
        print("Result:", result)
        
        # Test sensor reading
        print("\nTesting sensor read...")
        result = tool_gen.execute_tool("read_lidar_base_scan")
        print("Result:", result)
        
        # Cleanup
        tool_gen.shutdown_ros()
    
    # Run test
    asyncio.run(test_ros_tools())


if __name__ == "__main__":
    main()
