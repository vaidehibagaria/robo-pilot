#!/usr/bin/env python3
"""
ROS Bridge - Interfaces RCM framework with ROS/Gazebo simulation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import logging
import asyncio
import threading
from typing import Dict, Any, Optional
from dataclasses import dataclass
import time

# ROS 2 message imports
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2, Imu, JointState
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
try:
    from control_msgs.msg import JointJog
except Exception:
    JointJog = None
try:
    # MoveIt 2 interfaces
    from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetMotionPlan, GetPositionIK
    from moveit_msgs.action import MoveGroup
    from shape_msgs.msg import SolidPrimitive
except Exception:
    GetMotionPlan = None
    GetPositionIK = None
    MoveGroup = None
    SolidPrimitive = None
try:
    # TF2 for transform lookups
    import tf2_ros
    from tf2_geometry_msgs import do_transform_pose, do_transform_pose_stamped
    from geometry_msgs.msg import TransformStamped
except Exception:
    tf2_ros = None
    try:
        from tf2_geometry_msgs import do_transform_pose  # may still exist
    except Exception:
        do_transform_pose = None
    try:
        from tf2_geometry_msgs import do_transform_pose_stamped  # may not exist
    except Exception:
        do_transform_pose_stamped = None
    TransformStamped = None
try:
    from control_msgs.msg import JointTrajectoryControllerState
except Exception:
    JointTrajectoryControllerState = None

logger = logging.getLogger(__name__)

@dataclass
class RobotPose:
    """Represents robot position and orientation"""
    x: float = 0.0  # meters
    y: float = 0.0  # meters
    theta: float = 0.0  # radians (heading)
    timestamp: float = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "x": round(self.x, 3),
            "y": round(self.y, 3), 
            "theta": round(self.theta, 3),
            "theta_deg": round(math.degrees(self.theta), 1),
            "timestamp": self.timestamp
        }

class ROSBridge(Node):
    """ROS Bridge for interfacing with Gazebo simulation"""
    
    def __init__(self, robot_namespace: str = ""):
        super().__init__('rcm_ros_bridge')
        
        self.robot_namespace = robot_namespace
        self.current_pose = RobotPose()
        self.pose_history = [self.current_pose]
        self.total_distance = 0.0
        self.total_rotation = 0.0
        
        # Sensor data storage
        self.latest_scan = None
        self.latest_imu = None
        self.latest_pointcloud = None
        self.latest_amcl = None
        
        # Movement tracking
        self.last_pose_for_distance = self.current_pose

        # Joint state tracking
        self.latest_joint_state: Optional[JointState] = None
        self.joint_name_to_index: dict = {}

        # Controller publishers (configured later)
        self._traj_pub = None
        self._forward_position_pub = None
        self._servo_delta_joint_pub = None
        self._servo_delta_twist_pub = None
        # Expected controller joint order (learned from controller state)
        self._traj_ctrl_joint_names: Optional[list] = None

        # MoveIt planning client
        self._moveit_client = None
        
        # TF buffer and listener for pose tracking
        if tf2_ros:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        else:
            self.tf_buffer = None
            self.tf_listener = None
        
        self._setup_publishers()
        self._setup_subscribers()
        self._setup_action_clients()
        
        # Planning/IK metrics
        self.metrics = {
            'plans_requested': 0,
            'plans_succeeded': 0,
            'plans_failed': 0,
            'ik_requests': 0,
            'ik_success': 0,
            'ik_failed': 0,
            'last_error': None,
        }
        self._last_move_goal_handle = None

        logger.info(f"ROS Bridge initialized for robot namespace: '{robot_namespace}'")
    
    def _setup_publishers(self):
        """Setup ROS publishers"""
        # Velocity command publisher
        cmd_vel_topic = f"{self.robot_namespace}/cmd_vel" if self.robot_namespace else "/cmd_vel"
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        
        # Initial pose publisher (for localization)
        initial_pose_topic = f"{self.robot_namespace}/initialpose" if self.robot_namespace else "/initialpose"
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, initial_pose_topic, 10)
        
        logger.info(f"Publishers setup: cmd_vel={cmd_vel_topic}")
    
    def _setup_subscribers(self):
        """Setup ROS subscribers"""
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Odometry subscriber
        odom_topic = f"{self.robot_namespace}/odom" if self.robot_namespace else "/odom"
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_callback, qos_profile
        )
        
        # LaserScan subscriber
        scan_topic = f"{self.robot_namespace}/scan" if self.robot_namespace else "/scan"
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, qos_profile
        )
        
        # IMU subscriber (if available)
        imu_topic = f"{self.robot_namespace}/imu" if self.robot_namespace else "/imu"
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self._imu_callback, qos_profile
        )

        # AMCL pose (optional)
        amcl_topic = f"{self.robot_namespace}/amcl_pose" if self.robot_namespace else "/amcl_pose"
        try:
            self.create_subscription(PoseWithCovarianceStamped, amcl_topic, self._amcl_pose_callback, qos_profile)
        except Exception:
            pass

        # Joint states subscriber (if available)
        joint_states_topic = f"{self.robot_namespace}/joint_states" if self.robot_namespace else "/joint_states"
        self.joint_states_sub = self.create_subscription(
            JointState, joint_states_topic, self._joint_states_callback, qos_profile
        )
        
        logger.info(f"Subscribers setup: odom={odom_topic}, scan={scan_topic}")
    
    def _setup_action_clients(self):
        """Setup ROS action clients"""
        # Navigation action client
        nav_action = f"{self.robot_namespace}/navigate_to_pose" if self.robot_namespace else "/navigate_to_pose"
        self.nav_client = ActionClient(self, NavigateToPose, nav_action)
        
        # MoveIt action client for motion planning
        if MoveGroup:
            move_action = f"{self.robot_namespace}/move_action" if self.robot_namespace else "/move_action"
            self._moveit_client = ActionClient(self, MoveGroup, move_action)
            logger.info(f"Action clients setup: navigate_to_pose={nav_action}, move_action={move_action}")
        else:
            logger.warning("MoveIt action interfaces not available")
            logger.info(f"Action clients setup: navigate_to_pose={nav_action}")
    
    def _odom_callback(self, msg: Odometry):
        """Handle odometry messages"""
        # Extract position
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Convert quaternion to yaw angle
        yaw = self._quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
        
        # Update current pose
        old_pose = self.current_pose
        self.current_pose = RobotPose(position.x, position.y, yaw)
        
        # Update distance tracking
        if old_pose:
            distance_moved = math.sqrt(
                (self.current_pose.x - old_pose.x)**2 + 
                (self.current_pose.y - old_pose.y)**2
            )
            if distance_moved > 0.01:  # Only count significant movements
                self.total_distance += distance_moved
                self.pose_history.append(self.current_pose)
                
                # Update rotation tracking
                angle_diff = abs(self.current_pose.theta - old_pose.theta)
                if angle_diff > math.pi:
                    angle_diff = 2 * math.pi - angle_diff
                if angle_diff > 0.01:  # Only count significant rotations
                    self.total_rotation += angle_diff
    
    def _scan_callback(self, msg: LaserScan):
        """Handle laser scan messages"""
        self.latest_scan = {
            "ranges": list(msg.ranges),
            "angle_min": msg.angle_min,
            "angle_max": msg.angle_max,
            "angle_increment": msg.angle_increment,
            "time_increment": msg.time_increment,
            "scan_time": msg.scan_time,
            "range_min": msg.range_min,
            "range_max": msg.range_max,
            "timestamp": time.time()
        }
    
    def _imu_callback(self, msg: Imu):
        """Handle IMU messages"""
        self.latest_imu = {
            "orientation": [
                msg.orientation.x, msg.orientation.y, 
                msg.orientation.z, msg.orientation.w
            ],
            "angular_velocity": [
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
            ],
            "linear_acceleration": [
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
            ],
            "timestamp": time.time()
        }

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Handle AMCL pose (if present)"""
        try:
            cov = list(msg.pose.covariance) if hasattr(msg.pose, 'covariance') else None
            self.latest_amcl = {
                "frame_id": getattr(msg.header, 'frame_id', ''),
                "position": [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
                "orientation": [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w],
                "covariance": cov,
                "timestamp": time.time()
            }
        except Exception:
            self.latest_amcl = {"error": "parse_failed", "timestamp": time.time()}

    def _joint_states_callback(self, msg: JointState):
        """Handle joint states messages"""
        self.latest_joint_state = msg
        # Build name->index map once
        if msg and msg.name and not self.joint_name_to_index:
            self.joint_name_to_index = {n: i for i, n in enumerate(msg.name)}
    
    def _quaternion_to_yaw(self, x, y, z, w):
        """Convert quaternion to yaw angle"""
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    
    # RCM-compatible methods
    def drive_straight(self, distance_m: float, speed_mps: float = 0.5) -> Dict[str, Any]:
        """Drive straight for a specified distance"""
        if distance_m == 0:
            return {"status": "success", "message": "No movement requested", "pose": self.current_pose.to_dict()}
        
        old_pose = self.current_pose
        
        # Create and publish velocity command
        twist = Twist()
        twist.linear.x = speed_mps if distance_m > 0 else -speed_mps
        twist.angular.z = 0.0
        
        # Calculate duration based on distance and speed
        duration = abs(distance_m) / speed_mps
        
        # Publish velocity command
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)  # 10 Hz publishing rate
        
        # Stop the robot
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        
        # Wait a moment for odometry to update
        time.sleep(0.2)
        
        return {
            "status": "success",
            "action": "drive_straight",
            "parameters": {"distance_m": distance_m, "speed_mps": speed_mps},
            "old_pose": old_pose.to_dict(),
            "new_pose": self.current_pose.to_dict(),
            "message": f"Drove {distance_m}m to position ({self.current_pose.x:.2f}, {self.current_pose.y:.2f})"
        }
    
    def rotate_in_place(self, yaw_rad: float, yaw_rate_rps: float = 0.5) -> Dict[str, Any]:
        """Rotate in place by specified angle"""
        if yaw_rad == 0:
            return {"status": "success", "message": "No rotation requested", "pose": self.current_pose.to_dict()}
        
        old_pose = self.current_pose
        
        # Create and publish rotation command
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = yaw_rate_rps if yaw_rad > 0 else -yaw_rate_rps
        
        # Calculate duration based on angle and rate
        duration = abs(yaw_rad) / yaw_rate_rps
        
        # Publish rotation command
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)  # 10 Hz publishing rate
        
        # Stop the robot
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        
        # Wait a moment for odometry to update
        time.sleep(0.2)
        
        return {
            "status": "success",
            "action": "rotate_in_place",
            "parameters": {"yaw_rad": yaw_rad, "yaw_rate_rps": yaw_rate_rps},
            "old_pose": old_pose.to_dict(),
            "new_pose": self.current_pose.to_dict(),
            "rotation_rad": yaw_rad,
            "rotation_deg": round(math.degrees(yaw_rad), 1),
            "message": f"Rotated {math.degrees(yaw_rad):.1f}° to heading {self.current_pose.theta_deg:.1f}°"
        }
    
    def set_base_twist(self, vx: float, vy: float, wz: float, duration: float = 1.0) -> Dict[str, Any]:
        """Set base velocity for specified duration"""
        if vx == 0 and vy == 0 and wz == 0:
            return {"status": "success", "message": "No velocity commanded", "pose": self.current_pose.to_dict()}
        
        old_pose = self.current_pose
        
        # Create and publish velocity command
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy  # Note: Most robots don't support lateral movement
        twist.angular.z = wz
        
        # Publish velocity command for duration
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)  # 10 Hz publishing rate
        
        # Stop the robot
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        
        # Wait a moment for odometry to update
        time.sleep(0.2)
        
        return {
            "status": "success",
            "action": "set_base_twist",
            "parameters": {"vx": vx, "vy": vy, "wz": wz, "duration": duration},
            "old_pose": old_pose.to_dict(),
            "new_pose": self.current_pose.to_dict(),
            "message": f"Applied velocity for {duration}s, moved to ({self.current_pose.x:.2f}, {self.current_pose.y:.2f})"
        }
    
    def stop(self) -> Dict[str, Any]:
        """Stop the robot"""
        twist = Twist()  # All zeros
        self.cmd_vel_pub.publish(twist)
        
        return {
            "status": "success",
            "action": "stop",
            "message": "Robot stopped",
            "current_pose": self.current_pose.to_dict()
        }

    # ========== Manipulator/Controllers Configuration and Commands ==========
    def configure_controllers(self, controllers: list, servo_topics: dict = None):
        """Create publishers for available controllers and servo from RCM ros_interfaces.

        controllers: list of dicts with fields {name, type, topics{command,state,...}}
        servo_topics: optional dict with keys delta_joint_cmds, delta_twist_cmds
        """
        # Trajectory controller
        try:
            traj_ctrl = next((c for c in controllers or [] if c.get('type') == 'trajectory' and c.get('topics', {}).get('command')), None)
            if traj_ctrl and not self._traj_pub:
                topic = traj_ctrl['topics']['command']
                # Match controller QoS (typically BEST_EFFORT + VOLATILE)
                qos_ctrl = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                    depth=10
                )
                self._traj_pub = self.create_publisher(JointTrajectory, topic, qos_ctrl)
                logger.info(f"Trajectory controller publisher created: {topic} (BEST_EFFORT/VOLATILE)")
                # Subscribe to controller_state to learn expected joint order
                if JointTrajectoryControllerState and traj_ctrl.get('topics', {}).get('controller_state'):
                    state_topic = traj_ctrl['topics']['controller_state']
                    self.create_subscription(
                        JointTrajectoryControllerState,
                        state_topic,
                        self._traj_controller_state_cb,
                        10
                    )
                    logger.info(f"Subscribed to controller state: {state_topic}")
        except Exception as e:
            logger.warning(f"Failed to configure trajectory controller: {e}")

        # Forward position controller
        try:
            fwd_ctrl = next((c for c in controllers or [] if c.get('type') == 'position' and c.get('topics', {}).get('command')), None)
            if fwd_ctrl and not self._forward_position_pub:
                topic = fwd_ctrl['topics']['command']
                self._forward_position_pub = self.create_publisher(Float64MultiArray, topic, 10)
                logger.info(f"Forward position controller publisher created: {topic}")
        except Exception as e:
            logger.warning(f"Failed to configure forward position controller: {e}")

        # Servo
        try:
            if servo_topics:
                # Match MoveIt Servo subscribers (BEST_EFFORT, VOLATILE)
                qos_be = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                    depth=10
                )
                if servo_topics.get('delta_joint_cmds') and JointJog and not self._servo_delta_joint_pub:
                    self._servo_delta_joint_pub = self.create_publisher(JointJog, servo_topics['delta_joint_cmds'], qos_be)
                    logger.info(f"Servo delta_joint publisher created: {servo_topics['delta_joint_cmds']}")
                if servo_topics.get('delta_twist_cmds') and not self._servo_delta_twist_pub:
                    self._servo_delta_twist_pub = self.create_publisher(TwistStamped, servo_topics['delta_twist_cmds'], qos_be)
                    logger.info(f"Servo delta_twist publisher created: {servo_topics['delta_twist_cmds']}")
        except Exception as e:
            logger.warning(f"Failed to configure servo publishers: {e}")

        # Configure MoveIt service client if available
        try:
            if GetMotionPlan and self._moveit_client is None:
                # Typical service name when MoveGroup is running
                self._moveit_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
                logger.info("MoveIt GetMotionPlan client created: /plan_kinematic_path")
        except Exception as e:
            logger.warning(f"Failed to create MoveIt client: {e}")
        # IK service client (optional)
        try:
            if GetPositionIK:
                self._ik_client = self.create_client(GetPositionIK, '/compute_ik')
                logger.info("MoveIt IK client created: /compute_ik")
            else:
                self._ik_client = None
        except Exception as e:
            logger.warning(f"Failed to create IK client: {e}")

    def publish_joint_trajectory(self, joint_names: list, positions: list, duration: float = 2.0) -> Dict[str, Any]:
        """Publish a single-point JointTrajectory to the configured trajectory controller."""
        if not self._traj_pub:
            return {"status": "error", "message": "Trajectory controller not configured"}
        try:
            # Determine expected joint order
            expected = self._traj_ctrl_joint_names or (self.latest_joint_state.name if self.latest_joint_state else list(joint_names))
            # Build a full positions vector matching expected order, filling unspecified joints from latest state
            target_map = {n: p for n, p in zip(joint_names, positions)}
            current_map = {}
            if self.latest_joint_state:
                for n, p in zip(self.latest_joint_state.name, self.latest_joint_state.position):
                    current_map[n] = p
            ordered_names = []
            ordered_pos = []
            for n in expected:
                if n in target_map:
                    ordered_names.append(n)
                    ordered_pos.append(float(target_map[n]))
                elif n in current_map:
                    ordered_names.append(n)
                    ordered_pos.append(float(current_map[n]))
            if not ordered_names:
                return {"status": "error", "message": "No matching joints found for trajectory controller"}

            msg = JointTrajectory()
            msg.joint_names = ordered_names
            point = JointTrajectoryPoint()
            point.positions = ordered_pos
            point.time_from_start = rclpy.duration.Duration(seconds=float(max(0.1, duration))).to_msg()
            msg.points = [point]
            self._traj_pub.publish(msg)
            return {"status": "success", "message": f"Sent JointTrajectory with {len(joint_names)} joints"}
        except Exception as e:
            logger.error(f"Error publishing JointTrajectory: {e}")
            return {"status": "error", "message": str(e)}

    def _traj_controller_state_cb(self, msg: 'JointTrajectoryControllerState'):
        try:
            if msg and hasattr(msg, 'joint_names') and msg.joint_names:
                self._traj_ctrl_joint_names = list(msg.joint_names)
        except Exception:
            pass

    # ========== IK and Explainable Errors Helpers ==========
    def _compute_ik(self, target_pose: Dict[str, float], group_name: str, ee_link: str, frame_id: str, timeout_sec: float = 1.0) -> Dict[str, Any]:
        """Call MoveIt GetPositionIK if available; return {'status', 'joints'| 'message'}."""
        if GetPositionIK is None or getattr(self, '_ik_client', None) is None:
            return {"status": "error", "message": "IK service not available"}
        try:
            if not self._ik_client.wait_for_service(timeout_sec=timeout_sec):
                return {"status": "error", "message": "IK service not responding"}
            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.pose.position.x = float(target_pose.get('x', 0.0))
            ps.pose.position.y = float(target_pose.get('y', 0.0))
            ps.pose.position.z = float(target_pose.get('z', 0.0))
            ps.pose.orientation.x = float(target_pose.get('qx', 0.0))
            ps.pose.orientation.y = float(target_pose.get('qy', 0.0))
            ps.pose.orientation.z = float(target_pose.get('qz', 0.0))
            ps.pose.orientation.w = float(target_pose.get('qw', 1.0))

            req = GetPositionIK.Request()
            req.ik_request.group_name = group_name
            req.ik_request.ik_link_name = ee_link
            req.ik_request.pose_stamped = ps
            req.ik_request.timeout = rclpy.duration.Duration(seconds=timeout_sec).to_msg()
            req.ik_request.attempts = 5
            if self.latest_joint_state:
                seed = RobotState()
                seed.joint_state = self.latest_joint_state
                req.ik_request.robot_state = seed

            # metrics
            self.metrics['ik_requests'] = self.metrics.get('ik_requests', 0) + 1

            future = self._ik_client.call_async(req)
            start = time.time()
            while rclpy.ok() and not future.done() and (time.time() - start) < timeout_sec:
                rclpy.spin_once(self, timeout_sec=0.05)
            if not future.done():
                self.metrics['ik_failed'] = self.metrics.get('ik_failed', 0) + 1
                return {"status": "error", "message": "IK timed out"}
            resp = future.result()
            if resp and resp.error_code.val == resp.error_code.SUCCESS:
                self.metrics['ik_success'] = self.metrics.get('ik_success', 0) + 1
                joints = {n: p for n, p in zip(resp.solution.joint_state.name, resp.solution.joint_state.position)}
                return {"status": "success", "joints": joints}
            else:
                self.metrics['ik_failed'] = self.metrics.get('ik_failed', 0) + 1
                code = resp.error_code.val if resp else None
                return {"status": "error", "message": f"IK failed (code {code})"}
        except Exception as e:
            self.metrics['ik_failed'] = self.metrics.get('ik_failed', 0) + 1
            return {"status": "error", "message": f"IK exception: {e}"}

    def _map_moveit_error(self, code: int) -> str:
        mapping = {
            1: "Success",
            -1: "Planning failed",
            -2: "Invalid motion plan",
            -3: "Plan invalidated by environment change",
            -4: "Control failed",
            -5: "Unable to acquire sensor data",
            -6: "Planning timed out or no valid goal states",
            -7: "Preempted",
            -10: "Start state in collision",
            -11: "Start state violates path constraints",
            -12: "Goal in collision",
            -13: "Goal violates path constraints",
            -14: "Joint limits violated",
        }
        return mapping.get(code, f"MoveIt error code {code}")
    # ========== MoveIt Planning/Execution ==========
    def plan_to_pose(self, target_pose: Dict[str, Any], group_name: str = 'ur_manipulator', ee_link: str = 'tool0', frame_id: str = 'base_link', vel_scale: float = 0.2, acc_scale: float = 0.2) -> Dict[str, Any]:
        """Call MoveIt GetMotionPlan service to plan to a pose target.

        target_pose: {'position':{'x','y','z'}, 'orientation':{'x','y','z','w'}}
        """
        if not self._moveit_client or not GetMotionPlan:
            return {"status": "error", "message": "MoveIt planning service not available"}
        if not self._moveit_client.wait_for_service(timeout_sec=2.0):
            return {"status": "error", "message": "MoveIt planning service not responding"}
        try:
            req = GetMotionPlan.Request()
            req.motion_plan_request = MotionPlanRequest()
            mpr = req.motion_plan_request
            mpr.group_name = group_name
            mpr.max_velocity_scaling_factor = float(vel_scale)
            mpr.max_acceleration_scaling_factor = float(acc_scale)

            constraints = Constraints()
            # Position constraint via a small sphere at goal
            pc = PositionConstraint()
            pc.header.frame_id = frame_id
            pc.link_name = ee_link
            from shape_msgs.msg import SolidPrimitive
            sphere = SolidPrimitive()
            sphere.type = SolidPrimitive.SPHERE
            sphere.dimensions = [0.001]
            from geometry_msgs.msg import Pose
            pose = Pose()
            pose.position.x = float(target_pose['position']['x'])
            pose.position.y = float(target_pose['position']['y'])
            pose.position.z = float(target_pose['position']['z'])
            pose.orientation.x = float(target_pose['orientation'].get('x', 0.0))
            pose.orientation.y = float(target_pose['orientation'].get('y', 0.0))
            pose.orientation.z = float(target_pose['orientation'].get('z', 0.0))
            pose.orientation.w = float(target_pose['orientation'].get('w', 1.0))
            pc.constraint_region.primitives.append(sphere)
            pc.constraint_region.primitive_poses.append(pose)
            constraints.position_constraints.append(pc)

            oc = OrientationConstraint()
            oc.header.frame_id = frame_id
            oc.link_name = ee_link
            oc.orientation = pose.orientation
            oc.absolute_x_axis_tolerance = 0.01
            oc.absolute_y_axis_tolerance = 0.01
            oc.absolute_z_axis_tolerance = 0.01
            oc.weight = 1.0
            constraints.orientation_constraints.append(oc)

            mpr.goal_constraints.append(constraints)

            future = self._moveit_client.call_async(req)
            from rclpy.task import Future
            start = time.time()
            while rclpy.ok() and not future.done() and (time.time() - start) < 5.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            if not future.done():
                return {"status": "error", "message": "MoveIt planning timed out"}
            resp = future.result()
            if resp and resp.motion_plan_response and resp.motion_plan_response.error_code.val == resp.motion_plan_response.error_code.SUCCESS:
                traj = resp.motion_plan_response.trajectory.joint_trajectory
                # Execute by publishing trajectory to controller
                if self._traj_pub:
                    self._traj_pub.publish(traj)
                    return {"status": "success", "message": "Plan executed", "points": len(traj.points)}
                else:
                    return {"status": "success", "message": "Plan computed (no traj publisher)", "points": len(traj.points)}
            else:
                code = None
                try:
                    code = resp.motion_plan_response.error_code.val
                except Exception:
                    pass
                return {"status": "error", "message": f"MoveIt planning failed (code {code})"}
        except Exception as e:
            logger.error(f"MoveIt planning error: {e}")
            return {"status": "error", "message": str(e)}

    def publish_forward_position(self, positions: list) -> Dict[str, Any]:
        """Publish Float64MultiArray to forward position controller (joint order determined by controller)."""
        if not self._forward_position_pub:
            return {"status": "error", "message": "Forward position controller not configured"}
        try:
            arr = Float64MultiArray()
            arr.data = [float(x) for x in positions]
            self._forward_position_pub.publish(arr)
            return {"status": "success", "message": f"Sent forward position command ({len(arr.data)} values)"}
        except Exception as e:
            logger.error(f"Error publishing forward position: {e}")
            return {"status": "error", "message": str(e)}

    def publish_servo_delta_joint(self, joint_names: list, deltas: list, duration: float = 1.0, rate_hz: float = 50.0) -> Dict[str, Any]:
        """Publish JointJog messages for a duration."""
        if not self._servo_delta_joint_pub or not JointJog:
            return {"status": "error", "message": "Servo delta_joint publisher not configured"}
        try:
            period = 1.0 / max(1.0, rate_hz)
            end_time = time.time() + max(0.0, duration)
            while time.time() < end_time:
                j = JointJog()
                j.header.stamp = self.get_clock().now().to_msg()
                j.joint_names = list(joint_names)
                j.displacements = [float(x) for x in deltas]
                # JointJog.duration is a float64 in ROS 2 (seconds), not a Duration message
                j.duration = float(period)
                self._servo_delta_joint_pub.publish(j)
                time.sleep(period)
            return {"status": "success", "message": "Sent servo delta joint commands"}
        except Exception as e:
            logger.error(f"Error publishing servo delta joint: {e}")
            return {"status": "error", "message": str(e)}

    def publish_servo_delta_twist(self, linear: list, angular: list, frame_id: str = "base_link", duration: float = 1.0, rate_hz: float = 50.0) -> Dict[str, Any]:
        """Publish TwistStamped messages for a duration (MoveIt Servo)."""
        if not self._servo_delta_twist_pub:
            return {"status": "error", "message": "Servo delta_twist publisher not configured"}
        try:
            period = 1.0 / max(1.0, rate_hz)
            end_time = time.time() + max(0.0, duration)
            while time.time() < end_time:
                tw = TwistStamped()
                tw.header.stamp = self.get_clock().now().to_msg()
                tw.header.frame_id = frame_id
                tw.twist.linear.x = float(linear[0] if len(linear) > 0 else 0.0)
                tw.twist.linear.y = float(linear[1] if len(linear) > 1 else 0.0)
                tw.twist.linear.z = float(linear[2] if len(linear) > 2 else 0.0)
                tw.twist.angular.x = float(angular[0] if len(angular) > 0 else 0.0)
                tw.twist.angular.y = float(angular[1] if len(angular) > 1 else 0.0)
                tw.twist.angular.z = float(angular[2] if len(angular) > 2 else 0.0)
                self._servo_delta_twist_pub.publish(tw)
                time.sleep(period)
            return {"status": "success", "message": "Sent servo delta twist commands"}
        except Exception as e:
            logger.error(f"Error publishing servo delta twist: {e}")
            return {"status": "error", "message": str(e)}
    
    def get_status(self) -> Dict[str, Any]:
        """Get current robot status"""
        return {
            "current_pose": self.current_pose.to_dict(),
            "total_distance_traveled": round(self.total_distance, 3),
            "total_rotation_deg": round(math.degrees(self.total_rotation), 1),
            "number_of_moves": len(self.pose_history) - 1,
            "path_length": len(self.pose_history),
            "metrics": self.metrics
        }
    
    def read_lidar_scan(self) -> Dict[str, Any]:
        """Read latest LIDAR scan data"""
        if self.latest_scan is None:
            return {"status": "error", "message": "No LIDAR data available"}
        
        return {
            "status": "success",
            "data": self.latest_scan,
            "message": f"LIDAR scan with {len(self.latest_scan['ranges'])} points"
        }

    # ========== Perception / Localization Tools ==========
    def is_obstacle_ahead(self, threshold_m: float = 0.5, fov_deg: float = 60.0) -> Dict[str, Any]:
        """Check whether an obstacle exists within the forward FOV and distance threshold."""
        scan = self.latest_scan
        now = time.time()
        if not scan or (now - scan.get('timestamp', 0)) > 0.5:
            return {"status": "error", "message": "LIDAR data not fresh (<0.5s)", "fresh": False}
        ranges = scan['ranges']
        angle_min = scan['angle_min']
        angle_inc = scan['angle_increment']
        fov_rad = math.radians(max(0.0, min(180.0, fov_deg)))
        half = fov_rad / 2.0
        hits = []
        for i, r in enumerate(ranges):
            ang = angle_min + i * angle_inc
            if -half <= ang <= half and r > 0.0:
                hits.append((r, ang))
        if not hits:
            return {"status": "success", "obstacle": False, "fresh": True, "message": "No obstacle ahead in FOV"}
        rmin, angmin = min(hits, key=lambda x: x[0])
        if rmin < threshold_m:
            msg = (
                f"Obstacle within {threshold_m:.2f} m ahead at {rmin:.2f} m, "
                f"bearing {math.degrees(angmin):.1f}° (FOV {fov_deg:.0f}°)"
            )
        else:
            msg = (
                f"No obstacle within {threshold_m:.2f} m ahead; nearest return "
                f"{rmin:.2f} m at {math.degrees(angmin):.1f}° (FOV {fov_deg:.0f}°)"
            )
        return {
            "status": "success",
            "obstacle": bool(rmin < threshold_m),
            "min_distance_m": float(rmin),
            "bearing_rad": float(angmin),
            "bearing_deg": float(math.degrees(angmin)),
            "threshold_m": float(threshold_m),
            "fov_deg": float(fov_deg),
            "fresh": True,
            "message": msg
        }

    def obstacle_report(self, bins: int = 12) -> Dict[str, Any]:
        """Sectorize 360° into N bins and report min range per bin and closest obstacle."""
        scan = self.latest_scan
        now = time.time()
        if not scan or (now - scan.get('timestamp', 0)) > 0.5:
            return {"status": "error", "message": "LIDAR data not fresh (<0.5s)", "fresh": False}
        ranges = scan['ranges']
        angle_min = scan['angle_min']
        angle_max = scan['angle_max']
        angle_inc = scan['angle_increment']
        span = angle_max - angle_min
        bins = max(4, int(bins))
        bin_size = span / bins
        mins = [float('inf')] * bins
        for i, r in enumerate(ranges):
            if r <= 0.0:
                continue
            ang = angle_min + i * angle_inc
            b = int((ang - angle_min) / bin_size)
            if 0 <= b < bins:
                mins[b] = min(mins[b], r)
        closest_dist = min(mins) if any(m != float('inf') for m in mins) else None
        closest_bin = mins.index(closest_dist) if closest_dist is not None else None
        if closest_dist is not None:
            msg = f"Closest obstacle {closest_dist:.2f} m (bin {closest_bin}/{bins})"
        else:
            msg = "No obstacles detected in any sector"
        return {
            "status": "success",
            "bins": bins,
            "min_ranges": [None if m == float('inf') else float(m) for m in mins],
            "closest": {"bin": closest_bin, "distance_m": float(closest_dist) if closest_dist is not None else None},
            "fresh": True,
            "message": msg
        }

    def free_direction(self, fov_deg: float = 30.0, min_clear_m: float = 1.0) -> Dict[str, Any]:
        """Return a forward-facing yaw sector that appears clear per LIDAR."""
        scan = self.latest_scan
        now = time.time()
        if not scan or (now - scan.get('timestamp', 0)) > 0.5:
            return {"status": "error", "message": "LIDAR data not fresh (<0.5s)", "fresh": False}
        ranges = scan['ranges']
        angle_min = scan['angle_min']
        angle_inc = scan['angle_increment']
        fov_rad = math.radians(max(0.0, min(180.0, fov_deg)))
        half = fov_rad / 2.0
        best = None
        for center in [0.0, math.radians(15), math.radians(-15), math.radians(30), math.radians(-30)]:
            window = []
            for i, r in enumerate(ranges):
                ang = angle_min + i * angle_inc
                if (center - half) <= ang <= (center + half) and r > 0.0:
                    window.append(r)
            if window and min(window) >= min_clear_m:
                best = center
                break
        if best is None:
            return {"status": "success", "clear": False, "message": "No clear sector found", "fresh": True}
        bearing_deg = math.degrees(best)
        msg = f"Clear sector centered at {bearing_deg:.1f}° (FOV {fov_deg:.0f}°, clearance ≥ {min_clear_m:.2f} m)"
        return {"status": "success", "clear": True, "center_bearing_rad": float(best), "center_bearing_deg": float(bearing_deg), "fov_deg": float(fov_deg), "fresh": True, "message": msg}

    def get_pose_tool(self, frame: str = 'map') -> Dict[str, Any]:
        """Resolve robot pose in 'map' (preferred) or 'odom' frame via TF; fallback to amcl_pose."""
        preferred = frame if frame in ('map', 'odom') else 'map'
        if self.tf_buffer:
            for fr in [preferred, 'map' if preferred != 'map' else 'odom']:
                try:
                    transform = self.tf_buffer.lookup_transform(fr, 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
                    pos = transform.transform.translation
                    ori = transform.transform.rotation
                    yaw = self._quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
                    return {"status": "success", "frame": fr, "x": pos.x, "y": pos.y, "theta_rad": yaw}
                except Exception:
                    continue
        if self.latest_amcl and (time.time() - self.latest_amcl.get('timestamp', 0)) < 1.0:
            x, y, z = self.latest_amcl.get('position', [0,0,0])
            ox, oy, oz, ow = self.latest_amcl.get('orientation', [0,0,0,1])
            yaw = self._quaternion_to_yaw(ox, oy, oz, ow)
            return {"status": "success", "frame": self.latest_amcl.get('frame_id') or 'map', "x": x, "y": y, "theta_rad": yaw, "covariance": self.latest_amcl.get('covariance')}
        return {"status": "error", "message": "Pose not available (no TF and no fresh AMCL)"}

    def where_am_i(self) -> Dict[str, Any]:
        """Natural language summary of current pose."""
        pose = self.get_pose_tool('map')
        if pose.get('status') != 'success':
            pose = self.get_pose_tool('odom')
            if pose.get('status') != 'success':
                return pose
        theta_deg = math.degrees(pose['theta_rad'])
        return {"status": "success", "message": f"At ({pose['x']:.2f}, {pose['y']:.2f}) in {pose['frame']} frame, heading {theta_deg:.1f}°"}

    def describe_environment(self) -> Dict[str, Any]:
        """Combine obstacle report and localization into a concise description."""
        rep = self.obstacle_report()
        pos = self.where_am_i()
        parts = []
        if rep.get('status') == 'success':
            mins = rep.get('min_ranges') or []
            closest = rep.get('closest', {})
            if closest.get('distance_m') is not None:
                parts.append(f"closest obstacle {closest['distance_m']:.2f}m in bin {closest.get('bin')}")
            free_bins = [i for i, m in enumerate(mins) if (m is None)]
            if free_bins:
                parts.append(f"free sectors: {free_bins}")
        else:
            parts.append("lidar not fresh")
        if pos.get('status') == 'success':
            parts.append(pos.get('message'))
        else:
            parts.append("pose unavailable")
        return {"status": "success", "message": "; ".join(parts), "obstacles": rep, "pose": pos}

    # ========== TF utilities ==========
    def transform_pose(self, pose: Dict[str, Any], source_frame: str, target_frame: str) -> Dict[str, Any]:
        """Transform a pose dict from source_frame to target_frame using TF2.

        pose must contain either {'x','y','z','qx','qy','qz','qw'} or
        {'position':[x,y,z], 'orientation_xyzw':[x,y,z,w]}.
        """
        if not self.tf_buffer:
            return {"status": "error", "message": "TF2 transform not available"}
        try:
            ps = PoseStamped()
            ps.header.frame_id = source_frame
            if 'position' in pose and 'orientation_xyzw' in pose:
                ps.pose.position.x = float(pose['position'][0])
                ps.pose.position.y = float(pose['position'][1])
                ps.pose.position.z = float(pose['position'][2])
                ps.pose.orientation.x = float(pose['orientation_xyzw'][0])
                ps.pose.orientation.y = float(pose['orientation_xyzw'][1])
                ps.pose.orientation.z = float(pose['orientation_xyzw'][2])
                ps.pose.orientation.w = float(pose['orientation_xyzw'][3])
            else:
                ps.pose.position.x = float(pose.get('x', 0.0))
                ps.pose.position.y = float(pose.get('y', 0.0))
                ps.pose.position.z = float(pose.get('z', 0.0))
                ps.pose.orientation.x = float(pose.get('qx', 0.0))
                ps.pose.orientation.y = float(pose.get('qy', 0.0))
                ps.pose.orientation.z = float(pose.get('qz', 0.0))
                ps.pose.orientation.w = float(pose.get('qw', 1.0))

            # Lookup transform target <- source
            tf = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            # Prefer pose-stamped transform helper if available
            if do_transform_pose_stamped:
                out = do_transform_pose_stamped(ps, tf)
                pose_out = out.pose
            elif do_transform_pose:
                # Transform Pose only, using helper, then reuse orientation/position
                out_pose = do_transform_pose(ps.pose, tf)
                pose_out = out_pose
            else:
                # Fallback: use Buffer.transform for PoseStamped if supported
                out = self.tf_buffer.transform(ps, target_frame, timeout=rclpy.duration.Duration(seconds=1.0))
                pose_out = out.pose
            return {
                "status": "success",
                "x": pose_out.position.x,
                "y": pose_out.position.y,
                "z": pose_out.position.z,
                "qx": pose_out.orientation.x,
                "qy": pose_out.orientation.y,
                "qz": pose_out.orientation.z,
                "qw": pose_out.orientation.w,
                "frame": target_frame
            }
        except Exception as e:
            return {"status": "error", "message": f"TF transform failed: {e}"}
    
    def read_imu_data(self) -> Dict[str, Any]:
        """Read latest IMU data"""
        if self.latest_imu is None:
            return {"status": "error", "message": "No IMU data available"}
        
        return {
            "status": "success",
            "data": self.latest_imu,
            "message": "IMU data retrieved"
        }
    
    def get_path_visualization(self) -> str:
        """Generate path visualization"""
        if len(self.pose_history) < 2:
            return "No movement yet - robot at origin (0, 0)"
        
        # Find bounds
        min_x = min(pose.x for pose in self.pose_history)
        max_x = max(pose.x for pose in self.pose_history)
        min_y = min(pose.y for pose in self.pose_history)
        max_y = max(pose.y for pose in self.pose_history)
        
        # Create simple text representation
        path_info = []
        path_info.append(f"Robot Path (Total distance: {self.total_distance:.2f}m)")
        path_info.append("-" * 40)
        
        for i, pose in enumerate(self.pose_history):
            theta_deg = math.degrees(pose.theta)
            if i == 0:
                path_info.append(f"START: ({pose.x:.2f}, {pose.y:.2f}) facing {theta_deg:.1f}°")
            elif i == len(self.pose_history) - 1:
                path_info.append(f"END:   ({pose.x:.2f}, {pose.y:.2f}) facing {theta_deg:.1f}°")
            else:
                path_info.append(f"  {i:2d}:  ({pose.x:.2f}, {pose.y:.2f}) facing {theta_deg:.1f}°")
        
        # Add bounds info
        path_info.append("-" * 40)
        path_info.append(f"Bounds: X[{min_x:.2f}, {max_x:.2f}] Y[{min_y:.2f}, {max_y:.2f}]")
        
        return "\n".join(path_info)
    
    def reset_position(self, new_pose: RobotPose = None):
        """Reset robot position tracking"""
        old_pose = self.current_pose
        
        # Reset tracking variables
        self.current_pose = new_pose or RobotPose()
        self.pose_history = [self.current_pose]
        self.total_distance = 0.0
        self.total_rotation = 0.0
        
        # Optionally publish initial pose to ROS (for localization reset)
        if new_pose is None or (new_pose.x == 0 and new_pose.y == 0 and new_pose.theta == 0):
            initial_pose_msg = PoseWithCovarianceStamped()
            initial_pose_msg.header.frame_id = "map"
            initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
            initial_pose_msg.pose.pose.position.x = 0.0
            initial_pose_msg.pose.pose.position.y = 0.0
            initial_pose_msg.pose.pose.position.z = 0.0
            initial_pose_msg.pose.pose.orientation.w = 1.0
            
            self.initial_pose_pub.publish(initial_pose_msg)
        
        logger.info(f"Robot position reset: {old_pose.to_dict()} -> {self.current_pose.to_dict()}")
        return {
            "status": "success",
            "message": "Robot position reset",
            "old_pose": old_pose.to_dict(),
            "new_pose": self.current_pose.to_dict()
        }
    
    def plan_and_execute_pose_goal(self, target_pose: Dict[str, float], group_name: str = "ur_manipulator", 
                                   link_name: str = "tool0", frame_id: str = "base_link", 
                                   max_velocity_scaling: float = 0.2, max_acceleration_scaling: float = 0.2,
                                   position_tolerance_m: float = 0.03, orientation_tolerance_rad: float = 0.1745,
                                   allow_position_only: bool = True,
                                   precheck_ik: bool = True) -> Dict[str, Any]:
        """
        Plan and execute motion to a target pose using MoveIt action server.
        
        Args:
            target_pose: Dict with keys x, y, z, qx, qy, qz, qw (position and orientation)
            group_name: MoveIt planning group name (e.g., "ur_manipulator")
            link_name: End-effector link name (e.g., "tool0")
            frame_id: Reference frame (e.g., "base_link")
            max_velocity_scaling: Velocity scaling factor (0.0-1.0)
            max_acceleration_scaling: Acceleration scaling factor (0.0-1.0)
        
        Returns:
            Dict with status, message, and execution result
        """
        if not self._moveit_client:
            return {"status": "error", "message": "MoveIt action client not initialized"}
        
        if not MoveGroup or not SolidPrimitive:
            return {"status": "error", "message": "MoveIt action interfaces not available"}
        
        try:
            # Wait for MoveIt action server
            self.metrics['plans_requested'] = self.metrics.get('plans_requested', 0) + 1
            if not self._moveit_client.wait_for_server(timeout_sec=5.0):
                return {"status": "error", "message": "MoveIt action server not available"}
            
            # Build target pose message
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = frame_id
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = float(target_pose.get('x', 0.0))
            pose_stamped.pose.position.y = float(target_pose.get('y', 0.0))
            pose_stamped.pose.position.z = float(target_pose.get('z', 0.0))
            pose_stamped.pose.orientation.x = float(target_pose.get('qx', 0.0))
            pose_stamped.pose.orientation.y = float(target_pose.get('qy', 0.0))
            pose_stamped.pose.orientation.z = float(target_pose.get('qz', 0.0))
            pose_stamped.pose.orientation.w = float(target_pose.get('qw', 1.0))
            
            # IK pre-validation with adaptive strategies (optional)
            if precheck_ik:
                ik_result = self._compute_ik(target_pose, group_name, link_name, frame_id, timeout_sec=0.5)
                if ik_result.get('status') != 'success' and allow_position_only:
                    pos_only = {**target_pose, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}
                    ik_result = self._compute_ik(pos_only, group_name, link_name, frame_id, timeout_sec=0.5)
                    if ik_result.get('status') == 'success':
                        target_pose = pos_only
                if ik_result.get('status') != 'success':
                    import random
                    for _ in range(5):
                        dx = (random.random() - 0.5) * 2 * position_tolerance_m
                        dy = (random.random() - 0.5) * 2 * position_tolerance_m
                        dz = (random.random() - 0.5) * 2 * position_tolerance_m
                        nudged = {**target_pose, 'x': target_pose.get('x',0.0)+dx, 'y': target_pose.get('y',0.0)+dy, 'z': target_pose.get('z',0.0)+dz}
                        ik_try = self._compute_ik(nudged, group_name, link_name, frame_id, timeout_sec=0.3)
                        if ik_try.get('status') == 'success':
                            target_pose = nudged
                            ik_result = ik_try
                            break

            # Create constraints (relaxed by default)
            constraints = Constraints()
            
            # Position constraint - small sphere around target
            pos_constraint = PositionConstraint()
            pos_constraint.header.frame_id = frame_id
            pos_constraint.link_name = link_name
            pos_constraint.target_point_offset.x = 0.0
            pos_constraint.target_point_offset.y = 0.0
            pos_constraint.target_point_offset.z = 0.0
            
            # Define constraint region as sphere (relaxed)
            sphere = SolidPrimitive()
            sphere.type = SolidPrimitive.SPHERE
            sphere.dimensions = [max(0.001, float(position_tolerance_m))]
            pos_constraint.constraint_region.primitives.append(sphere)
            pos_constraint.constraint_region.primitive_poses.append(pose_stamped.pose)
            constraints.position_constraints.append(pos_constraint)
            
            # Orientation constraint (optional)
            if not allow_position_only:
                orient_constraint = OrientationConstraint()
                orient_constraint.header.frame_id = frame_id
                orient_constraint.link_name = link_name
                orient_constraint.orientation = pose_stamped.pose.orientation
                tol = max(0.01, float(orientation_tolerance_rad))
                orient_constraint.absolute_x_axis_tolerance = tol
                orient_constraint.absolute_y_axis_tolerance = tol
                orient_constraint.absolute_z_axis_tolerance = tol
                orient_constraint.weight = 0.5
                constraints.orientation_constraints.append(orient_constraint)
            
            # Build MoveIt goal
            goal_msg = MoveGroup.Goal()
            goal_msg.request.group_name = group_name
            goal_msg.request.goal_constraints.append(constraints)
            goal_msg.request.max_velocity_scaling_factor = max_velocity_scaling
            goal_msg.request.max_acceleration_scaling_factor = max_acceleration_scaling
            goal_msg.request.allowed_planning_time = 10.0  # seconds
            
            logger.info(f"Sending MoveIt goal: {target_pose} to {group_name}/{link_name}")
            
            # Send goal asynchronously
            send_goal_future = self._moveit_client.send_goal_async(goal_msg)
            
            # Wait for goal to be accepted (with timeout)
            timeout_start = time.time()
            timeout_duration = 10.0  # seconds
            
            while not send_goal_future.done() and (time.time() - timeout_start) < timeout_duration:
                time.sleep(0.1)
            
            if not send_goal_future.done():
                return {"status": "error", "message": "Goal submission timeout"}
            
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                return {"status": "error", "message": "MoveIt goal rejected"}
            
            logger.info("MoveIt goal accepted, waiting for execution...")
            
            # Wait for result
            get_result_future = goal_handle.get_result_async()
            timeout_start = time.time()
            timeout_duration = 30.0  # seconds for planning + execution
            
            while not get_result_future.done() and (time.time() - timeout_start) < timeout_duration:
                time.sleep(0.1)
            
            if not get_result_future.done():
                return {"status": "error", "message": "Motion execution timeout"}
            
            result = get_result_future.result().result
            
            if result.error_code.val == result.error_code.SUCCESS:
                logger.info("MoveIt motion executed successfully!")
                self.metrics['plans_succeeded'] = self.metrics.get('plans_succeeded', 0) + 1
                return {
                    "status": "success", 
                    "message": f"Motion to pose {target_pose} executed successfully",
                    "target_pose": target_pose,
                    "group_name": group_name,
                    "link_name": link_name
                }
            else:
                code = result.error_code.val
                reason = self._map_moveit_error(code)
                self.metrics['plans_failed'] = self.metrics.get('plans_failed', 0) + 1
                self.metrics['last_error'] = reason
                logger.error(f"MoveIt planning/execution failed ({reason})")
                if self._traj_pub and isinstance(ik_result, dict) and ik_result.get('status') == 'success':
                    joints = ik_result.get('joints', {})
                    names = list(joints.keys())
                    positions = [joints[n] for n in names]
                    pub_res = self.publish_joint_trajectory(names, positions, duration=3.0)
                    if pub_res.get('status') == 'success':
                        return {"status": "success", "message": f"Executed IK solution directly: {pub_res.get('message')}"}
                return {"status": "error", "message": f"MoveIt planning/execution failed ({reason})"}
                
        except Exception as e:
            error_msg = f"Error in MoveIt planning: {e}"
            logger.error(error_msg)
            return {"status": "error", "message": error_msg}
    
    def get_end_effector_pose(self, link_name: str = "tool0", frame_id: str = "base_link") -> Dict[str, Any]:
        """
        Get current end-effector pose using TF transforms.
        
        Args:
            link_name: End-effector link name (e.g., "tool0")
            frame_id: Reference frame (e.g., "base_link")
        
        Returns:
            Dict with status, message, and pose data (position, orientation)
        """
        if not self.tf_buffer:
            return {"status": "error", "message": "TF2 not available"}
        
        try:
            # Look up transform from base_link to end-effector
            transform = self.tf_buffer.lookup_transform(
                frame_id, link_name, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Extract position and orientation
            pos = transform.transform.translation
            ori = transform.transform.rotation
            
            pose = {
                "position": {"x": pos.x, "y": pos.y, "z": pos.z},
                "orientation": {"x": ori.x, "y": ori.y, "z": ori.z, "w": ori.w}
            }
            
            return {
                "status": "success",
                "message": f"Current pose of {link_name} in {frame_id}",
                "pose": pose,
                "link_name": link_name,
                "frame_id": frame_id
            }
            
        except Exception as e:
            error_msg = f"Failed to get pose for {link_name}: {e}"
            logger.error(error_msg)
            return {"status": "error", "message": error_msg}
    
    def move_relative_to_current_pose(self, offset: Dict[str, float], link_name: str = "tool0", 
                                      frame_id: str = "base_link", **kwargs) -> Dict[str, Any]:
        """
        Move end-effector relative to its current pose.
        
        Args:
            offset: Dict with x, y, z offsets in meters
            link_name: End-effector link name
            frame_id: Reference frame
            **kwargs: Additional parameters for MoveIt planning
        
        Returns:
            Dict with status, message, and execution result
        """
        # Get current pose
        current_result = self.get_end_effector_pose(link_name, frame_id)
        if current_result.get("status") != "success":
            return current_result
        
        current_pose = current_result["pose"]
        
        # Calculate target pose by adding offsets
        target_pose = {
            "x": current_pose["position"]["x"] + offset.get("x", 0.0),
            "y": current_pose["position"]["y"] + offset.get("y", 0.0),
            "z": current_pose["position"]["z"] + offset.get("z", 0.0),
            "qx": current_pose["orientation"]["x"],
            "qy": current_pose["orientation"]["y"], 
            "qz": current_pose["orientation"]["z"],
            "qw": current_pose["orientation"]["w"]
        }
        
        logger.info(f"Moving {link_name} by offset {offset} from current pose")
        
        # Execute the motion
        return self.plan_and_execute_pose_goal(
            target_pose, 
            kwargs.get("group_name", "ur_manipulator"),
            link_name, 
            frame_id,
            kwargs.get("max_velocity_scaling", 0.2),
            kwargs.get("max_acceleration_scaling", 0.2)
        )


class ROSBridgeManager:
    """Manages ROS Bridge in a separate thread"""
    
    def __init__(self, robot_namespace: str = ""):
        self.robot_namespace = robot_namespace
        self.ros_bridge = None
        self.ros_thread = None
        self.executor = None
        self._running = False
    
    def start(self):
        """Start ROS bridge in separate thread"""
        if self._running:
            return
        
        def ros_thread_func():
            try:
                rclpy.init()
                self.ros_bridge = ROSBridge(self.robot_namespace)
                
                # Use SingleThreadedExecutor for better control
                from rclpy.executors import SingleThreadedExecutor
                self.executor = SingleThreadedExecutor()
                self.executor.add_node(self.ros_bridge)
                
                logger.info("ROS Bridge started")
                self._running = True
                
                # Spin until shutdown
                while self._running and rclpy.ok():
                    self.executor.spin_once(timeout_sec=0.1)
                    
            except Exception as e:
                logger.error(f"Error in ROS thread: {e}")
            finally:
                if self.ros_bridge:
                    self.ros_bridge.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
        
        self.ros_thread = threading.Thread(target=ros_thread_func, daemon=True)
        self.ros_thread.start()
        
        # Wait for bridge to initialize
        timeout = 10  # seconds
        start_time = time.time()
        while self.ros_bridge is None and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        
        if self.ros_bridge is None:
            raise RuntimeError("Failed to initialize ROS bridge within timeout")
        
        logger.info("ROS Bridge Manager started successfully")
    
    def stop(self):
        """Stop ROS bridge"""
        self._running = False
        if self.ros_thread and self.ros_thread.is_alive():
            self.ros_thread.join(timeout=5)
        logger.info("ROS Bridge Manager stopped")
    
    def get_bridge(self) -> Optional[ROSBridge]:
        """Get the ROS bridge instance"""
        return self.ros_bridge


# Example usage
def main():
    """Test the ROS bridge"""
    import sys
    
    # Initialize ROS bridge
    manager = ROSBridgeManager()
    
    try:
        manager.start()
        bridge = manager.get_bridge()
        
        if bridge is None:
            print("Failed to initialize ROS bridge")
            sys.exit(1)
        
        print("ROS Bridge initialized successfully!")
        print("Current pose:", bridge.current_pose.to_dict())
        
        # Test movement commands
        print("\nTesting drive_straight...")
        result = bridge.drive_straight(1.0, 0.2)
        print("Result:", result)
        
        print("\nTesting rotate_in_place...")
        result = bridge.rotate_in_place(math.pi/2, 0.5)
        print("Result:", result)
        
        print("\nFinal pose:", bridge.current_pose.to_dict())
        
        # Keep running to receive messages
        print("Press Ctrl+C to stop...")
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        manager.stop()


if __name__ == "__main__":
    main()





