#!/usr/bin/env python3
"""
Robot State Tracker - Simulates robot position and orientation for visualization
"""

import math
import logging
from typing import Dict, Any, Tuple, List
from dataclasses import dataclass
import json
import time

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
    
    def distance_to(self, other: 'RobotPose') -> float:
        """Calculate distance to another pose"""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

class RobotStateTracker:
    """Tracks robot state including position, orientation, and movement history"""
    
    def __init__(self, initial_pose: RobotPose = None):
        self.current_pose = initial_pose or RobotPose()
        self.pose_history: List[RobotPose] = [self.current_pose]
        self.total_distance = 0.0
        self.total_rotation = 0.0
        
        logger.info(f"Robot state tracker initialized at {self.current_pose.to_dict()}")
    
    def drive_straight(self, distance_m: float, speed_mps: float = 0.5) -> Dict[str, Any]:
        """Simulate driving straight forward/backward"""
        if distance_m == 0:
            return {"status": "success", "message": "No movement requested", "pose": self.current_pose.to_dict()}
        
        # Calculate new position
        new_x = self.current_pose.x + distance_m * math.cos(self.current_pose.theta)
        new_y = self.current_pose.y + distance_m * math.sin(self.current_pose.theta)
        
        # Update pose
        old_pose = self.current_pose
        self.current_pose = RobotPose(new_x, new_y, self.current_pose.theta)
        self.pose_history.append(self.current_pose)
        
        # Update statistics
        self.total_distance += abs(distance_m)
        
        result = {
            "status": "success",
            "action": "drive_straight",
            "parameters": {"distance_m": distance_m, "speed_mps": speed_mps},
            "old_pose": old_pose.to_dict(),
            "new_pose": self.current_pose.to_dict(),
            "distance_moved": abs(distance_m),
            "total_distance": round(self.total_distance, 3)
        }
        
        logger.info(f"Drove straight {distance_m}m: {old_pose.to_dict()} -> {self.current_pose.to_dict()}")
        return result
    
    def rotate_in_place(self, yaw_rad: float, yaw_rate_rps: float = 0.5) -> Dict[str, Any]:
        """Simulate rotating in place"""
        if yaw_rad == 0:
            return {"status": "success", "message": "No rotation requested", "pose": self.current_pose.to_dict()}
        
        # Update orientation
        old_pose = self.current_pose
        new_theta = self.current_pose.theta + yaw_rad
        
        # Normalize angle to [-pi, pi]
        new_theta = math.atan2(math.sin(new_theta), math.cos(new_theta))
        
        self.current_pose = RobotPose(self.current_pose.x, self.current_pose.y, new_theta)
        self.pose_history.append(self.current_pose)
        
        # Update statistics
        self.total_rotation += abs(yaw_rad)
        
        result = {
            "status": "success",
            "action": "rotate_in_place",
            "parameters": {"yaw_rad": yaw_rad, "yaw_rate_rps": yaw_rate_rps},
            "old_pose": old_pose.to_dict(),
            "new_pose": self.current_pose.to_dict(),
            "rotation_rad": yaw_rad,
            "rotation_deg": round(math.degrees(yaw_rad), 1),
            "total_rotation_deg": round(math.degrees(self.total_rotation), 1)
        }
        
        direction = "left" if yaw_rad > 0 else "right"
        logger.info(f"Rotated {direction} {abs(math.degrees(yaw_rad)):.1f}°: {old_pose.to_dict()} -> {self.current_pose.to_dict()}")
        return result
    
    def set_base_twist(self, vx: float, vy: float, wz: float, duration: float = 1.0) -> Dict[str, Any]:
        """Simulate velocity control for a duration"""
        if vx == 0 and vy == 0 and wz == 0:
            return {"status": "success", "message": "No velocity commanded", "pose": self.current_pose.to_dict()}
        
        old_pose = self.current_pose
        
        # Simple integration (assumes differential drive, so vy is ignored)
        if wz != 0:
            # Circular motion
            radius = vx / wz if wz != 0 else float('inf')
            theta_change = wz * duration
            
            # Update position and orientation
            new_x = self.current_pose.x + radius * (math.sin(self.current_pose.theta + theta_change) - math.sin(self.current_pose.theta))
            new_y = self.current_pose.y - radius * (math.cos(self.current_pose.theta + theta_change) - math.cos(self.current_pose.theta))
            new_theta = self.current_pose.theta + theta_change
        else:
            # Straight line motion
            distance = vx * duration
            new_x = self.current_pose.x + distance * math.cos(self.current_pose.theta)
            new_y = self.current_pose.y + distance * math.sin(self.current_pose.theta)
            new_theta = self.current_pose.theta
        
        # Normalize angle
        new_theta = math.atan2(math.sin(new_theta), math.cos(new_theta))
        
        self.current_pose = RobotPose(new_x, new_y, new_theta)
        self.pose_history.append(self.current_pose)
        
        # Update statistics
        distance_moved = old_pose.distance_to(self.current_pose)
        self.total_distance += distance_moved
        self.total_rotation += abs(wz * duration)
        
        result = {
            "status": "success",
            "action": "set_base_twist",
            "parameters": {"vx": vx, "vy": vy, "wz": wz, "duration": duration},
            "old_pose": old_pose.to_dict(),
            "new_pose": self.current_pose.to_dict(),
            "distance_moved": round(distance_moved, 3),
            "total_distance": round(self.total_distance, 3)
        }
        
        logger.info(f"Applied twist for {duration}s: {old_pose.to_dict()} -> {self.current_pose.to_dict()}")
        return result
    
    def stop(self) -> Dict[str, Any]:
        """Stop the robot (no state change)"""
        result = {
            "status": "success",
            "action": "stop",
            "message": "Robot stopped",
            "current_pose": self.current_pose.to_dict(),
            "total_distance": round(self.total_distance, 3),
            "total_rotation_deg": round(math.degrees(self.total_rotation), 1)
        }
        
        logger.info(f"Robot stopped at {self.current_pose.to_dict()}")
        return result
    
    def get_status(self) -> Dict[str, Any]:
        """Get current robot status"""
        return {
            "current_pose": self.current_pose.to_dict(),
            "total_distance_traveled": round(self.total_distance, 3),
            "total_rotation_deg": round(math.degrees(self.total_rotation), 1),
            "number_of_moves": len(self.pose_history) - 1,
            "path_length": len(self.pose_history)
        }
    
    def get_path_visualization(self) -> str:
        """Generate a simple ASCII visualization of the robot's path"""
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
        """Reset robot to initial position"""
        old_pose = self.current_pose
        self.current_pose = new_pose or RobotPose()
        self.pose_history = [self.current_pose]
        self.total_distance = 0.0
        self.total_rotation = 0.0
        
        logger.info(f"Robot position reset: {old_pose.to_dict()} -> {self.current_pose.to_dict()}")
        return {
            "status": "success",
            "message": "Robot position reset",
            "old_pose": old_pose.to_dict(),
            "new_pose": self.current_pose.to_dict()
        }

# Example usage and testing
def main():
    """Test the robot state tracker"""
    tracker = RobotStateTracker()
    
    print("Initial state:")
    print(json.dumps(tracker.get_status(), indent=2))
    
    # Simulate some movements
    print("\n1. Drive forward 2 meters:")
    result = tracker.drive_straight(2.0)
    print(json.dumps(result, indent=2))
    
    print("\n2. Turn left 90 degrees:")
    result = tracker.rotate_in_place(math.pi/2)
    print(json.dumps(result, indent=2))
    
    print("\n3. Drive forward 1 meter:")
    result = tracker.drive_straight(1.0)
    print(json.dumps(result, indent=2))
    
    print("\n4. Turn right 45 degrees:")
    result = tracker.rotate_in_place(-math.pi/4)
    print(json.dumps(result, indent=2))
    
    print("\nFinal status:")
    print(json.dumps(tracker.get_status(), indent=2))
    
    print("\nPath visualization:")
    print(tracker.get_path_visualization())

if __name__ == "__main__":
    main()
