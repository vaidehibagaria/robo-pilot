#!/usr/bin/env python3
"""
RCM Server - FastAPI service that serves Robot Capability Manifests
"""

import json
import logging
from pathlib import Path
from typing import Dict, Any, Optional

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RCMServer:
    def __init__(self):
        self.app = FastAPI(
            title="Robot Capability Manifest Server",
            description="Serves authoritative robot capability manifests",
            version="1.0.0"
        )
        
        # Enable CORS for web clients
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        
        # In-memory storage for RCMs
        self.rcms: Dict[str, Dict[str, Any]] = {}
        
        # Setup routes
        self._setup_routes()
    
    def _setup_routes(self):
        @self.app.get("/")
        async def root():
            return {"message": "RCM Server is running", "robots": list(self.rcms.keys())}
        
        @self.app.get("/robots")
        async def list_robots():
            """List all available robots"""
            return {"robots": list(self.rcms.keys())}
        
        @self.app.get("/robots/{robot_id}/rcm")
        async def get_rcm(robot_id: str):
            """Get complete RCM for a robot"""
            if robot_id not in self.rcms:
                raise HTTPException(status_code=404, detail=f"Robot {robot_id} not found")
            return self.rcms[robot_id]
        
        @self.app.get("/robots/{robot_id}/capabilities")
        async def get_capabilities(robot_id: str):
            """Get just the capabilities/primitives for a robot"""
            if robot_id not in self.rcms:
                raise HTTPException(status_code=404, detail=f"Robot {robot_id} not found")
            
            rcm = self.rcms[robot_id]
            return {
                "robot_id": robot_id,
                "primitives": rcm.get("primitives", []),
                "locomotion": rcm.get("locomotion", {}),
                "end_effectors": rcm.get("end_effectors", []),
                "has_gripper": rcm.get("has_gripper", False),
                "gripper_type": rcm.get("gripper_type", "none"),
                "joints": rcm.get("joints", {}),
                "sensors": rcm.get("sensors", [])
            }
        
        @self.app.get("/robots/{robot_id}/constraints")
        async def get_constraints(robot_id: str):
            """Get safety constraints for a robot"""
            if robot_id not in self.rcms:
                raise HTTPException(status_code=404, detail=f"Robot {robot_id} not found")
            
            rcm = self.rcms[robot_id]
            constraints = {}
            
            # Joint limits
            joints = rcm.get("joints", {})
            for joint_name, joint_data in joints.items():
                limits = joint_data.get("limits", {})
                if any(v is not None for v in limits.values()):
                    constraints[f"joint_{joint_name}"] = limits
            
            # Dynamics constraints
            dynamics = rcm.get("dynamics", {})
            if dynamics.get("mass_kg"):
                constraints["payload_capacity_kg"] = max(0, dynamics["mass_kg"] * 0.1)  # 10% of robot mass
            
            # Safety profile
            safety = rcm.get("safety_profile", {})
            constraints["safety"] = safety
            
            return {"robot_id": robot_id, "constraints": constraints}
        
        @self.app.post("/robots/{robot_id}/rcm")
        async def upload_rcm(robot_id: str, rcm: Dict[str, Any]):
            """Upload/update RCM for a robot"""
            self.rcms[robot_id] = rcm
            logger.info(f"Updated RCM for robot: {robot_id}")
            return {"message": f"RCM updated for robot {robot_id}"}
        
        @self.app.delete("/robots/{robot_id}")
        async def delete_robot(robot_id: str):
            """Remove a robot's RCM"""
            if robot_id not in self.rcms:
                raise HTTPException(status_code=404, detail=f"Robot {robot_id} not found")
            
            del self.rcms[robot_id]
            return {"message": f"Robot {robot_id} removed"}
    
    def load_rcm_from_file(self, robot_id: str, rcm_path: str):
        """Load RCM from JSON file"""
        try:
            with open(rcm_path, 'r') as f:
                rcm = json.load(f)
            self.rcms[robot_id] = rcm
            logger.info(f"Loaded RCM for robot {robot_id} from {rcm_path}")
        except Exception as e:
            logger.error(f"Failed to load RCM from {rcm_path}: {e}")
            raise
    
    def run(self, host: str = "0.0.0.0", port: int = 8000):
        """Run the server"""
        logger.info(f"Starting RCM server on {host}:{port}")
        uvicorn.run(self.app, host=host, port=port)

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="RCM Server")
    parser.add_argument("--host", default="0.0.0.0", help="Host to bind to")
    parser.add_argument("--port", type=int, default=8000, help="Port to bind to")
    parser.add_argument("--rcm", help="Path to RCM JSON file to load")
    parser.add_argument("--robot-id", default="default", help="Robot ID for loaded RCM")
    
    args = parser.parse_args()
    
    server = RCMServer()
    
    # Load RCM if provided
    if args.rcm:
        server.load_rcm_from_file(args.robot_id, args.rcm)
    
    server.run(host=args.host, port=args.port)

if __name__ == "__main__":
    main()
