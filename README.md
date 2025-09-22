# 🤖 RCM Framework

**No-code natural language control for any robot using AI language models**

[![Docker](https://img.shields.io/badge/Docker-Ready-blue?logo=docker)](https://hub.docker.com)
[![Python](https://img.shields.io/badge/Python-3.8+-green?logo=python)](https://python.org)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Transform any robot into an AI-controllable system using natural language commands. No coding required - just describe what you want the robot to do in plain English.

## ✨ Features

- 🗣️ **Natural Language Control**: Control robots using natural language prompts
- 🔧 **No-Code Setup**: Works with any robot via Docker
- 🤖 **Multi-Model Support**: GPT-4, Claude, or any OpenAI-compatible API
- 🛡️ **Safety First**: Built-in constraint enforcement and capability validation
- 📊 **Real-time Tracking**: Live position monitoring and path visualization
- 🔄 **Multi-Robot**: Support for multiple robots simultaneously
- 🚀 **One-Click Deploy**: Docker container with everything included
- 🧠 **Semantic Understanding**: Advanced environment understanding with specialized LLM for grasping tasks

## 🚀 Quick Start

### Option 1: Docker (Recommended)

**Step 1: Launch your robot**
```bash
# First, launch your robot in one terminal
ros2 launch your_robot_package your_robot_launch_file.launch.py
```

**Step 2: Run the RCM framework**
```bash
# Clone the repository
git clone https://github.com/yourusername/rcm-robot-control.git
cd rcm-robot-control

# Set your OpenAI API key
export OPENAI_API_KEY="your-api-key-here"

# Run with Docker (in another terminal)
docker-compose up --build
```

> **Note**: The framework automatically detects your robot from ROS topics (`/robot_description` and `/robot_description_semantic`). No need to specify robot files - it gets everything from the running ROS system.

### Option 2: Local Installation

**Step 1: Launch your robot**
```bash
# First, launch your robot in one terminal
ros2 launch your_robot_package your_robot_launch_file.launch.py
```

**Step 2: Run the RCM framework**
```bash
# Clone and setup
git clone https://github.com/yourusername/rcm-robot-control.git
cd rcm-robot-control

# Install dependencies
pip install -r requirements.txt

# Setup GPT integration
python setup_gpt.py

# Generate RCM from ROS robot description (in another terminal)
python urdf_to_rcm.py -o my_robot_rcm.json

# Run the framework
python main_ros.py --json my_robot_rcm.json
```

## 💬 Example Usage

Once running, simply type natural language commands:

```
You: Move forward and then turn left
Robot: ✓ Executing drive_straight(2.0m) then rotate_in_place(90°)

You: What sensors do you have?
Robot: I have a LIDAR sensor with 360° range detection

You: Stop immediately
Robot: ✓ Emergency stop activated
```

## 🔧 Supported Robots

- **Any ROS-compatible robot** with URDF description
- **Mobile robots** with differential drive
- **Manipulator arms** with joint control
- **Custom robots** via URDF conversion

## 📋 Robot Capability Manifest (RCM)

The framework uses RCMs to understand robot capabilities:

```json
{
  "robot_id": "robot",
  "capabilities": {
    "locomotion": {
      "type": "differential_drive",
      "max_linear_velocity": 0.22,
      "max_angular_velocity": 2.84
    },
    "sensors": {
      "lidar": {
        "range_max": 3.5,
        "angle_range": "360°"
      }
    }
  }
}
```

## 🛠️ Adding Your Robot

1. **Launch your robot**:
   ```bash
   ros2 launch your_robot_package your_robot_launch_file.launch.py
   ```

2. **Generate RCM from ROS robot description**:
   ```bash
   python urdf_to_rcm.py -o my_robot_rcm.json
   ```

3. **Run with your robot**:
   ```bash
   python main_ros.py --json my_robot_rcm.json
   ```

## 🐳 Docker & ROS Integration

The framework works seamlessly with Docker because it **does not search the filesystem** for robot files. Instead, it:

- **Connects to ROS topics** (`/robot_description`, `/robot_description_semantic`) 
- **Gets robot data from running ROS nodes** (not from files)
- **Works across Docker containers** as long as ROS topics are accessible

### Docker Network Setup

For Docker to access ROS topics from your host system, ensure your `docker-compose.yml` includes:

```yaml
network_mode: "host"
```

This allows the Docker container to access the same network as your ROS system, enabling topic communication.

## 🔧 Troubleshooting

### Common Issues

**"Did not receive /robot_description from ROS"**
- Make sure your robot is launched and running
- Check that `/robot_description` topic is published: `ros2 topic list | grep robot_description`
- Verify ROS environment is sourced in the terminal where you launch the robot

**Docker can't connect to ROS topics**
- Ensure `network_mode: "host"` is set in `docker-compose.yml`
- Make sure ROS_DOMAIN_ID is consistent between host and container
- Check firewall settings that might block local communication

**Robot not responding to commands**
- Verify the robot is properly launched and controllers are active
- Check ROS topic names match your robot's configuration
- Ensure the generated RCM file contains the correct robot capabilities

## 🔧 Configuration

Create a `.env` file to customize settings:

```env
OPENAI_API_KEY=your_api_key_here
OPENAI_MODEL=gpt-4-turbo-preview
OPENAI_TEMPERATURE=0.1
DEFAULT_ROBOT_ID=turtlebot
DEFAULT_RCM_PATH=turtlebot_rcm.json
```

## 📊 API Endpoints

The framework exposes a REST API for programmatic access:

- `GET /robots` - List available robots
- `GET /robots/{robot_id}/capabilities` - Get robot capabilities
- `POST /robots/{robot_id}/execute` - Execute robot commands

## 🏗️ Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Natural       │    │   RCM Server     │    │   Robot         │
│   Language      │───▶│   (FastAPI)      │───▶│   Controller    │
│   Commands      │    │                  │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   AI Model      │    │   Tool Generator │    │   Safety        │
│   (GPT/Claude)  │    │   (Dynamic)      │    │   Constraints   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

## 🤝 Contributing

We welcome contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Built with [FastAPI](https://fastapi.tiangolo.com/) and [OpenAI](https://openai.com/)
- Robot kinematics powered by [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
- ROS 2 integration for real robot control



