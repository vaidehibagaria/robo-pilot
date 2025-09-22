#!/bin/bash

# RCM Robot Control Framework - Docker Startup Script

echo "🤖 RCM Robot Control Framework"
echo "================================"

# Check if .env file exists, if not create a template
if [ ! -f .env ]; then
    echo "📝 Creating .env template file..."
    cat > .env << EOF
# OpenAI API Configuration
OPENAI_API_KEY=your_openai_api_key_here

# Optional OpenAI Settings (defaults shown)
OPENAI_MODEL=gpt-4-turbo-preview
OPENAI_MAX_TOKENS=1000
OPENAI_TEMPERATURE=0.1
OPENAI_TIMEOUT=30

# RCM Server Configuration
RCM_SERVER_PORT=8000
RCM_SERVER_HOST=0.0.0.0

# Robot Configuration
DEFAULT_ROBOT_ID=turtlebot
DEFAULT_RCM_PATH=turtlebot_rcm.json
EOF
    echo "✓ Created .env template file"
    echo ""
    echo "⚠️  Please edit .env file and add your OpenAI API key before running the framework"
    echo "   You can also set the OPENAI_API_KEY environment variable"
    echo ""
fi

# Source environment variables
if [ -f .env ]; then
    export $(cat .env | grep -v '^#' | xargs)
fi

# Check if API key is set
if [ -z "$OPENAI_API_KEY" ] || [ "$OPENAI_API_KEY" = "your_openai_api_key_here" ]; then
    echo "⚠️  No OpenAI API key found!"
    echo "   Please set OPENAI_API_KEY in .env file or as environment variable"
    echo "   The framework will run in demo mode without GPT integration"
    echo ""
    DEMO_MODE=true
else
    echo "✓ OpenAI API key found"
    DEMO_MODE=false
fi

# Check if RCM file exists, if not generate one
if [ ! -f "$DEFAULT_RCM_PATH" ]; then
    echo "📋 RCM file not found. Generating default RCM..."
    
    # Check if we have a URDF file to convert
    if [ -f "turtlebot3_burger.urdf" ]; then
        echo "   Converting URDF to RCM..."
        python3 urdf_to_rcm.py turtlebot3_burger.urdf -o "$DEFAULT_RCM_PATH"
    else
        echo "   Using built-in TurtleBot RCM..."
        # The turtlebot_rcm.json should already exist in the repo
        if [ ! -f "turtlebot_rcm.json" ]; then
            echo "   Creating basic TurtleBot RCM..."
            cat > turtlebot_rcm.json << 'EOF'
{
  "robot_id": "turtlebot",
  "name": "TurtleBot3 Burger",
  "description": "Mobile robot with differential drive",
  "capabilities": {
    "locomotion": {
      "type": "differential_drive",
      "max_linear_velocity": 0.22,
      "max_angular_velocity": 2.84,
      "wheel_base": 0.16
    },
    "sensors": {
      "lidar": {
        "frame": "base_scan",
        "range_min": 0.12,
        "range_max": 3.5,
        "angle_min": -3.14159,
        "angle_max": 3.14159
      }
    }
  },
  "constraints": {
    "workspace": {
      "x_min": -10.0,
      "x_max": 10.0,
      "y_min": -10.0,
      "y_max": 10.0,
      "z_min": 0.0,
      "z_max": 0.5
    }
  }
}
EOF
        fi
        cp turtlebot_rcm.json "$DEFAULT_RCM_PATH"
    fi
    echo "✓ RCM file created: $DEFAULT_RCM_PATH"
fi

# Start the framework
echo ""
echo "🚀 Starting RCM Robot Control Framework..."
echo "   Robot: $DEFAULT_ROBOT_ID"
echo "   RCM: $DEFAULT_RCM_PATH"
echo "   Server: http://localhost:8000"
echo ""

if [ "$DEMO_MODE" = true ]; then
    echo "🎭 Running in DEMO mode (no GPT integration)"
    python3 main.py --rcm "$DEFAULT_RCM_PATH" --robot-id "$DEFAULT_ROBOT_ID" --demo
else
    echo "🤖 Running with GPT integration"
    python3 main.py --rcm "$DEFAULT_RCM_PATH" --robot-id "$DEFAULT_ROBOT_ID" --api-key "$OPENAI_API_KEY"
fi

