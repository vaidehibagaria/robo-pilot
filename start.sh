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

# Generated RCM Configuration
GENERATED_RCM_PATH=generated_robot_rcm.json
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

# Generate RCM from ROS robot description
echo "📋 Generating RCM from ROS robot description..."
echo "   This will automatically detect your robot from ROS topics"

# Run urdf_to_rcm.py to generate RCM from ROS
python3 urdf_to_rcm.py -o "$GENERATED_RCM_PATH"

if [ $? -eq 0 ]; then
    echo "✓ RCM file generated successfully: $GENERATED_RCM_PATH"
else
    echo "❌ Failed to generate RCM from ROS robot description"
    echo "   Make sure your robot is running and publishing /robot_description topic"
    exit 1
fi

# Start the framework
echo ""
echo "🚀 Starting RCM Robot Control Framework..."
echo "   RCM: $GENERATED_RCM_PATH"
echo "   Server: http://localhost:8000"
echo ""

if [ "$DEMO_MODE" = true ]; then
    echo "🎭 Running in DEMO mode (no GPT integration)"
    python3 main_ros.py --json "$GENERATED_RCM_PATH" --demo
else
    echo "🤖 Running with GPT integration"
    python3 main_ros.py --json "$GENERATED_RCM_PATH" --api-key "$OPENAI_API_KEY"
fi

