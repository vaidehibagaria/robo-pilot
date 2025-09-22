#!/usr/bin/env python3
"""
Setup script for GPT integration with RCM Robot Control Framework
"""

import os
import sys
from pathlib import Path

def create_env_file():
    """Create .env file for configuration"""
    env_path = Path(".env")
    
    if env_path.exists():
        print("✓ .env file already exists")
        return
    
    print("Creating .env file...")
    
    # Get API key from user
    api_key = input("Enter your OpenAI API key (starts with 'sk-'): ").strip()
    
    if not api_key:
        print("No API key provided. You'll need to set OPENAI_API_KEY environment variable.")
        api_key = "your_openai_api_key_here"
    elif not api_key.startswith('sk-'):
        print("Warning: API key doesn't match expected OpenAI format (should start with 'sk-')")
    
    # Create .env file
    env_content = f"""# OpenAI API Configuration
OPENAI_API_KEY={api_key}

# Optional OpenAI Settings (defaults shown)
OPENAI_MODEL=gpt-4-turbo-preview
OPENAI_MAX_TOKENS=1000
OPENAI_TEMPERATURE=0.1
OPENAI_TIMEOUT=30

# RCM Server Configuration
RCM_SERVER_PORT=8000
RCM_SERVER_HOST=localhost

# Robot Configuration
DEFAULT_ROBOT_ID=turtlebot
DEFAULT_RCM_PATH=turtlebot_rcm.json
"""
    
    with open(env_path, 'w') as f:
        f.write(env_content)
    
    print(f"✓ Created .env file with API key")

def install_dependencies():
    """Install required dependencies"""
    print("Installing dependencies...")
    
    try:
        import subprocess
        result = subprocess.run([sys.executable, "-m", "pip", "install", "-r", "requirements.txt"], 
                              capture_output=True, text=True)
        
        if result.returncode == 0:
            print("✓ Dependencies installed successfully")
        else:
            print(f"✗ Error installing dependencies: {result.stderr}")
            return False
    except Exception as e:
        print(f"✗ Error installing dependencies: {e}")
        return False
    
    return True

def test_gpt_connection():
    """Test GPT API connection"""
    print("Testing GPT API connection...")
    
    try:
        from gpt_integration import create_gpt_integration
        import asyncio
        
        async def test():
            gpt = create_gpt_integration()
            return await gpt.test_connection()
        
        if asyncio.run(test()):
            print("✓ GPT API connection successful")
            return True
        else:
            print("✗ GPT API connection failed")
            return False
            
    except Exception as e:
        print(f"✗ Error testing GPT connection: {e}")
        return False

def main():
    """Main setup function"""
    print("RCM Robot Control Framework - GPT Integration Setup")
    print("=" * 50)
    
    # Check if we're in the right directory
    if not Path("main.py").exists():
        print("✗ Please run this script from the RCM project directory")
        sys.exit(1)
    
    # Install dependencies
    if not install_dependencies():
        print("Setup failed at dependency installation")
        sys.exit(1)
    
    # Create .env file
    create_env_file()
    
    # Test connection
    test_connection = input("\nTest GPT API connection now? (y/n): ").strip().lower()
    if test_connection == 'y':
        if test_gpt_connection():
            print("\n✓ Setup completed successfully!")
            print("\nYou can now run the framework with GPT integration:")
            print("  python main.py --rcm turtlebot_rcm.json --robot-id turtlebot")
        else:
            print("\n⚠ Setup completed but GPT connection test failed.")
            print("Please check your API key in the .env file.")
    else:
        print("\n✓ Setup completed!")
        print("Remember to test your GPT connection before using the framework.")
    
    print("\nFor more information, see the README.md file.")

if __name__ == "__main__":
    main()
