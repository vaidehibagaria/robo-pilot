#!/usr/bin/env python3
"""
Test script to verify Docker setup and basic functionality
"""

import json
import os
import sys
from pathlib import Path

def test_imports():
    """Test that all required modules can be imported"""
    try:
        import fastapi
        import uvicorn
        import httpx
        import pydantic
        import openai
        from dotenv import load_dotenv
        print("✓ All required Python packages imported successfully")
        return True
    except ImportError as e:
        print(f"✗ Import error: {e}")
        return False

def test_rcm_files():
    """Test that RCM files exist and are valid JSON"""
    rcm_files = [
        "turtlebot_rcm.json",
        "ur5_rcm.json",
        "panda_rcm.json"
    ]
    
    for rcm_file in rcm_files:
        if Path(rcm_file).exists():
            try:
                with open(rcm_file, 'r') as f:
                    data = json.load(f)
                print(f"✓ {rcm_file} is valid JSON")
            except json.JSONDecodeError as e:
                print(f"✗ {rcm_file} is not valid JSON: {e}")
                return False
        else:
            print(f"⚠ {rcm_file} not found (optional)")
    
    return True

def test_main_script():
    """Test that main.py can be imported and has required functions"""
    try:
        # Add current directory to path
        sys.path.insert(0, str(Path.cwd()))
        
        from main import RCMFramework
        print("✓ main.py imports successfully")
        
        # Test framework initialization
        framework = RCMFramework()
        print("✓ RCMFramework initializes successfully")
        
        return True
    except Exception as e:
        print(f"✗ Error testing main.py: {e}")
        return False

def test_environment():
    """Test environment setup"""
    # Check if .env file exists
    if Path(".env").exists():
        print("✓ .env file exists")
    else:
        print("⚠ .env file not found (will be created by start.sh)")
    
    # Check Python version
    python_version = sys.version_info
    if python_version >= (3, 8):
        print(f"✓ Python version {python_version.major}.{python_version.minor} is supported")
    else:
        print(f"✗ Python version {python_version.major}.{python_version.minor} is not supported (need 3.8+)")
        return False
    
    return True

def test_docker_files():
    """Test that Docker files exist and are valid"""
    docker_files = ["Dockerfile", "docker-compose.yml", "start.sh"]
    
    for docker_file in docker_files:
        if Path(docker_file).exists():
            print(f"✓ {docker_file} exists")
        else:
            print(f"✗ {docker_file} missing")
            return False
    
    # Check if start.sh is executable
    start_sh = Path("start.sh")
    if start_sh.exists():
        if os.access(start_sh, os.X_OK):
            print("✓ start.sh is executable")
        else:
            print("⚠ start.sh is not executable (will be fixed in Docker)")
    
    return True

def main():
    """Run all tests"""
    print("🧪 Testing RCM Robot Control Framework Docker Setup")
    print("=" * 60)
    
    tests = [
        ("Environment", test_environment),
        ("Docker Files", test_docker_files),
        ("Python Imports", test_imports),
        ("RCM Files", test_rcm_files),
        ("Main Script", test_main_script),
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\n🔍 Testing {test_name}...")
        if test_func():
            passed += 1
        else:
            print(f"❌ {test_name} test failed")
    
    print("\n" + "=" * 60)
    print(f"📊 Test Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All tests passed! Docker setup should work correctly.")
        return 0
    else:
        print("⚠️  Some tests failed. Please check the issues above.")
        return 1

if __name__ == "__main__":
    sys.exit(main())

