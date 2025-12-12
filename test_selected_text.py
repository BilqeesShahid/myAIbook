#!/usr/bin/env python3
"""
Test script to verify the selected text functionality works properly
"""
import asyncio
import json
import requests
from datetime import datetime

def test_selected_text_endpoint():
    """Test the selected text endpoint directly"""
    url = "http://localhost:8000/api/ask/selected-text"

    # Sample request data for testing
    test_data = {
        "query": "Summarize this text",
        "selected_text": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.",
        "session_id": f"test_session_{int(datetime.now().timestamp())}",
        "language": "en"
    }

    try:
        response = requests.post(
            url,
            json=test_data,
            headers={"Content-Type": "application/json"},
            timeout=30
        )

        if response.status_code == 200:
            result = response.json()
            print("✅ Selected text endpoint test passed!")
            print(f"Response: {result['response'][:200]}...")
            return True
        else:
            print(f"❌ Selected text endpoint test failed with status {response.status_code}")
            print(f"Response: {response.text}")
            return False

    except requests.exceptions.ConnectionError:
        print("❌ Cannot connect to API server. Make sure the backend is running on localhost:8000")
        return False
    except Exception as e:
        print(f"❌ Error testing selected text endpoint: {e}")
        return False

if __name__ == "__main__":
    print("Testing selected text functionality...")
    success = test_selected_text_endpoint()

    if success:
        print("\n🎉 Selected text functionality is working correctly!")
    else:
        print("\n⚠️  There may be an issue with the selected text functionality.")
        print("Make sure the backend server is running with: python -m uvicorn src.api.main:app --reload --port 8000")