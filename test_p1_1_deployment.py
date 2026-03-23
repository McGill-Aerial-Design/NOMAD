#!/usr/bin/env python3
"""
Test script for P1-1 pose frame normalization deployment verification.
"""

import sys
import json
from pathlib import Path

def check_vio_frame_id():
    """Check if VIOData has frame_id field in ros_http_bridge.py."""
    try:
        sys.path.insert(0, str(Path(__file__).parent))
        from edge_core.ros_http_bridge import VIOData
        
        fields = VIOData.__dataclass_fields__.keys()
        has_frame_id = "frame_id" in fields
        
        print(f"✓ VIOData imported successfully")
        print(f"  Fields: {list(fields)}")
        print(f"  Has 'frame_id': {has_frame_id}")
        
        if has_frame_id:
            import inspect
            source = inspect.getsource(VIOData)
            if 'ros_optical' in source:
                print(f"  frame_id default value confirmed: 'ros_optical'")
        
        return has_frame_id
    except ImportError as e:
        print(f"✗ Failed to import VIOData: {e}")
        return False

def check_api_frame_id():
    """Check if api.py includes frame_id in WebSocket messages."""
    try:
        api_file = Path(__file__).parent / "edge_core" / "api.py"
        content = api_file.read_text()
        
        # Check for frame_id in WebSocket endpoint
        has_websocket_frame_id = '"frame_id": "ros_optical"' in content
        
        print(f"✓ api.py read successfully")
        print(f"  Has frame_id in WebSocket: {has_websocket_frame_id}")
        
        # Count occurrences
        count = content.count('"frame_id"')
        print(f"  Total frame_id occurrences: {count}")
        
        return has_websocket_frame_id
    except Exception as e:
        print(f"✗ Failed to check api.py: {e}")
        return False

def main():
    print("=" * 60)
    print("P1-1 Pose Frame Normalization Deployment Check")
    print("=" * 60)
    print()
    
    print("1. Checking VIOData dataclass...")
    vio_ok = check_vio_frame_id()
    print()
    
    print("2. Checking API WebSocket frame_id...")
    api_ok = check_api_frame_id()
    print()
    
    print("=" * 60)
    if vio_ok and api_ok:
        print("✓ P1-1 changes DETECTED in local workspace")
        sys.exit(0)
    else:
        print("✗ P1-1 changes INCOMPLETE or MISSING")
        sys.exit(1)

if __name__ == "__main__":
    main()
