#!/usr/bin/env python3
"""
Diagnose why mesh data isn't reaching SLAM3DView.

Tests the entire data flow:
1. ROS topic /nvblox_node/color_layer_marker publishing
2. ros_http_bridge receiving and forwarding
3. Edge Core API receiving and storing
4. WebSocket /ws/slam sending to Mission Planner

Run on Jetson:
    python3 scripts/dev/diagnose_mesh.py
"""
import argparse
import json
import os
import sys
import time
import requests

# Test ROS topic availability
def test_ros_topic():
    """Check if nvblox is publishing mesh data."""
    print("=" * 70)
    print("TEST 1: ROS Topic /nvblox_node/color_layer_marker")
    print("=" * 70)
    
    try:
        import rclpy
        from rclpy.node import Node
        from visualization_msgs.msg import Marker
        
        rclpy.init()
        node = Node('mesh_diagnostic')
        
        received = [False]
        msg_count = [0]
        voxel_count = [0]
        
        def callback(msg):
            received[0] = True
            msg_count[0] += 1
            voxel_count[0] = len(msg.points)
            if msg_count[0] <= 3:
                print(f"  ✓ Message #{msg_count[0]}: {len(msg.points)} voxels, type={msg.type}")
        
        sub = node.create_subscription(
            Marker,
            '/nvblox_node/color_layer_marker',
            callback,
            10
        )
        
        print("  Listening for 5 seconds...")
        start = time.time()
        while time.time() - start < 5.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        
        node.destroy_node()
        rclpy.shutdown()
        
        if not received[0]:
            print("  ✗ FAILED: No messages received")
            print("    - Is nvblox_node running?")
            print("    - Check: ros2 topic list | grep nvblox")
            print("    - Check: ros2 topic echo /nvblox_node/color_layer_marker --once")
            return False
        else:
            print(f"  ✓ PASSED: {msg_count[0]} messages, last had {voxel_count[0]} voxels")
            if voxel_count[0] == 0:
                print("    ⚠ WARNING: Voxel count is 0 (camera may need to move)")
            return True
    
    except ImportError as e:
        print(f"  ✗ FAILED: ROS not available ({e})")
        print("    - Run this script inside Isaac ROS container")
        return False
    except Exception as e:
        print(f"  ✗ FAILED: {e}")
        return False

def test_edge_core_api(base_url="http://172.17.0.1:8000"):
    """Check if Edge Core is receiving mesh data."""
    print("\n" + "=" * 70)
    print("TEST 2: Edge Core API /api/task/2/slam/mesh")
    print("=" * 70)
    
    try:
        url = f"{base_url}/api/task/2/slam/mesh?format=summary"
        print(f"  GET {url}")
        
        resp = requests.get(url, timeout=5)
        if resp.status_code != 200:
            print(f"  ✗ FAILED: HTTP {resp.status_code}")
            return False
        
        data = resp.json()
        print(f"  Response: {json.dumps(data, indent=2)}")
        
        if data.get("available"):
            print(f"  ✓ PASSED: Mesh data available")
            print(f"    - Mode: {data.get('mode')}")
            print(f"    - Items: {data.get('block_count')}/{data.get('total_blocks')}")
            print(f"    - Timestamp: {data.get('timestamp')}")
            return True
        else:
            print("  ✗ FAILED: Mesh data not available")
            print("    - Is ros_http_bridge running?")
            print("    - Check: curl http://172.17.0.1:8000/api/vio/status")
            return False
    
    except requests.exceptions.ConnectionError:
        print(f"  ✗ FAILED: Cannot connect to {base_url}")
        print("    - Is Edge Core running?")
        print("    - Check: curl http://172.17.0.1:8000/health")
        return False
    except Exception as e:
        print(f"  ✗ FAILED: {e}")
        return False

def test_ros_http_bridge(base_url="http://172.17.0.1:8000"):
    """Check ros_http_bridge stats."""
    print("\n" + "=" * 70)
    print("TEST 3: ros_http_bridge Stats")
    print("=" * 70)
    
    try:
        url = f"{base_url}/api/vio/status"
        print(f"  GET {url}")
        
        resp = requests.get(url, timeout=5)
        if resp.status_code != 200:
            print(f"  ✗ FAILED: HTTP {resp.status_code}")
            return False
        
        data = resp.json()
        
        # Check if bridge stats exist
        if "bridge_stats" not in data:
            print("  ✗ FAILED: No bridge_stats in response")
            print("    - ros_http_bridge may not be running")
            return False
        
        stats = data["bridge_stats"]
        mesh_recv = stats.get("mesh_received", 0)
        mesh_sent = stats.get("mesh_sent", 0)
        
        print(f"  Bridge Stats:")
        print(f"    - Mesh received from ROS: {mesh_recv}")
        print(f"    - Mesh sent to Edge Core: {mesh_sent}")
        print(f"    - VIO received: {stats.get('vio_received', 0)}")
        
        if mesh_recv == 0:
            print("  ✗ FAILED: Bridge not receiving mesh from ROS")
            print("    - Check TEST 1 (ROS topic)")
            print("    - Is --enable-mesh flag set?")
            return False
        elif mesh_sent == 0:
            print("  ✗ FAILED: Bridge receiving but not sending")
            print("    - Check Edge Core logs for errors")
            return False
        else:
            print(f"  ✓ PASSED: Bridge active ({mesh_recv} recv, {mesh_sent} sent)")
            return True
    
    except Exception as e:
        print(f"  ✗ FAILED: {e}")
        return False

def test_websocket(base_url="http://172.17.0.1:8000"):
    """Check WebSocket /ws/slam for mesh delivery."""
    print("\n" + "=" * 70)
    print("TEST 4: WebSocket /ws/slam")
    print("=" * 70)
    
    try:
        import websocket
        
        ws_url = base_url.replace("http://", "ws://").replace("https://", "wss://") + "/ws/slam"
        print(f"  Connecting to {ws_url}")
        
        ws = websocket.create_connection(ws_url, timeout=5)
        print("  ✓ Connected")
        
        mesh_received = False
        pose_count = 0
        
        print("  Listening for 5 seconds...")
        start = time.time()
        while time.time() - start < 5.0:
            try:
                msg = ws.recv()
                data = json.loads(msg)
                msg_type = data.get("type")
                
                if msg_type == "pose":
                    pose_count += 1
                    if pose_count <= 3:
                        print(f"    - Pose #{pose_count}: x={data.get('x', 0):.2f}, y={data.get('y', 0):.2f}, yaw={data.get('body_yaw', 0):.1f}°")
                elif msg_type == "mesh":
                    mesh_received = True
                    mesh_data = data.get("mesh", {})
                    voxels = mesh_data.get("voxels", [])
                    print(f"    ✓ Mesh message: {len(voxels)} voxels")
            except:
                break
        
        ws.close()
        
        if pose_count == 0:
            print("  ✗ FAILED: No pose messages received")
            return False
        elif not mesh_received:
            print(f"  ⚠ WARNING: {pose_count} pose messages but NO mesh")
            print("    - Mesh may not have changed during test")
            print("    - Try moving camera to generate new voxels")
            return False
        else:
            print(f"  ✓ PASSED: {pose_count} poses + mesh data received")
            return True
    
    except ImportError:
        print("  ⚠ SKIPPED: websocket-client not installed")
        print("    - Install: pip install websocket-client")
        return None
    except Exception as e:
        print(f"  ✗ FAILED: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description="Diagnose mesh transmission")
    parser.add_argument("--base-url", default="http://172.17.0.1:8000", help="Edge Core base URL")
    args = parser.parse_args()
    
    print("\n" + "=" * 70)
    print("NOMAD MESH TRANSMISSION DIAGNOSTIC")
    print("=" * 70)
    print(f"Edge Core URL: {args.base_url}")
    print("")
    
    results = {}
    
    # Test 1: ROS topic
    if os.environ.get("ROS_DISTRO"):
        results["ros_topic"] = test_ros_topic()
    else:
        print("=" * 70)
        print("TEST 1: ROS Topic (SKIPPED - not in ROS environment)")
        print("=" * 70)
        results["ros_topic"] = None
    
    # Test 2: Edge Core API
    results["api"] = test_edge_core_api(args.base_url)
    
    # Test 3: Bridge stats
    results["bridge"] = test_ros_http_bridge(args.base_url)
    
    # Test 4: WebSocket
    results["websocket"] = test_websocket(args.base_url)
    
    # Summary
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    
    passed = sum(1 for v in results.values() if v is True)
    failed = sum(1 for v in results.values() if v is False)
    skipped = sum(1 for v in results.values() if v is None)
    
    for test, result in results.items():
        status = "✓ PASS" if result is True else ("✗ FAIL" if result is False else "⚠ SKIP")
        print(f"  {test:15s}: {status}")
    
    print("")
    print(f"Results: {passed} passed, {failed} failed, {skipped} skipped")
    
    if failed > 0:
        print("\n" + "=" * 70)
        print("TROUBLESHOOTING STEPS")
        print("=" * 70)
        if not results.get("ros_topic"):
            print("1. Start nvblox:")
            print("   ros2 launch nvblox_examples_bringup realsense_example.launch.py")
        if not results.get("api"):
            print("2. Start Edge Core:")
            print("   python3 -m edge_core.main --port 8000")
        if not results.get("bridge"):
            print("3. Start ros_http_bridge:")
            print("   python3 edge_core/ros_http_bridge.py --host 172.17.0.1 --enable-mesh")
        if not results.get("websocket"):
            print("4. Check Mission Planner connection to WebSocket")
    else:
        print("\n✓ All tests passed! Mesh should be visible in SLAM3DView.")
        print("  If still showing 'waiting for mesh', check Mission Planner logs.")
    
    return 0 if failed == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
