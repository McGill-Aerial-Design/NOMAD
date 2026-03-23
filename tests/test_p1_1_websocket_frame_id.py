#!/usr/bin/env python3
"""
Test P1-1 WebSocket frame_id field presence and correctness.
Usage: python test_p1_1_websocket.py --jetson-ip 100.85.121.98 --frames 100
"""

import websocket
import json
import sys
import argparse
import time
from datetime import datetime

def test_frame_id(jetson_ip, frame_count=100, timeout=30):
    """Test WebSocket frames for frame_id field."""
    
    uri = f"ws://{jetson_ip}:8000/ws/slam"
    print(f"\n{'='*70}")
    print(f"P1-1 WebSocket Frame ID Test")
    print(f"{'='*70}")
    print(f"Target: {uri}")
    print(f"Frames to capture: {frame_count}")
    print(f"Timeout: {timeout}s")
    print(f"Start time: {datetime.now()}\n")
    
    try:
        ws = websocket.create_connection(uri, timeout=timeout)
        print("✓ WebSocket connection established\n")
        
        stats = {
            'total_frames': 0,
            'frames_with_id': 0,
            'correct_frame_ids': 0,
            'incorrect_frame_ids': [],
            'missing_frame_ids': [],
            'frame_types': {},
            'sample_frames': [],
        }
        
        start_time = time.time()
        
        for i in range(frame_count):
            try:
                msg = ws.recv(timeout=1.0)
                frame = json.loads(msg)
                
                stats['total_frames'] += 1
                frame_type = frame.get('type', 'unknown')
                stats['frame_types'][frame_type] = stats['frame_types'].get(frame_type, 0) + 1
                
                # Store first few frames for inspection
                if len(stats['sample_frames']) < 3:
                    stats['sample_frames'].append(frame)
                
                # Check for frame_id
                if 'frame_id' in frame:
                    stats['frames_with_id'] += 1
                    frame_id = frame.get('frame_id')
                    
                    if frame_id == 'ros_optical':
                        stats['correct_frame_ids'] += 1
                        if i < 5 or i % 20 == 0:  # Print first 5 and every 20th
                            print(f"✓ Frame {i:3d}: type='{frame_type:6s}', frame_id='{frame_id}'")
                    else:
                        stats['incorrect_frame_ids'].append({
                            'frame': i,
                            'type': frame_type,
                            'value': frame_id
                        })
                        print(f"✗ Frame {i:3d}: type='{frame_type:6s}', frame_id='{frame_id}' (expected 'ros_optical')")
                else:
                    stats['missing_frame_ids'].append({'frame': i, 'type': frame_type})
                    print(f"✗ Frame {i:3d}: type='{frame_type:6s}' - MISSING frame_id")
                    
            except json.JSONDecodeError as e:
                print(f"✗ Frame {i}: JSON parse error - {e}")
            except websocket.WebSocketTimeoutException:
                print(f"✗ Frame {i}: Receive timeout (last frame)")
                break
            except Exception as e:
                print(f"✗ Frame {i}: {type(e).__name__}: {e}")
                break
        
        ws.close()
        elapsed = time.time() - start_time
        
        # Results summary
        print(f"\n{'='*70}")
        print("Test Results Summary")
        print(f"{'='*70}")
        print(f"Total frames received: {stats['total_frames']}")
        print(f"Frames with frame_id: {stats['frames_with_id']} ({stats['frames_with_id']/max(1,stats['total_frames'])*100:.1f}%)")
        print(f"Correct frame_id values: {stats['correct_frame_ids']} ({stats['correct_frame_ids']/max(1,stats['frames_with_id'])*100:.1f}%)")
        print(f"Incorrect frame_id: {len(stats['incorrect_frame_ids'])}")
        print(f"Missing frame_id: {len(stats['missing_frame_ids'])}")
        print(f"\nFrame type distribution: {stats['frame_types']}")
        
        if stats['sample_frames']:
            print(f"\nSample Frame 1 Keys: {list(stats['sample_frames'][0].keys())}")
        
        if stats['incorrect_frame_ids']:
            print(f"\n⚠ Incorrect frame_id values ({len(stats['incorrect_frame_ids'])}):")
            for err in stats['incorrect_frame_ids'][:5]:
                print(f"  Frame {err['frame']:3d} ({err['type']}): {err['value']}")
        
        if stats['missing_frame_ids']:
            print(f"\n⚠ Missing frame_id ({len(stats['missing_frame_ids'])}):")
            for err in stats['missing_frame_ids'][:5]:
                print(f"  Frame {err['frame']:3d} ({err['type']})")
        
        print(f"\nElapsed time: {elapsed:.1f}s")
        print(f"Reception rate: {stats['total_frames']/elapsed:.1f} fps")
        
        # Pass/Fail decision
        print(f"\n{'='*70}")
        if stats['correct_frame_ids'] == stats['total_frames']:
            print(f"✓ PASS: All {stats['total_frames']} frames have correct frame_id")
            print(f"{'='*70}\n")
            return True
        else:
            faults = stats['total_frames'] - stats['correct_frame_ids']
            print(f"✗ FAIL: {faults}/{stats['total_frames']} frames have missing or incorrect frame_id")
            print(f"{'='*70}\n")
            return False
            
    except ConnectionRefusedError:
        print(f"✗ Connection refused on {jetson_ip}:8000")
        print(f"  Is edge_core service running? Check: systemctl status nomad.service")
        print(f"  Is Tailscale connected? Check: tailscale status\n")
        return False
    except websocket.WebSocketConnectionClosedException:
        print(f"✗ WebSocket connection closed unexpectedly")
        print(f"  The edge_core service may have crashed\n")
        return False
    except Exception as e:
        print(f"✗ Error: {type(e).__name__}: {e}\n")
        import traceback
        traceback.print_exc()
        return False

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Test P1-1 WebSocket frame_id field presence and correctness'
    )
    parser.add_argument(
        '--jetson-ip', 
        default='100.85.121.98', 
        help='Jetson IP address (default: 100.85.121.98)'
    )
    parser.add_argument(
        '--frames', 
        type=int, 
        default=100, 
        help='Number of frames to capture (default: 100)'
    )
    parser.add_argument(
        '--timeout', 
        type=int, 
        default=30, 
        help='Timeout in seconds (default: 30)'
    )
    
    args = parser.parse_args()
    
    # Check for websocket-client library
    try:
        import websocket
    except ImportError:
        print("ERROR: websocket-client not installed")
        print("Run: pip install websocket-client")
        sys.exit(1)
    
    success = test_frame_id(args.jetson_ip, args.frames, args.timeout)
    sys.exit(0 if success else 1)
