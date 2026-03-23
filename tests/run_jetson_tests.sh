#!/bin/bash
# P1-1 & P3-7 Test Runner for Jetson Orin Nano
# Place in ~/NOMAD/tests/ and run: bash run_jetson_tests.sh

set -e

TEST_DIR="$(dirname "$0")"
NOMAD_ROOT="$(dirname "$TEST_DIR")"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$TEST_DIR/jetson_test_${TIMESTAMP}.log"

echo "======================================================================"
echo "P1-1 & P3-7 Jetson Test Suite"
echo "======================================================================"
echo "Start time: $(date)"
echo "Log file: $LOG_FILE"
echo "======================================================================"

{
    echo "======================================================================"
    echo "Test Environment"
    echo "======================================================================"
    echo "Hostname: $(hostname)"
    echo "User: $(whoami)"
    echo "Working Dir: $(pwd)"
    echo "NOMAD Root: $NOMAD_ROOT"
    echo ""
    
    # Test 1: Git status
    echo "======================================================================"
    echo "Test 1: Git Changes Status"
    echo "======================================================================"
    cd "$NOMAD_ROOT"
    git status
    echo ""
    
    # Test 2: P1-1 code changes present
    echo "======================================================================"
    echo "Test 2: P1-1 Code Changes Verification"
    echo "======================================================================"
    
    echo "Checking edge_core/api.py for frame_id..."
    if grep -q '"frame_id": "ros_optical"' edge_core/api.py; then
        echo "✓ Found frame_id in api.py (WebSocket endpoint)"
        grep -n '"frame_id": "ros_optical"' edge_core/api.py | head -3
    else
        echo "✗ frame_id NOT found in api.py"
    fi
    echo ""
    
    echo "Checking edge_core/ros_http_bridge.py for frame_id..."
    if grep -q "frame_id.*ros_optical" edge_core/ros_http_bridge.py; then
        echo "✓ Found frame_id in ros_http_bridge.py"
        grep -n "frame_id.*ros_optical" edge_core/ros_http_bridge.py | head -3
    else
        echo "✗ frame_id NOT found in ros_http_bridge.py"
    fi
    echo ""
    
    # Test 3: Service status
    echo "======================================================================"
    echo "Test 3: Systemd Service Status"
    echo "======================================================================"
    systemctl status nomad.service || echo "Note: nomad.service not as systemd service"
    echo ""
    
    # Test 4: API health check
    echo "======================================================================"
    echo "Test 4: API Health Check"
    echo "======================================================================"
    if curl -s http://localhost:8000/health > /dev/null 2>&1; then
        echo "✓ API responding on port 8000"
        curl -s http://localhost:8000/health | python3 -m json.tool
    else
        echo "✗ API not responding (edge_core may not be running)"
        echo "   Start with: sudo systemctl restart nomad.service"
    fi
    echo ""
    
    # Test 5: Recent logs check
    echo "======================================================================"
    echo "Test 5: Recent Edge Core Activity"
    echo "======================================================================"
    # Check for recent log files
    if [ -d ~/.local/share/nomad/logs/ ]; then
        echo "Recent log files:"
        ls -lth ~/.local/share/nomad/logs/ | head -5
        echo ""
        echo "Last 10 lines of main log:"
        tail -10 ~/.local/share/nomad/logs/edge_core.log 2>/dev/null || echo "No edge_core.log found"
    else
        echo "Log directory not found at ~/.local/share/nomad/logs/"
    fi
    echo ""
    
    # Test 6: VIO endpoint check
    echo "======================================================================"
    echo "Test 6: VIO API Endpoint Check"
    echo "======================================================================"
    if curl -s http://localhost:8000/api/vio/status > /dev/null 2>&1; then
        echo "✓ VIO endpoint responding"
        curl -s http://localhost:8000/api/vio/status | python3 -m json.tool | head -20
    else
        echo "✗ VIO endpoint not responding"
    fi
    echo ""
    
    # Test 7: Mesh endpoint check
    echo "======================================================================"
    echo "Test 7: Mesh Status Check"
    echo "======================================================================"
    # This would require adding a /api/mesh/status endpoint (or similar)
    # For now, just check if mesh data exists in state
    echo "Note: Mesh status check requires running instance"
    echo "      Connect to WebSocket ws://localhost:8000/ws/slam to view live data"
    echo ""
    
    echo "======================================================================"
    echo "Test Suite Complete"
    echo "======================================================================"
    echo "End time: $(date)"
    
} | tee "$LOG_FILE"

echo ""
echo "Logs saved to: $LOG_FILE"
echo ""
echo "Next steps:"
echo "1. Review log file for any failures"
echo "2. From Windows, run: python tests/test_p1_1_websocket_frame_id.py"
echo "3. From Windows, run: python tests/test_p3_7_debounce.py"
echo ""
