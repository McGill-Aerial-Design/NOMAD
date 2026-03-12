#!/usr/bin/env python3
"""
ROS2 Mesh Bridge for NOMAD - nvblox 3D SLAM Visualization.

This module bridges the nvblox ROS2 mesh topic to HTTP endpoints for
visualization in the Mission Planner plugin on Windows Ground Control Station.

Architecture:
- Subscribes to /nvblox_node/mesh (nvblox_msgs/Mesh)
- Converts mesh blocks to simplified JSON format (vertices, triangles, colors)
- Provides thread-safe access to the latest mesh data
- Supports delta updates (tracking changed blocks)
- Enables 3D SLAM visualization in Mission Planner GCS

Usage:
    from edge_core.ros_mesh_bridge import MeshBridge
    
    bridge = MeshBridge()
    bridge.start()
    
    # Get latest mesh for HTTP API
    mesh_data = bridge.get_latest_mesh()
    
    # Get only changed blocks since last request
    delta = bridge.get_mesh_delta()

Target: NVIDIA Jetson Orin Nano with Isaac ROS + nvblox
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, asdict
from typing import Optional, Dict, List, Tuple

logger = logging.getLogger(__name__)

# ROS2 imports - required
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    ROS2_AVAILABLE = True
except ImportError:
    logger.warning("ROS2 not available - MeshBridge will not function")
    ROS2_AVAILABLE = False
    Node = object  # Dummy for type hints

# nvblox message imports
try:
    from nvblox_msgs.msg import Mesh, MeshBlock
    from geometry_msgs.msg import Point32
    from std_msgs.msg import ColorRGBA
    NVBLOX_AVAILABLE = True
except ImportError:
    logger.warning("nvblox_msgs not available - using mock types")
    NVBLOX_AVAILABLE = False
    Mesh = None
    MeshBlock = None


@dataclass
class SimplifiedMeshBlock:
    """
    Simplified mesh block data for JSON serialization.
    
    Represents a single voxel block from nvblox's mesh output,
    converted to a format easily consumed by Mission Planner.
    """
    index: Tuple[int, int, int]  # Block index (x, y, z)
    vertices: List[List[float]]  # List of [x, y, z] coordinates
    triangles: List[List[int]]   # List of [idx1, idx2, idx3] vertex indices
    colors: Optional[List[List[int]]] = None  # Optional [r, g, b] per vertex (0-255)
    timestamp: float = 0.0  # Unix timestamp when block was received


@dataclass
class MeshData:
    """
    Complete mesh data container.
    
    Contains all mesh blocks and metadata for the current SLAM map.
    """
    blocks: List[SimplifiedMeshBlock]
    block_size: float  # Size of each voxel block in meters
    total_vertices: int
    total_triangles: int
    timestamp: float  # Unix timestamp of mesh update
    frame_id: str = "map"


class MeshBridgeNode(Node):
    """ROS2 node for subscribing to nvblox mesh topic."""
    
    def __init__(self, mesh_topic: str, callback):
        super().__init__("nomad_mesh_bridge")
        
        # QoS profile optimized for mesh data
        # TRANSIENT_LOCAL ensures we get the last published mesh on startup
        mesh_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        
        # Subscribe to mesh topic
        self.create_subscription(
            Mesh,
            mesh_topic,
            callback,
            mesh_qos,
        )
        
        self.get_logger().info(f"Subscribed to mesh: {mesh_topic}")


class MeshBridge:
    """
    Bridge between nvblox ROS2 mesh and HTTP API.
    
    Thread-safe mesh data management with delta tracking for
    efficient updates to Mission Planner GCS.
    
    Features:
    - Automatic mesh updates from nvblox
    - Full mesh access for initial visualization
    - Delta updates for incremental changes
    - Thread-safe concurrent access
    - Comprehensive statistics and monitoring
    """
    
    def __init__(
        self,
        mesh_topic: str = "/nvblox_node/mesh",
        enable_colors: bool = True,
        max_blocks: int = 10000,  # Safety limit on mesh size
    ):
        """
        Initialize the mesh bridge.
        
        Args:
            mesh_topic: ROS2 topic for nvblox mesh output
            enable_colors: Whether to include color data (if available)
            max_blocks: Maximum number of blocks to store (safety limit)
        """
        if not ROS2_AVAILABLE or not NVBLOX_AVAILABLE:
            logger.error("ROS2 or nvblox_msgs not available - MeshBridge disabled")
            self._available = False
            return
        
        self._mesh_topic = mesh_topic
        self._enable_colors = enable_colors
        self._max_blocks = max_blocks
        
        # State
        self._running = False
        self._lock = threading.Lock()
        self._latest_mesh: Optional[MeshData] = None
        self._mesh_timestamp: Optional[float] = None
        
        # Delta tracking: track which blocks have changed since last delta request
        self._changed_blocks: Dict[Tuple[int, int, int], SimplifiedMeshBlock] = {}
        self._last_delta_timestamp: float = 0.0
        
        # ROS2 node and spin thread
        self._node: Optional[MeshBridgeNode] = None
        self._spin_thread: Optional[threading.Thread] = None
        
        # Statistics
        self._mesh_count = 0
        self._block_count = 0
        self._error_count = 0
        self._last_update_time = 0.0
        
        self._available = True
        logger.info(f"MeshBridge initialized for topic: {mesh_topic}")
    
    # ========================================================================
    # Lifecycle
    # ========================================================================
    
    def start(self) -> bool:
        """
        Start the mesh bridge.
        
        Returns:
            True if started successfully, False otherwise
        """
        if not self._available:
            logger.error("MeshBridge not available")
            return False
        
        if self._running:
            logger.warning("MeshBridge already running")
            return True
        
        try:
            # Initialize ROS2
            if not rclpy.ok():
                rclpy.init()
            
            # Create node
            self._node = MeshBridgeNode(
                mesh_topic=self._mesh_topic,
                callback=self._handle_mesh,
            )
            
            # Start spin thread
            self._running = True
            self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
            self._spin_thread.start()
            
            logger.info("MeshBridge started successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start MeshBridge: {e}")
            self._running = False
            return False
    
    def stop(self) -> None:
        """Stop the mesh bridge."""
        if not self._running:
            return
        
        self._running = False
        
        # Cleanup ROS2
        if self._node:
            try:
                self._node.destroy_node()
            except Exception as e:
                logger.error(f"Error destroying node: {e}")
        
        # Shutdown ROS2 (only if we own the context)
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            logger.error(f"Error shutting down ROS2: {e}")
        
        # Wait for spin thread
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        
        logger.info("MeshBridge stopped")
    
    def _spin_loop(self) -> None:
        """ROS2 spin loop in background thread."""
        while self._running and rclpy.ok():
            try:
                rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception as e:
                logger.error(f"Spin error: {e}")
                self._error_count += 1
    
    # ========================================================================
    # Mesh Processing
    # ========================================================================
    
    def _handle_mesh(self, msg: Mesh) -> None:
        """
        Handle incoming mesh message from nvblox.
        
        Args:
            msg: nvblox_msgs/Mesh message containing mesh blocks
        """
        try:
            start_time = time.time()
            
            # Extract block size
            block_size = msg.block_size if hasattr(msg, 'block_size') else 0.2  # Default to 20cm
            
            # Process all mesh blocks
            blocks = []
            total_vertices = 0
            total_triangles = 0
            
            for ros_block in msg.blocks[:self._max_blocks]:  # Safety limit
                block = self._process_mesh_block(ros_block)
                if block:
                    blocks.append(block)
                    total_vertices += len(block.vertices)
                    total_triangles += len(block.triangles)
                    
                    # Track changed blocks for delta updates
                    with self._lock:
                        self._changed_blocks[block.index] = block
            
            # Create mesh data container
            mesh_data = MeshData(
                blocks=blocks,
                block_size=block_size,
                total_vertices=total_vertices,
                total_triangles=total_triangles,
                timestamp=time.time(),
                frame_id=msg.header.frame_id if hasattr(msg, 'header') else "map",
            )
            
            # Store latest mesh (thread-safe)
            with self._lock:
                self._latest_mesh = mesh_data
                self._mesh_timestamp = mesh_data.timestamp
                self._last_update_time = time.time()
                self._mesh_count += 1
                self._block_count += len(blocks)
            
            processing_time = (time.time() - start_time) * 1000
            logger.info(
                f"Processed mesh: {len(blocks)} blocks, "
                f"{total_vertices} vertices, {total_triangles} triangles "
                f"in {processing_time:.1f}ms"
            )
            
        except Exception as e:
            logger.error(f"Error processing mesh: {e}")
            self._error_count += 1
    
    def _process_mesh_block(self, block: MeshBlock) -> Optional[SimplifiedMeshBlock]:
        """
        Convert a ROS mesh block to simplified format.
        
        Args:
            block: nvblox_msgs/MeshBlock from ROS
        
        Returns:
            Simplified mesh block or None if invalid
        """
        try:
            # Extract block index
            if hasattr(block, 'index'):
                index = (block.index.x, block.index.y, block.index.z)
            else:
                # Fallback for different message structures
                index = (0, 0, 0)
            
            # Extract vertices
            vertices = []
            if hasattr(block, 'vertices'):
                for vertex in block.vertices:
                    if hasattr(vertex, 'x'):
                        # Point32 or Point
                        vertices.append([float(vertex.x), float(vertex.y), float(vertex.z)])
                    else:
                        # Array format
                        vertices.append([float(vertex[0]), float(vertex[1]), float(vertex[2])])
            
            if not vertices:
                return None  # Skip empty blocks
            
            # Extract triangles (vertex indices)
            triangles = []
            if hasattr(block, 'triangles'):
                # Triangles are typically stored as flat array [i0, i1, i2, i3, i4, i5, ...]
                triangle_indices = block.triangles
                for i in range(0, len(triangle_indices), 3):
                    if i + 2 < len(triangle_indices):
                        triangles.append([
                            int(triangle_indices[i]),
                            int(triangle_indices[i + 1]),
                            int(triangle_indices[i + 2])
                        ])
            
            if not triangles:
                return None  # Skip blocks without triangles
            
            # Extract colors (optional)
            colors = None
            if self._enable_colors and hasattr(block, 'colors') and block.colors:
                colors = []
                for color in block.colors:
                    if hasattr(color, 'r'):
                        # ColorRGBA (float 0-1)
                        colors.append([
                            int(color.r * 255),
                            int(color.g * 255),
                            int(color.b * 255)
                        ])
                    else:
                        # Already in [r, g, b] format
                        colors.append([int(color[0]), int(color[1]), int(color[2])])
            
            return SimplifiedMeshBlock(
                index=index,
                vertices=vertices,
                triangles=triangles,
                colors=colors,
                timestamp=time.time(),
            )
            
        except Exception as e:
            logger.error(f"Error processing mesh block: {e}")
            return None
    
    # ========================================================================
    # Data Access (Thread-Safe)
    # ========================================================================
    
    def get_latest_mesh(self) -> Optional[Dict]:
        """
        Get the complete latest mesh data.
        
        Returns:
            Dictionary with mesh data suitable for JSON serialization,
            or None if no mesh received yet
        """
        with self._lock:
            if self._latest_mesh is None:
                return None
            return asdict(self._latest_mesh)
    
    def get_mesh_delta(self) -> Optional[Dict]:
        """
        Get only the mesh blocks that changed since last delta request.
        
        This enables efficient incremental updates for visualization.
        
        Returns:
            Dictionary with changed blocks and metadata, or None if no changes
        """
        with self._lock:
            if not self._changed_blocks:
                return None
            
            # Create delta update
            delta = {
                "changed_blocks": [asdict(block) for block in self._changed_blocks.values()],
                "timestamp": time.time(),
                "block_count": len(self._changed_blocks),
                "delta_since": self._last_delta_timestamp,
            }
            
            # Clear changed blocks and update timestamp
            self._changed_blocks.clear()
            self._last_delta_timestamp = time.time()
            
            return delta
    
    def get_mesh_timestamp(self) -> Optional[float]:
        """
        Get timestamp of the latest mesh update.
        
        Returns:
            Unix timestamp or None if no mesh received
        """
        with self._lock:
            return self._mesh_timestamp
    
    def get_mesh_summary(self) -> Dict:
        """
        Get a summary of the mesh without full data.
        
        Useful for checking if mesh is available and basic stats.
        
        Returns:
            Dictionary with mesh summary statistics
        """
        with self._lock:
            if self._latest_mesh is None:
                return {
                    "available": False,
                    "timestamp": None,
                    "block_count": 0,
                    "total_vertices": 0,
                    "total_triangles": 0,
                }
            
            return {
                "available": True,
                "timestamp": self._latest_mesh.timestamp,
                "block_count": len(self._latest_mesh.blocks),
                "total_vertices": self._latest_mesh.total_vertices,
                "total_triangles": self._latest_mesh.total_triangles,
                "block_size": self._latest_mesh.block_size,
                "frame_id": self._latest_mesh.frame_id,
            }
    
    def clear_mesh(self) -> None:
        """Clear the stored mesh data."""
        with self._lock:
            self._latest_mesh = None
            self._mesh_timestamp = None
            self._changed_blocks.clear()
        logger.info("Mesh data cleared")
    
    # ========================================================================
    # Status and Statistics
    # ========================================================================
    
    def get_status(self) -> Dict:
        """
        Get bridge status and statistics.
        
        Returns:
            Dictionary with status information
        """
        with self._lock:
            mesh_age = None
            if self._mesh_timestamp:
                mesh_age = time.time() - self._mesh_timestamp
            
            return {
                "running": self._running,
                "available": self._available,
                "mesh_count": self._mesh_count,
                "block_count": self._block_count,
                "error_count": self._error_count,
                "mesh_timestamp": self._mesh_timestamp,
                "mesh_age_seconds": mesh_age,
                "last_update_time": self._last_update_time,
                "changed_blocks_pending": len(self._changed_blocks),
                "has_mesh": self._latest_mesh is not None,
            }
    
    def is_available(self) -> bool:
        """Check if the bridge is available (dependencies met)."""
        return self._available
    
    def is_running(self) -> bool:
        """Check if the bridge is currently running."""
        return self._running


# ============================================================================
# Singleton Instance
# ============================================================================

_instance: Optional[MeshBridge] = None


def get_mesh_bridge() -> Optional[MeshBridge]:
    """
    Get the mesh bridge singleton instance.
    
    Returns:
        MeshBridge instance or None if not initialized
    """
    return _instance


def init_mesh_bridge(
    mesh_topic: str = "/nvblox_node/mesh",
    enable_colors: bool = True,
    max_blocks: int = 10000,
) -> Optional[MeshBridge]:
    """
    Initialize the mesh bridge singleton.
    
    Args:
        mesh_topic: ROS2 topic for nvblox mesh
        enable_colors: Whether to include color data
        max_blocks: Maximum number of blocks to store
    
    Returns:
        MeshBridge instance or None if initialization failed
    """
    global _instance
    
    if _instance is not None:
        logger.warning("MeshBridge already initialized, returning existing instance")
        return _instance
    
    _instance = MeshBridge(
        mesh_topic=mesh_topic,
        enable_colors=enable_colors,
        max_blocks=max_blocks,
    )
    
    return _instance


# ============================================================================
# Main - Standalone Testing
# ============================================================================

def main():
    """Standalone test runner for mesh bridge."""
    import argparse
    
    parser = argparse.ArgumentParser(description="NOMAD Mesh Bridge")
    parser.add_argument("--topic", default="/nvblox_node/mesh", help="Mesh topic")
    parser.add_argument("--no-colors", action="store_true", help="Disable color data")
    parser.add_argument("--max-blocks", type=int, default=10000, help="Max blocks")
    args = parser.parse_args()
    
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    
    # Create and start bridge
    bridge = MeshBridge(
        mesh_topic=args.topic,
        enable_colors=not args.no_colors,
        max_blocks=args.max_blocks,
    )
    
    if not bridge.start():
        logger.error("Failed to start mesh bridge")
        return
    
    logger.info("Mesh bridge running... Press Ctrl+C to stop")
    
    try:
        # Print status every 5 seconds
        while True:
            time.sleep(5)
            status = bridge.get_status()
            summary = bridge.get_mesh_summary()
            logger.info(f"Status: {status}")
            logger.info(f"Mesh: {summary}")
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()
