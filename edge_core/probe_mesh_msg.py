#!/usr/bin/env python3
"""
Diagnostic: probe the actual nvblox_msgs/Mesh message structure.

Run inside the Isaac ROS container:
    python3 /workspaces/isaac_ros-dev/scripts/dev/probe_mesh_msg.py

Prints the exact field names, types, and shapes of the Mesh and MeshBlock
messages so we can fix the ros_http_bridge parsing.
"""

import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
except ImportError:
    print("ERROR: rclpy not available. Run inside Isaac ROS container.")
    sys.exit(1)

try:
    from nvblox_msgs.msg import Mesh, MeshBlock
    print("OK: nvblox_msgs.msg imported (Mesh, MeshBlock)")
except ImportError:
    print("ERROR: nvblox_msgs not available. Build with: colcon build --packages-up-to nvblox_msgs")
    sys.exit(1)


# Print the message definition
print("\n=== Mesh message fields ===")
for slot in Mesh.__slots__:
    print(f"  Mesh.{slot} -> {getattr(Mesh, slot, '?')}")

print("\n=== MeshBlock message fields ===")
for slot in MeshBlock.__slots__:
    print(f"  MeshBlock.{slot} -> {getattr(MeshBlock, slot, '?')}")

# Also check via get_fields_and_field_types if available
if hasattr(Mesh, 'get_fields_and_field_types'):
    print("\n=== Mesh.get_fields_and_field_types() ===")
    for name, ftype in Mesh.get_fields_and_field_types().items():
        print(f"  {name}: {ftype}")

if hasattr(MeshBlock, 'get_fields_and_field_types'):
    print("\n=== MeshBlock.get_fields_and_field_types() ===")
    for name, ftype in MeshBlock.get_fields_and_field_types().items():
        print(f"  {name}: {ftype}")


class MeshProbe(Node):
    def __init__(self):
        super().__init__("mesh_probe")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Mesh, "/nvblox_node/mesh", self.cb, qos)
        self.got_msg = False
        self.get_logger().info("Waiting for /nvblox_node/mesh ...")

    def cb(self, msg):
        if self.got_msg:
            return
        self.got_msg = True
        self.get_logger().info("=== GOT MESH MESSAGE ===")

        # Top-level fields
        print(f"\nheader.frame_id = {msg.header.frame_id}")
        print(f"header.stamp = {msg.header.stamp}")
        for attr in ['block_size', 'block_size_m', 'clear']:
            if hasattr(msg, attr):
                print(f"msg.{attr} = {getattr(msg, attr)}")

        n_blocks = len(msg.blocks) if hasattr(msg, 'blocks') else 0
        n_indices = len(msg.block_indices) if hasattr(msg, 'block_indices') else 0
        print(f"\nlen(msg.blocks) = {n_blocks}")
        print(f"len(msg.block_indices) = {n_indices}")

        # Block indices
        if n_indices > 0:
            idx0 = msg.block_indices[0]
            print(f"\nblock_indices[0] type = {type(idx0).__name__}")
            print(f"block_indices[0] = {idx0}")
            if hasattr(idx0, 'x'):
                print(f"  .x={idx0.x}  .y={idx0.y}  .z={idx0.z}")

        # Probe first non-empty block
        for bi, blk in enumerate(msg.blocks[:10]):
            vlen = len(blk.vertices) if hasattr(blk, 'vertices') else -1
            tlen = len(blk.triangles) if hasattr(blk, 'triangles') else -1
            clen = len(blk.colors) if hasattr(blk, 'colors') else -1
            nlen = len(blk.normals) if hasattr(blk, 'normals') else -1

            if vlen <= 0:
                continue

            print(f"\n=== block[{bi}] (first non-empty) ===")
            print(f"  len(vertices)  = {vlen}")
            print(f"  len(triangles) = {tlen}")
            print(f"  len(colors)    = {clen}")
            print(f"  len(normals)   = {nlen}")

            # Check block index
            if hasattr(blk, 'index'):
                idx = blk.index
                print(f"  block.index = {idx}")
                if hasattr(idx, 'x'):
                    print(f"    .x={idx.x}  .y={idx.y}  .z={idx.z}")

            # Probe vertex format
            v0 = blk.vertices[0]
            print(f"\n  vertices[0] type  = {type(v0).__name__}")
            print(f"  vertices[0] value = {v0}")
            print(f"  hasattr(v0, 'x')  = {hasattr(v0, 'x')}")
            if hasattr(v0, 'x'):
                print(f"    Point32: x={v0.x}, y={v0.y}, z={v0.z}")
                if vlen >= 2:
                    v1 = blk.vertices[1]
                    print(f"  vertices[1]: x={v1.x}, y={v1.y}, z={v1.z}")
            else:
                # Flat array: show first 9 values (3 vertices)
                sample = [float(blk.vertices[i]) for i in range(min(9, vlen))]
                print(f"  Flat array, first 9 values = {sample}")
                n_verts = vlen // 3
                print(f"  Deduced vertex count = {n_verts}")

            # Probe triangle format
            if tlen > 0:
                t0 = blk.triangles[0]
                print(f"\n  triangles[0] type  = {type(t0).__name__}")
                print(f"  triangles[0] value = {t0}")
                sample_t = [int(blk.triangles[i]) for i in range(min(9, tlen))]
                print(f"  First 9 triangle indices = {sample_t}")
                n_tris = tlen // 3
                print(f"  Deduced triangle count = {n_tris}")

            # Probe color format
            if clen > 0:
                c0 = blk.colors[0]
                print(f"\n  colors[0] type  = {type(c0).__name__}")
                print(f"  colors[0] value = {c0}")
                print(f"  hasattr(c0, 'r') = {hasattr(c0, 'r')}")
                if hasattr(c0, 'r'):
                    print(f"    ColorRGBA: r={c0.r}, g={c0.g}, b={c0.b}, a={c0.a}")
                else:
                    sample_c = [float(blk.colors[i]) for i in range(min(8, clen))]
                    print(f"  Flat array, first 8 values = {sample_c}")
                    if all(0.0 <= v <= 1.0 for v in sample_c):
                        print(f"  Values 0-1 range -> RGBA stride 4, vertex count = {clen // 4}")
                    else:
                        print(f"  Values > 1 -> probably 0-255 RGB stride 3, vertex count = {clen // 3}")

            break  # Only probe first non-empty block

        print("\n=== PROBE COMPLETE ===")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = MeshProbe()

    start = time.time()
    timeout = 15.0
    print(f"\nListening for {timeout}s...")

    while rclpy.ok() and not node.got_msg:
        rclpy.spin_once(node, timeout_sec=0.5)
        if time.time() - start > timeout:
            print(f"\nTIMEOUT: No mesh message received in {timeout}s")
            print("Check that nvblox is running: ros2 topic list | grep nvblox")
            print("Check mesh rate: ros2 topic hz /nvblox_node/mesh")
            break

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
