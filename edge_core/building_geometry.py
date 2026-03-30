"""
Building Geometry Extraction from NVBlox 3D Map.

Analyzes nvblox voxel data (stored in Edge Core state) to:
1. Identify planar surfaces (walls, roof, ground)
2. Extract building bounding box and dimensions
3. Label building faces by cardinal direction (N/S/E/W)
4. Compute target positions relative to building faces with
   decimeter precision (e.g., "3.3m above ground, 1.6m from western wall")

The nvblox mesh/voxel data is received via the ROS HTTP bridge and stored
in app.state.slam_mesh_data. This module processes that data snapshot.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger("edge_core.building_geometry")


@dataclass
class BuildingFace:
    """A detected face (wall/roof/ground) of the building."""
    name: str                 # e.g. "north", "south", "east", "west", "roof", "ground"
    normal: Tuple[float, float, float]   # outward-facing normal vector
    plane_offset: float       # signed distance from origin along normal
    # Bounding box of this face in world coordinates
    min_x: float = 0.0
    min_y: float = 0.0
    min_z: float = 0.0
    max_x: float = 0.0
    max_y: float = 0.0
    max_z: float = 0.0
    # Dimensions
    width_m: float = 0.0     # horizontal extent
    height_m: float = 0.0    # vertical extent (for walls) or depth (for roof)
    area_m2: float = 0.0


@dataclass
class BuildingGeometry:
    """Extracted building geometry from nvblox map."""
    # Axis-aligned bounding box of the building
    min_x: float = 0.0
    min_y: float = 0.0
    min_z: float = 0.0
    max_x: float = 0.0
    max_y: float = 0.0
    max_z: float = 0.0
    # Dimensions
    width_m: float = 0.0      # X extent (East-West in ROS frame)
    depth_m: float = 0.0      # Y extent (vertical in ROS frame, but depends on frame)
    height_m: float = 0.0     # Z extent (North-South in ROS frame)
    # Detected faces
    faces: Dict[str, BuildingFace] = field(default_factory=dict)
    # Ground plane Z value (lowest occupied voxels)
    ground_z: float = 0.0
    # Number of voxels used in analysis
    voxel_count: int = 0
    # Heading offset: the compass heading that aligns +X with East
    # Used to label N/S/E/W faces correctly based on drone heading
    heading_deg: float = 0.0

    def to_dict(self) -> dict:
        """Convert to JSON-serializable dict."""
        return {
            "bbox": {
                "min": [round(self.min_x, 2), round(self.min_y, 2), round(self.min_z, 2)],
                "max": [round(self.max_x, 2), round(self.max_y, 2), round(self.max_z, 2)],
            },
            "dimensions": {
                "width_m": round(self.width_m, 1),
                "depth_m": round(self.depth_m, 1),
                "height_m": round(self.height_m, 1),
            },
            "ground_z": round(self.ground_z, 2),
            "voxel_count": self.voxel_count,
            "heading_deg": round(self.heading_deg, 1),
            "faces": {
                name: {
                    "name": face.name,
                    "width_m": round(face.width_m, 1),
                    "height_m": round(face.height_m, 1),
                    "area_m2": round(face.area_m2, 1),
                    "bbox": {
                        "min": [round(face.min_x, 2), round(face.min_y, 2), round(face.min_z, 2)],
                        "max": [round(face.max_x, 2), round(face.max_y, 2), round(face.max_z, 2)],
                    },
                }
                for name, face in self.faces.items()
            },
        }


@dataclass
class TargetPlacement:
    """Position of a target relative to the building."""
    target_id: str            # "A", "B", "C", ...
    color: str                # "red", "blue", etc.
    # Absolute 3D position
    x: float
    y: float
    z: float
    # Which building face it's on (or "ground", "roof")
    face_name: str            # "north", "south", "east", "west", "roof", "ground"
    # Distance measurements relative to building
    height_above_ground_m: float = 0.0
    dist_from_left_m: float = 0.0   # distance from left edge when facing the wall
    dist_from_right_m: float = 0.0  # distance from right edge when facing the wall
    dist_from_top_m: float = 0.0    # distance from top of wall
    dist_from_bottom_m: float = 0.0 # distance from ground level
    # For ground/roof targets: distance from each wall
    dist_from_building_m: float = 0.0  # if on ground, distance from nearest wall
    nearest_landmark: str = ""  # e.g. "0.2m left of the door"

    def describe(self) -> str:
        """Generate a competition-format text description."""
        color_cap = self.color.capitalize() if self.color else "Unknown"

        if self.face_name in ("north", "south", "east", "west"):
            # On a wall
            desc = (
                f"Target {self.target_id} is on the {self.face_name} face of the building, "
                f"{self.height_above_ground_m:.1f}m above ground"
            )
            # Add horizontal reference
            # When facing a wall, "left" and "right" depend on which face
            left_label, right_label = _face_lr_labels(self.face_name)
            if self.dist_from_left_m <= self.dist_from_right_m:
                desc += f" and {self.dist_from_left_m:.1f}m from the {left_label} wall"
            else:
                desc += f" and {self.dist_from_right_m:.1f}m from the {right_label} wall"
            desc += f". The colour is {color_cap.lower()}."
            return desc

        elif self.face_name == "roof":
            desc = (
                f"Target {self.target_id} is on the roof of the building"
            )
            # Report distance from nearest edge
            min_edge = min(self.dist_from_left_m, self.dist_from_right_m)
            if min_edge > 0:
                desc += f", approximately {min_edge:.1f}m from the nearest edge"
            desc += f". The colour is {color_cap.lower()}."
            return desc

        else:
            # On the ground
            desc = (
                f"Target {self.target_id} is on the ground, "
                f"{self.dist_from_building_m:.1f}m away from the {self.face_name} face of the building"
            )
            if self.nearest_landmark:
                desc += f" and {self.nearest_landmark}"
            desc += f". The colour is {color_cap.lower()}."
            return desc


def _face_lr_labels(face_name: str) -> Tuple[str, str]:
    """
    Get (left_label, right_label) when facing a wall from outside.

    When facing north wall from outside (looking south):
      left = east edge, right = west edge
    When facing south wall from outside (looking north):
      left = west edge, right = east edge
    When facing east wall from outside (looking west):
      left = north edge, right = south edge
    When facing west wall from outside (looking east):
      left = south edge, right = north edge
    """
    mapping = {
        "north": ("eastern", "western"),
        "south": ("western", "eastern"),
        "east":  ("northern", "southern"),
        "west":  ("southern", "northern"),
    }
    return mapping.get(face_name, ("left", "right"))


def extract_building_geometry(
    mesh_data: dict,
    heading_deg: float = 0.0,
    voxel_size: float = 0.05,
    ground_percentile: float = 5.0,
) -> Optional[BuildingGeometry]:
    """
    Extract building geometry from nvblox mesh/voxel data.

    The nvblox data is in the ROS odom frame (camera optical convention):
    - X: right
    - Y: down
    - Z: forward

    We use the drone's compass heading to map these to cardinal directions.

    Args:
        mesh_data: The slam_mesh_data dict from Edge Core state.
                   Contains either "voxels" (voxel mode) or "blocks" (block mode).
        heading_deg: Drone compass heading (0=North, 90=East). Used to orient
                     the building faces to cardinal directions.
        voxel_size: Size of each voxel in meters (default 0.05 = 5cm).
        ground_percentile: Percentile of lowest Z values to use as ground plane.

    Returns:
        BuildingGeometry or None if insufficient data.
    """
    if not mesh_data:
        return None

    mesh = mesh_data.get("mesh", mesh_data)

    # Extract voxel positions
    positions = []
    mode = mesh.get("mode", "")

    if mode == "voxel" and "voxels" in mesh:
        for v in mesh["voxels"]:
            p = v.get("p")
            if p and len(p) >= 3:
                positions.append(p)
        voxel_size = mesh.get("voxel_size", voxel_size)

    elif mode == "block" and "blocks" in mesh:
        block_size = mesh.get("block_size", 0.2)
        for b in mesh["blocks"]:
            idx = b.get("index")
            if idx and len(idx) >= 3:
                # Convert block index to world position
                positions.append([
                    idx[0] * block_size,
                    idx[1] * block_size,
                    idx[2] * block_size,
                ])
        voxel_size = block_size
    else:
        logger.warning("No valid voxel/block data in mesh")
        return None

    if len(positions) < 20:
        logger.warning(f"Insufficient voxels ({len(positions)}) for geometry extraction")
        return None

    pts = np.array(positions, dtype=np.float32)

    # NVBlox ROS frame: X-right, Y-down, Z-forward
    # Compute bounding box
    min_xyz = pts.min(axis=0)
    max_xyz = pts.max(axis=0)

    # Ground plane: lowest Y values (Y-down in ROS camera frame)
    # Actually in ROS odom frame Y is down for ZED camera convention
    # The "ground" is the highest Y value (most positive = most downward)
    # But this depends on the frame. Let's use the most common Y values
    # as ground reference.
    y_sorted = np.sort(pts[:, 1])
    ground_y = np.percentile(y_sorted, 100 - ground_percentile)  # highest Y = ground

    # Building occupied region: main cluster of voxels
    # Use the bounding box of the central 90% of voxels to avoid outliers
    p5 = np.percentile(pts, 5, axis=0)
    p95 = np.percentile(pts, 95, axis=0)

    geom = BuildingGeometry(
        min_x=float(p5[0]),
        min_y=float(p5[1]),
        min_z=float(p5[2]),
        max_x=float(p95[0]),
        max_y=float(p95[1]),
        max_z=float(p95[2]),
        width_m=float(p95[0] - p5[0]),    # X extent
        depth_m=float(p95[1] - p5[1]),     # Y extent (vertical)
        height_m=float(p95[2] - p5[2]),    # Z extent (depth/forward)
        ground_z=float(ground_y),
        voxel_count=len(positions),
        heading_deg=heading_deg,
    )

    # Map ROS axes to cardinal directions using drone heading.
    # Heading 0 = North means the drone faces +Z = North.
    # ROS odom frame with ZED: +X = right, +Y = down, +Z = forward.
    # If heading=0 (North), then +Z = North, +X = East
    # If heading=90 (East), then +Z = East, +X = South
    #
    # We label building faces based on which direction they face outward.
    heading_rad = math.radians(heading_deg)

    # The building's min/max in X and Z define 4 vertical faces.
    # Y (vertical) defines roof (min_y = top) and ground (max_y = bottom).

    # Vertical extent (wall height)
    wall_height = geom.depth_m  # Y extent = vertical height

    # Create faces
    # North face: face with outward normal pointing in the North direction
    # With heading=0: North = +Z, so north face is at max_z
    # With heading=90: North = -X, so north face is at min_x

    # Simplification: label faces relative to the drone's forward direction
    # and use heading to assign N/S/E/W labels.

    # Forward face (away from drone, +Z in odom) and back face (-Z)
    # Right face (+X) and left face (-X)
    # Top face (-Y, lowest Y) and bottom/ground (+Y, highest Y)

    # Map forward/back/left/right to N/S/E/W using heading
    # heading=0: forward=N, right=E, back=S, left=W
    # heading=90: forward=E, right=S, back=W, left=N

    directions = _heading_to_cardinal(heading_deg)

    faces = {}

    # Forward face (+Z direction outward)
    fwd_name = directions["forward"]
    faces[fwd_name] = BuildingFace(
        name=fwd_name,
        normal=(0, 0, 1),
        plane_offset=float(p95[2]),
        min_x=float(p5[0]), max_x=float(p95[0]),
        min_y=float(p5[1]), max_y=float(p95[1]),
        min_z=float(p95[2]), max_z=float(p95[2]),
        width_m=geom.width_m,
        height_m=wall_height,
        area_m2=geom.width_m * wall_height,
    )

    # Back face (-Z direction outward)
    back_name = directions["back"]
    faces[back_name] = BuildingFace(
        name=back_name,
        normal=(0, 0, -1),
        plane_offset=float(p5[2]),
        min_x=float(p5[0]), max_x=float(p95[0]),
        min_y=float(p5[1]), max_y=float(p95[1]),
        min_z=float(p5[2]), max_z=float(p5[2]),
        width_m=geom.width_m,
        height_m=wall_height,
        area_m2=geom.width_m * wall_height,
    )

    # Right face (+X direction outward)
    right_name = directions["right"]
    faces[right_name] = BuildingFace(
        name=right_name,
        normal=(1, 0, 0),
        plane_offset=float(p95[0]),
        min_x=float(p95[0]), max_x=float(p95[0]),
        min_y=float(p5[1]), max_y=float(p95[1]),
        min_z=float(p5[2]), max_z=float(p95[2]),
        width_m=geom.height_m,  # Z extent for this face
        height_m=wall_height,
        area_m2=geom.height_m * wall_height,
    )

    # Left face (-X direction outward)
    left_name = directions["left"]
    faces[left_name] = BuildingFace(
        name=left_name,
        normal=(-1, 0, 0),
        plane_offset=float(p5[0]),
        min_x=float(p5[0]), max_x=float(p5[0]),
        min_y=float(p5[1]), max_y=float(p95[1]),
        min_z=float(p5[2]), max_z=float(p95[2]),
        width_m=geom.height_m,
        height_m=wall_height,
        area_m2=geom.height_m * wall_height,
    )

    # Roof face (-Y direction outward, top)
    faces["roof"] = BuildingFace(
        name="roof",
        normal=(0, -1, 0),
        plane_offset=float(p5[1]),
        min_x=float(p5[0]), max_x=float(p95[0]),
        min_y=float(p5[1]), max_y=float(p5[1]),
        min_z=float(p5[2]), max_z=float(p95[2]),
        width_m=geom.width_m,
        height_m=geom.height_m,
        area_m2=geom.width_m * geom.height_m,
    )

    # Ground face (+Y direction, bottom)
    faces["ground"] = BuildingFace(
        name="ground",
        normal=(0, 1, 0),
        plane_offset=float(p95[1]),
        min_x=float(p5[0]), max_x=float(p95[0]),
        min_y=float(p95[1]), max_y=float(p95[1]),
        min_z=float(p5[2]), max_z=float(p95[2]),
        width_m=geom.width_m,
        height_m=geom.height_m,
        area_m2=geom.width_m * geom.height_m,
    )

    geom.faces = faces
    return geom


def _heading_to_cardinal(heading_deg: float) -> Dict[str, str]:
    """
    Map relative directions (forward/back/left/right) to cardinal directions
    based on compass heading.

    In the ROS odom frame with ZED:
    - +Z = forward (direction drone faces)
    - +X = right
    - +Y = down

    Args:
        heading_deg: Compass heading (0=N, 90=E, 180=S, 270=W)

    Returns:
        Dict mapping "forward", "back", "left", "right" to cardinal names
    """
    # Quantize heading to nearest 90 degrees
    h = heading_deg % 360
    quadrant = round(h / 90) % 4

    # quadrant 0 (N):   forward=N, right=E, back=S, left=W
    # quadrant 1 (E):   forward=E, right=S, back=W, left=N
    # quadrant 2 (S):   forward=S, right=W, back=N, left=E
    # quadrant 3 (W):   forward=W, right=N, back=E, left=S

    mappings = [
        {"forward": "north", "right": "east",  "back": "south", "left": "west"},
        {"forward": "east",  "right": "south", "back": "west",  "left": "north"},
        {"forward": "south", "right": "west",  "back": "north", "left": "east"},
        {"forward": "west",  "right": "north", "back": "east",  "left": "south"},
    ]

    return mappings[quadrant]


def locate_target_on_building(
    target_x: float,
    target_y: float,
    target_z: float,
    building: BuildingGeometry,
    proximity_threshold: float = 1.0,
) -> Optional[str]:
    """
    Determine which building face a target is on or near.

    Args:
        target_x, target_y, target_z: Target position in odom frame
        building: Extracted building geometry
        proximity_threshold: Max distance from face to count as "on" it (meters)

    Returns:
        Face name (e.g. "north", "roof", "ground") or None if not near building
    """
    best_face = None
    best_dist = float("inf")

    for face_name, face in building.faces.items():
        # Distance from target to face plane
        nx, ny, nz = face.normal
        # Point on plane
        if face_name in ("roof",):
            plane_pt = (face.min_x, face.min_y, face.min_z)
        elif face_name in ("ground",):
            plane_pt = (face.min_x, face.max_y, face.min_z)
        else:
            # Use plane_offset along normal
            plane_pt = (
                face.min_x if nx <= 0 else face.max_x,
                face.min_y if ny <= 0 else face.max_y,
                face.min_z if nz <= 0 else face.max_z,
            )

        # Signed distance from point to plane
        dist = abs(
            nx * (target_x - plane_pt[0])
            + ny * (target_y - plane_pt[1])
            + nz * (target_z - plane_pt[2])
        )

        if dist < best_dist:
            best_dist = dist
            best_face = face_name

    if best_dist > proximity_threshold:
        # Target is far from building - it's "on the ground near the building"
        # Find which face it's closest to horizontally
        return _nearest_face_horizontal(target_x, target_z, building)

    return best_face


def _nearest_face_horizontal(
    target_x: float,
    target_z: float,
    building: BuildingGeometry,
) -> str:
    """Find the nearest vertical face based on horizontal distance."""
    # Check distance to each vertical face (ignore roof/ground)
    min_dist = float("inf")
    nearest = "ground"

    for face_name, face in building.faces.items():
        if face_name in ("roof", "ground"):
            continue
        nx, ny, nz = face.normal
        if nx != 0:
            # X-aligned face: distance in X
            face_x = face.max_x if nx > 0 else face.min_x
            dist = abs(target_x - face_x)
        elif nz != 0:
            face_z = face.max_z if nz > 0 else face.min_z
            dist = abs(target_z - face_z)
        else:
            continue

        if dist < min_dist:
            min_dist = dist
            nearest = face_name

    return nearest


def compute_target_placement(
    target_x: float,
    target_y: float,
    target_z: float,
    target_color: str,
    target_id: str,
    building: BuildingGeometry,
) -> TargetPlacement:
    """
    Compute a target's position relative to the building.

    Args:
        target_x, target_y, target_z: Position in odom frame
        target_color: Color name (e.g. "red")
        target_id: Letter identifier (e.g. "A")
        building: Extracted building geometry

    Returns:
        TargetPlacement with all relative measurements
    """
    face_name = locate_target_on_building(
        target_x, target_y, target_z, building
    )

    if face_name is None:
        face_name = "ground"

    # Height above ground (ground_z is max Y in camera-down frame)
    # Y-down: ground is at building.ground_z (highest Y), ceiling at min_y
    height_above_ground = building.ground_z - target_y
    height_above_ground = max(0.0, height_above_ground)

    placement = TargetPlacement(
        target_id=target_id,
        color=target_color,
        x=target_x,
        y=target_y,
        z=target_z,
        face_name=face_name,
        height_above_ground_m=round(height_above_ground, 1),
    )

    face = building.faces.get(face_name)
    if not face:
        return placement

    if face_name in ("north", "south", "east", "west"):
        # Wall target - compute horizontal position on the face
        nx, _, nz = face.normal

        if abs(nx) > 0:
            # X-normal face (east/west): horizontal extent is along Z
            face_min_h = face.min_z
            face_max_h = face.max_z
            target_h = target_z
        else:
            # Z-normal face (north/south): horizontal extent is along X
            face_min_h = face.min_x
            face_max_h = face.max_x
            target_h = target_x

        dist_from_min = target_h - face_min_h
        dist_from_max = face_max_h - target_h

        # Map min/max to left/right when facing the wall from outside
        # This depends on which face and our _face_lr_labels convention
        placement.dist_from_left_m = round(max(0.0, dist_from_min), 1)
        placement.dist_from_right_m = round(max(0.0, dist_from_max), 1)
        placement.dist_from_top_m = round(max(0.0, target_y - face.min_y), 1)
        placement.dist_from_bottom_m = round(height_above_ground, 1)

    elif face_name == "roof":
        # Roof target - distance from edges
        placement.dist_from_left_m = round(max(0.0, target_x - face.min_x), 1)
        placement.dist_from_right_m = round(max(0.0, face.max_x - target_x), 1)

    else:
        # Ground target - distance from nearest wall
        min_wall_dist = float("inf")
        nearest_wall = ""
        for wname, wface in building.faces.items():
            if wname in ("roof", "ground"):
                continue
            wnx, _, wnz = wface.normal
            if abs(wnx) > 0:
                wall_coord = wface.max_x if wnx > 0 else wface.min_x
                dist = abs(target_x - wall_coord)
            else:
                wall_coord = wface.max_z if wnz > 0 else wface.min_z
                dist = abs(target_z - wall_coord)

            if dist < min_wall_dist:
                min_wall_dist = dist
                nearest_wall = wname

        placement.dist_from_building_m = round(max(0.0, min_wall_dist), 1)
        placement.face_name = nearest_wall  # "on the ground, Xm from the [face] face"

    return placement


def generate_target_descriptions(
    targets: List[dict],
    building: BuildingGeometry,
) -> List[str]:
    """
    Generate competition-format descriptions for all detected targets.

    Args:
        targets: List of target dicts with keys: x, y, z, color (or label)
        building: Extracted building geometry

    Returns:
        List of description strings in competition format
    """
    descriptions = []

    for i, target in enumerate(targets):
        target_id = chr(ord('A') + i) if i < 26 else str(i)

        # Extract color from label (e.g. "red_circle" -> "red") or color field
        color = target.get("color", "")
        if not color:
            label = target.get("label", "")
            color = label.replace("_circle", "").lower() if label else "unknown"

        x = target.get("x", 0.0)
        y = target.get("y", 0.0)
        z = target.get("z", 0.0)

        if x is None or y is None or z is None:
            continue

        placement = compute_target_placement(
            float(x), float(y), float(z),
            color, target_id, building,
        )
        descriptions.append(placement.describe())

    return descriptions
