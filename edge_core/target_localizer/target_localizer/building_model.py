"""
building_model.py

Maintains a geometric model of the target building for AEAC 2026 Task 1.
Handles:
  - Building face determination from drone pose
  - Auto-registered landmarks (doors, windows) from YOLO detections
  - Relative position computation for target descriptions
  - Corner naming and face labeling

The building is modeled as a rectangular prism defined by:
  - Center GPS coordinate (lat, lon)
  - Orientation angle (degrees clockwise from north, along the long axis)
  - Length and width (meters)
  - Height (meters, optional, used for roofline references)

Faces are labeled by their outward-facing cardinal direction (N/S/E/W)
relative to the building's oriented geometry.
"""

import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
from enum import Enum


# ------------------------------------------------------------------ #
#  GPS helpers
# ------------------------------------------------------------------ #
EARTH_RADIUS = 6_371_000.0  # meters


def gps_to_local(lat: float, lon: float,
                 ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """Convert GPS (lat, lon) to local ENU (east, north) in meters
    relative to a reference point."""
    d_lat = math.radians(lat - ref_lat)
    d_lon = math.radians(lon - ref_lon)
    north = d_lat * EARTH_RADIUS
    east = d_lon * EARTH_RADIUS * math.cos(math.radians(ref_lat))
    return east, north


def local_to_gps(east: float, north: float,
                 ref_lat: float, ref_lon: float) -> Tuple[float, float]:
    """Convert local ENU back to GPS."""
    lat = ref_lat + math.degrees(north / EARTH_RADIUS)
    lon = ref_lon + math.degrees(east / (EARTH_RADIUS * math.cos(math.radians(ref_lat))))
    return lat, lon


# ------------------------------------------------------------------ #
#  Data classes
# ------------------------------------------------------------------ #
class FaceID(Enum):
    """Canonical face labels based on outward normal direction."""
    NORTH = "north"
    SOUTH = "south"
    EAST = "east"
    WEST = "west"
    ROOF = "roof"
    GROUND = "ground"


@dataclass
class Landmark:
    """A detected landmark on a building face."""
    kind: str                    # "door", "window", "vent", etc.
    face: FaceID                 # which face it's on
    horiz_from_left: float       # meters from the left corner (viewer facing wall)
    height_agl: float            # meters above ground level
    confidence: float = 1.0      # detection confidence [0, 1]
    detections: int = 1          # how many times seen (for averaging)

    @property
    def description_name(self) -> str:
        return f"the {self.kind}"


@dataclass
class Corner:
    """A named corner of the building."""
    name: str                    # e.g. "northwest corner"
    east: float                  # local ENU east
    north: float                 # local ENU north


@dataclass
class Face:
    """One face (wall) of the building."""
    face_id: FaceID
    # The two corners, ordered left-to-right when viewed from outside
    corner_left: Corner
    corner_right: Corner
    width: float                 # face width in meters
    outward_normal_deg: float    # heading of outward normal (deg CW from north)
    landmarks: List[Landmark] = field(default_factory=list)

    def add_landmark(self, lm: Landmark, merge_radius: float = 0.5):
        """Add a landmark, merging with existing if within merge_radius."""
        for existing in self.landmarks:
            if existing.kind != lm.kind:
                continue
            dh = abs(existing.horiz_from_left - lm.horiz_from_left)
            dv = abs(existing.height_agl - lm.height_agl)
            if dh < merge_radius and dv < merge_radius:
                # Running average
                n = existing.detections
                existing.horiz_from_left = (existing.horiz_from_left * n + lm.horiz_from_left) / (n + 1)
                existing.height_agl = (existing.height_agl * n + lm.height_agl) / (n + 1)
                existing.confidence = max(existing.confidence, lm.confidence)
                existing.detections += 1
                return
        self.landmarks.append(lm)


# ------------------------------------------------------------------ #
#  Building Model
# ------------------------------------------------------------------ #
class BuildingModel:
    """
    Rectangular building model for target localization.

    Coordinate convention:
      - All internal math is in local ENU (east, north, up)
      - The building center is the origin
      - orientation_deg is the heading of the building's long axis,
        measured clockwise from north (e.g., 0 = long axis points north)
    """

    def __init__(self,
                 center_lat: float,
                 center_lon: float,
                 length: float,
                 width: float,
                 height: float,
                 orientation_deg: float = 0.0):
        """
        Args:
            center_lat/lon: GPS of building center
            length: dimension along the orientation axis (meters)
            width: dimension perpendicular to orientation (meters)
            height: building height in meters
            orientation_deg: heading of the long axis (deg CW from north)
        """
        self.center_lat = center_lat
        self.center_lon = center_lon
        self.length = length
        self.width = width
        self.height = height
        self.orientation_deg = orientation_deg

        # Precompute orientation in radians (math convention: CCW from east)
        # Convert from CW-from-north to CCW-from-east
        self.orientation_rad = math.radians(90.0 - orientation_deg)

        # Unit vectors along building axes (in ENU)
        # long_axis: direction of the building's length
        # short_axis: direction of the building's width (90 deg CCW from long)
        self.long_axis = np.array([
            math.cos(self.orientation_rad),
            math.sin(self.orientation_rad)
        ])
        self.short_axis = np.array([
            -math.sin(self.orientation_rad),
            math.cos(self.orientation_rad)
        ])

        # Build corners and faces
        self._build_geometry()

    def _build_geometry(self):
        """Compute corners and faces from building parameters."""
        half_l = self.length / 2.0
        half_w = self.width / 2.0

        # Four corners in local ENU relative to building center
        # Named by their approximate cardinal direction
        # Corner positions:
        #   long_axis points roughly "forward" (along orientation)
        #   short_axis points roughly "left" when facing along long_axis
        corner_positions = {
            'FL': self.long_axis * half_l + self.short_axis * half_w,   # forward-left
            'FR': self.long_axis * half_l - self.short_axis * half_w,   # forward-right
            'BL': -self.long_axis * half_l + self.short_axis * half_w,  # back-left
            'BR': -self.long_axis * half_l - self.short_axis * half_w,  # back-right
        }

        # Determine cardinal labels for corners based on their actual position
        self.corners = {}
        for key, pos in corner_positions.items():
            label = self._cardinal_label_for_corner(pos)
            self.corners[key] = Corner(name=f"{label} corner", east=pos[0], north=pos[1])

        # Build four faces
        # Each face is defined by two corners (left-to-right when viewed from outside)
        # and an outward normal

        # Forward face (along +long_axis direction)
        fwd_normal_deg = self.orientation_deg  # outward normal = along long axis heading
        # Backward face
        bwd_normal_deg = (self.orientation_deg + 180) % 360
        # Right face (along -short_axis)
        right_normal_deg = (self.orientation_deg + 90) % 360
        # Left face (along +short_axis)
        left_normal_deg = (self.orientation_deg + 270) % 360

        face_defs = [
            # (normal_deg, left_corner_key, right_corner_key, width)
            (fwd_normal_deg, 'FR', 'FL', self.width),    # forward face
            (bwd_normal_deg, 'BL', 'BR', self.width),    # backward face
            (right_normal_deg, 'FR', 'BR', self.length),  # right face (swap for viewer perspective)
            (left_normal_deg, 'BL', 'FL', self.length),   # left face
        ]

        # Fix corner ordering: when viewed from outside, "left" should be
        # the corner that is to the viewer's left. The viewer faces opposite
        # to the outward normal.
        self.faces = {}
        for normal_deg, lk, rk, w in face_defs:
            face_id = self._cardinal_face_id(normal_deg)
            # Swap left/right based on cross product with normal to ensure
            # correct viewer-left / viewer-right ordering
            cl = self.corners[lk]
            cr = self.corners[rk]
            viewer_right = self._viewer_right_direction(normal_deg)
            cl_pos = np.array([cl.east, cl.north])
            cr_pos = np.array([cr.east, cr.north])
            if np.dot(cr_pos - cl_pos, viewer_right) < 0:
                cl, cr = cr, cl
            self.faces[face_id] = Face(
                face_id=face_id,
                corner_left=cl,
                corner_right=cr,
                width=w,
                outward_normal_deg=normal_deg,
            )

    def _viewer_right_direction(self, normal_deg: float) -> np.ndarray:
        """When a viewer faces opposite the outward normal, their 'right' direction."""
        # Viewer faces heading = (normal_deg + 180) % 360
        # Viewer's right = 90 deg CW from their facing direction
        right_heading = (normal_deg + 180 + 90) % 360
        right_rad = math.radians(90.0 - right_heading)
        return np.array([math.cos(right_rad), math.sin(right_rad)])

    def _cardinal_face_id(self, normal_deg: float) -> FaceID:
        """Assign a cardinal face ID based on outward normal heading."""
        # Normalize to [0, 360)
        h = normal_deg % 360
        if 315 <= h or h < 45:
            return FaceID.NORTH
        elif 45 <= h < 135:
            return FaceID.EAST
        elif 135 <= h < 225:
            return FaceID.SOUTH
        else:
            return FaceID.WEST

    def _cardinal_label_for_corner(self, pos: np.ndarray) -> str:
        """Label a corner by its approximate cardinal direction from center."""
        ns = "north" if pos[1] >= 0 else "south"
        ew = "east" if pos[0] >= 0 else "west"
        return f"{ns}{ew}"

    # ------------------------------------------------------------ #
    #  Drone-relative queries
    # ------------------------------------------------------------ #
    def get_face_from_drone_pose(self,
                                 drone_lat: float,
                                 drone_lon: float,
                                 drone_heading_deg: float) -> Optional[Face]:
        """
        Determine which building face the drone is looking at based on
        drone GPS position and heading.

        Returns the face whose outward normal most closely opposes
        the drone's viewing direction, provided the drone is roughly
        in front of that face.
        """
        drone_east, drone_north = gps_to_local(
            drone_lat, drone_lon, self.center_lat, self.center_lon
        )

        # Vector from building center to drone
        to_drone = np.array([drone_east, drone_north])

        best_face = None
        best_score = -999.0

        for face in self.faces.values():
            # Outward normal direction as unit vector
            n_rad = math.radians(90.0 - face.outward_normal_deg)
            normal_vec = np.array([math.cos(n_rad), math.sin(n_rad)])

            # Score 1: drone is on the outward-normal side of this face
            side_score = np.dot(to_drone, normal_vec)
            if side_score < 0:
                continue  # drone is behind this face

            # Score 2: drone heading opposes the face normal
            # (drone looks toward the face)
            drone_look_rad = math.radians(90.0 - drone_heading_deg)
            drone_look = np.array([math.cos(drone_look_rad), math.sin(drone_look_rad)])
            alignment = -np.dot(drone_look, normal_vec)  # want to face INTO the wall

            combined = alignment * 2.0 + side_score * 0.5
            if combined > best_score:
                best_score = combined
                best_face = face

        return best_face

    def project_point_onto_face(self,
                                 point_east: float,
                                 point_north: float,
                                 point_up: float,
                                 face: Face) -> Tuple[float, float]:
        """
        Project a 3D point onto a building face.

        Returns:
            (horiz_from_left, height_agl): position on the face
            horiz_from_left is distance from the left corner when viewed
            from outside, in meters.
            height_agl is height above ground in meters.
        """
        # Left corner position
        cl = np.array([face.corner_left.east, face.corner_left.north])
        cr = np.array([face.corner_right.east, face.corner_right.north])

        # Horizontal axis along the face (left to right from viewer's perspective)
        face_horiz = cr - cl
        face_length = np.linalg.norm(face_horiz)
        if face_length < 0.01:
            return 0.0, point_up
        face_horiz_unit = face_horiz / face_length

        # Project point onto face horizontal axis
        point_2d = np.array([point_east, point_north])
        offset = point_2d - cl
        horiz_from_left = float(np.dot(offset, face_horiz_unit))

        # Clamp to face width
        horiz_from_left = max(0.0, min(horiz_from_left, face_length))

        return horiz_from_left, point_up

    def find_nearest_reference(self,
                                face: Face,
                                horiz_from_left: float,
                                height_agl: float) -> Tuple[str, float, str]:
        """
        Find the nearest reference point (landmark or corner) to describe
        a position on a face.

        Returns:
            (ref_name, distance, direction_phrase)
            e.g. ("the door", 1.2, "1.2m to the right of the door")
        """
        candidates = []

        # Corners as references
        candidates.append({
            'name': face.corner_left.name,
            'horiz': 0.0,
            'height': 0.0,
            'dist_h': abs(horiz_from_left),
            'is_corner': True
        })
        candidates.append({
            'name': face.corner_right.name,
            'horiz': face.width,
            'height': 0.0,
            'dist_h': abs(horiz_from_left - face.width),
            'is_corner': True
        })

        # Landmarks as references
        for lm in face.landmarks:
            dist_h = abs(horiz_from_left - lm.horiz_from_left)
            dist_v = abs(height_agl - lm.height_agl)
            candidates.append({
                'name': lm.description_name,
                'horiz': lm.horiz_from_left,
                'height': lm.height_agl,
                'dist_h': dist_h,
                'dist_v': dist_v,
                'is_corner': False,
                'kind': lm.kind
            })

        # Sort by horizontal distance (prefer closest reference)
        candidates.sort(key=lambda c: c['dist_h'])
        best = candidates[0]

        # Generate direction phrase relative to this reference
        # "left" and "right" are from the perspective of a viewer
        # standing outside, facing the wall
        horiz_diff = horiz_from_left - best['horiz']
        horiz_dist = abs(horiz_diff)
        horiz_dist_rounded = round(horiz_dist, 1)

        if best['is_corner']:
            # For corners, just say "Xm from the [corner]"
            phrase = f"{horiz_dist_rounded}m from {best['name']}"
        else:
            if horiz_dist_rounded < 0.2:
                h_phrase = f"directly at {best['name']}"
            elif horiz_diff > 0:
                h_phrase = f"{horiz_dist_rounded}m to the right of {best['name']} when facing the building from outside"
            else:
                h_phrase = f"{horiz_dist_rounded}m to the left of {best['name']} when facing the building from outside"

            # Add vertical offset from landmark if it's a window/door
            vert_diff = height_agl - best.get('height', 0.0)
            if abs(vert_diff) > 0.3 and not best['is_corner']:
                v_dist = round(abs(vert_diff), 1)
                if vert_diff > 0:
                    h_phrase += f" and {v_dist}m above it"
                else:
                    h_phrase += f" and {v_dist}m below it"

            phrase = h_phrase

        return best['name'], horiz_dist_rounded, phrase

    def add_landmark_from_3d(self,
                              point_east: float,
                              point_north: float,
                              point_up: float,
                              kind: str,
                              confidence: float,
                              drone_lat: float,
                              drone_lon: float,
                              drone_heading_deg: float):
        """Register a detected landmark (door, window, etc.) onto the building model."""
        face = self.get_face_from_drone_pose(drone_lat, drone_lon, drone_heading_deg)
        if face is None:
            return

        horiz_from_left, height_agl = self.project_point_onto_face(
            point_east, point_north, point_up, face
        )

        lm = Landmark(
            kind=kind,
            face=face.face_id,
            horiz_from_left=horiz_from_left,
            height_agl=height_agl,
            confidence=confidence,
        )
        face.add_landmark(lm)

    def get_summary(self) -> str:
        """Return a human-readable summary of the building model and landmarks."""
        lines = [
            f"Building: {self.length}m x {self.width}m x {self.height}m, "
            f"oriented {self.orientation_deg} deg from north",
            f"Center: ({self.center_lat:.6f}, {self.center_lon:.6f})",
            ""
        ]
        for face_id, face in self.faces.items():
            lm_count = len(face.landmarks)
            lines.append(f"  {face_id.value} face ({face.width:.1f}m wide): "
                         f"{lm_count} landmarks detected")
            for lm in face.landmarks:
                lines.append(f"    - {lm.kind} at h={lm.horiz_from_left:.1f}m, "
                             f"v={lm.height_agl:.1f}m ({lm.detections} detections)")
        return "\n".join(lines)
