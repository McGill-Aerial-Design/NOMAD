"""
building_model.py

Analytical building geometry for AEAC 2026 Task 1.

The model is an N-corner polygon footprint extruded vertically by `height`. Each
consecutive corner pair forms one wall face. Face names derive from the outward
normal heading snapped to 8 compass buckets (N, NE, E, SE, S, SW, W, NW); when
two faces share a bucket (L-shape, T-shape, etc.) the name disambiguates with
the bounding corner names.

CONOPS v1.3 only guarantees a single building lat/lon and "all critical
dimensions"; building shape is NOT necessarily rectangular. Pre-flight we
reconstruct the polygon from those dimensions plus a rotation derived from
public satellite imagery, then load the corner GPS list here.
"""

import math
import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Union
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
#  Face naming: 8-bucket compass snap
# ------------------------------------------------------------------ #
_COMPASS_BUCKETS = [
    ("north",     0.0),
    ("northeast", 45.0),
    ("east",      90.0),
    ("southeast", 135.0),
    ("south",     180.0),
    ("southwest", 225.0),
    ("west",      270.0),
    ("northwest", 315.0),
]


def _snap_heading_to_compass(heading_deg: float) -> str:
    """Snap a heading (0=N, 90=E, deg CW) to the nearest of 8 compass names."""
    h = heading_deg % 360.0
    best_name = "north"
    best_diff = 360.0
    for name, center in _COMPASS_BUCKETS:
        diff = abs(((h - center) + 180.0) % 360.0 - 180.0)
        if diff < best_diff:
            best_diff = diff
            best_name = name
    return best_name


# ------------------------------------------------------------------ #
#  FaceID kept only as a back-compat alias for callers that still want
#  string face labels.
# ------------------------------------------------------------------ #
class FaceID(str, Enum):
    NORTH = "north"
    SOUTH = "south"
    EAST = "east"
    WEST = "west"
    NORTHEAST = "northeast"
    NORTHWEST = "northwest"
    SOUTHEAST = "southeast"
    SOUTHWEST = "southwest"
    ROOF = "roof"
    GROUND = "ground"


# ------------------------------------------------------------------ #
#  Data classes
# ------------------------------------------------------------------ #
@dataclass
class Corner:
    """A named corner of the building polygon (local ENU)."""
    name: str           # e.g. "NW", "north corner", "A"
    east: float
    north: float

    @property
    def description_name(self) -> str:
        # Operator-facing form. Avoid double "corner" suffix.
        n = self.name.strip()
        if n.isdigit():
            return f"corner {n}"
        if n.lower().endswith("corner"):
            return n
        return f"{n} corner"


@dataclass
class Face:
    """One vertical wall face: a polygon edge from corner_left to corner_right
    when viewed from outside the building."""
    name: str                          # e.g. "north", or "north between A and B"
    bucket: str                        # raw compass bucket before disambiguation
    corner_left: Corner
    corner_right: Corner
    width: float
    outward_normal_deg: float          # heading (deg CW from north)
    plane_point_east: float
    plane_point_north: float
    normal_east: float
    normal_north: float


@dataclass
class PlaneHit:
    """Result of nearest-plane classification."""
    kind: str                 # 'wall', 'ground', or 'roof'
    face: Optional[Face]      # populated only when kind == 'wall'
    distance: float

    @property
    def label(self) -> str:
        return self.face.name if self.face is not None else self.kind


# ------------------------------------------------------------------ #
#  Building Model
# ------------------------------------------------------------------ #
class BuildingModel:
    """
    Polygon building model for target localization.

    Two ways to construct:
      1. Pass `corners_local` directly (preferred): list of (name, east, north)
         in the local ENU frame. The polygon may have any number of corners >=3.
      2. Pass rectangle parameters (`length`, `width`, `orientation_deg`) for
         the convenience case of a rectangular building. Corners are derived
         and labeled NW/NE/SE/SW.

    `center_lat`/`center_lon` define the GPS origin of the local ENU frame and
    must be provided in either case (used by the node for GPS<->local conversion).
    """

    def __init__(
        self,
        center_lat: float,
        center_lon: float,
        height: float,
        corners_local: Optional[List[Tuple[str, float, float]]] = None,
        # Rectangle convenience (only used when corners_local is None):
        length: float = 10.0,
        width: float = 6.0,
        orientation_deg: float = 0.0,
    ):
        self.center_lat = float(center_lat)
        self.center_lon = float(center_lon)
        self.height = float(height)

        if corners_local is None:
            # Build a 4-corner rectangle and use it through the polygon path.
            corners_local = self._rectangle_corners(
                length=float(length),
                width=float(width),
                orientation_deg=float(orientation_deg),
            )
            self.length = float(length)
            self.width = float(width)
            self.orientation_deg = float(orientation_deg)
        else:
            # Derived dimensions reported in summary, not used for math.
            self.length = 0.0
            self.width = 0.0
            self.orientation_deg = 0.0

        if len(corners_local) < 3:
            raise ValueError(
                f"BuildingModel requires at least 3 corners, got {len(corners_local)}"
            )

        self.corners: List[Corner] = [
            Corner(name=str(name), east=float(e), north=float(n))
            for (name, e, n) in corners_local
        ]
        self.corners_by_name: Dict[str, Corner] = {c.name: c for c in self.corners}

        # Centroid of the polygon, used as the "inside" reference for outward
        # normals. For non-convex shapes the centroid still lies on the inside
        # for typical building footprints (rectangles, L-shapes, T-shapes).
        cx = float(np.mean([c.east for c in self.corners]))
        cy = float(np.mean([c.north for c in self.corners]))
        self.centroid_east = cx
        self.centroid_north = cy

        self.faces: List[Face] = self._build_faces(self.corners)

        # Backwards-compat: dict access like building.faces[FaceID.NORTH].
        # When multiple faces share a compass bucket, only the first lands here.
        # New code should iterate self.faces (the list) instead.
        self.faces_by_name: Dict[str, Face] = {}
        for f in self.faces:
            self.faces_by_name.setdefault(f.bucket, f)
            self.faces_by_name[f.name] = f

        self.ground_plane_z = 0.0
        self.roof_plane_z = self.height

    # ------------------------------------------------------------ #
    #  Construction helpers
    # ------------------------------------------------------------ #
    def _rectangle_corners(self,
                           length: float,
                           width: float,
                           orientation_deg: float) -> List[Tuple[str, float, float]]:
        """Generate 4 corners labeled NW/NE/SE/SW from rectangle parameters."""
        # orientation_deg = heading of the long axis, deg CW from north.
        # Convert to standard math angle: CCW from east.
        orientation_rad = math.radians(90.0 - orientation_deg)
        long_axis = np.array([math.cos(orientation_rad), math.sin(orientation_rad)])
        short_axis = np.array([-math.sin(orientation_rad), math.cos(orientation_rad)])

        half_l = length / 2.0
        half_w = width / 2.0
        candidate_pts = [
            long_axis * half_l + short_axis * half_w,
            long_axis * half_l - short_axis * half_w,
            -long_axis * half_l - short_axis * half_w,
            -long_axis * half_l + short_axis * half_w,
        ]

        # Sort top-to-bottom by north, then left-to-right by east.
        ordered = sorted(candidate_pts, key=lambda p: (p[1], p[0]), reverse=True)
        top = sorted(ordered[:2], key=lambda p: p[0])  # west then east
        bot = sorted(ordered[2:], key=lambda p: p[0])

        # CCW order so polygon winding is consistent.
        return [
            ("NW", float(top[0][0]), float(top[0][1])),
            ("SW", float(bot[0][0]), float(bot[0][1])),
            ("SE", float(bot[1][0]), float(bot[1][1])),
            ("NE", float(top[1][0]), float(top[1][1])),
        ]

    def _build_faces(self, corners: List[Corner]) -> List[Face]:
        """Build wall faces from consecutive corner pairs. Returns faces with
        names disambiguated when multiple faces fall in the same compass
        bucket."""
        n = len(corners)
        cx = float(np.mean([c.east for c in corners]))
        cy = float(np.mean([c.north for c in corners]))

        raw_faces: List[Face] = []
        for i in range(n):
            c1 = corners[i]
            c2 = corners[(i + 1) % n]

            p1 = np.array([c1.east, c1.north], dtype=float)
            p2 = np.array([c2.east, c2.north], dtype=float)
            edge = p2 - p1
            edge_len = float(np.linalg.norm(edge))
            if edge_len < 1e-6:
                continue  # skip degenerate edges

            midpoint = 0.5 * (p1 + p2)
            outward = midpoint - np.array([cx, cy])
            norm = float(np.linalg.norm(outward))
            if norm < 1e-6:
                # Centroid lies on the edge midpoint (rare). Fall back to the
                # right-hand perpendicular of the edge as the outward normal.
                outward = np.array([edge[1], -edge[0]], dtype=float)
                norm = float(np.linalg.norm(outward))
            outward = outward / max(norm, 1e-6)

            # Order corners left-to-right when viewed from outside the wall.
            viewer_facing = -outward
            viewer_right = np.array([viewer_facing[1], -viewer_facing[0]], dtype=float)
            cl, cr = c1, c2
            if float(np.dot(p2 - p1, viewer_right)) < 0.0:
                cl, cr = c2, c1

            heading = self._heading_deg_from_vec(outward)
            bucket = _snap_heading_to_compass(heading)

            raw_faces.append(Face(
                name=bucket,  # provisional, disambiguated below
                bucket=bucket,
                corner_left=cl,
                corner_right=cr,
                width=edge_len,
                outward_normal_deg=heading,
                plane_point_east=float(midpoint[0]),
                plane_point_north=float(midpoint[1]),
                normal_east=float(outward[0]),
                normal_north=float(outward[1]),
            ))

        # Disambiguate names when multiple faces share a compass bucket.
        bucket_counts: Dict[str, int] = {}
        for f in raw_faces:
            bucket_counts[f.bucket] = bucket_counts.get(f.bucket, 0) + 1
        for f in raw_faces:
            if bucket_counts[f.bucket] > 1:
                f.name = (
                    f"{f.bucket} (between {f.corner_left.description_name} "
                    f"and {f.corner_right.description_name})"
                )
        return raw_faces

    @staticmethod
    def _heading_deg_from_vec(vec: np.ndarray) -> float:
        """Convert ENU vector to heading degrees clockwise from north."""
        rad = math.atan2(float(vec[1]), float(vec[0]))
        return (90.0 - math.degrees(rad)) % 360.0

    # ------------------------------------------------------------ #
    #  Geometry queries
    # ------------------------------------------------------------ #
    def _distance_to_wall_plane(self, point_east: float, point_north: float, face: Face) -> float:
        """Unsigned perpendicular distance from a horizontal point to one wall plane."""
        dx = point_east - face.plane_point_east
        dy = point_north - face.plane_point_north
        signed = dx * face.normal_east + dy * face.normal_north
        return abs(float(signed))

    def classify_nearest_plane(self,
                               point_east: float,
                               point_north: float,
                               point_up: float) -> PlaneHit:
        """Classify a 3D point by nearest of N walls + ground + roof."""
        best: Optional[PlaneHit] = None

        for face in self.faces:
            d = self._distance_to_wall_plane(point_east, point_north, face)
            if best is None or d < best.distance:
                best = PlaneHit(kind='wall', face=face, distance=d)

        d_ground = abs(float(point_up - self.ground_plane_z))
        if best is None or d_ground < best.distance:
            best = PlaneHit(kind='ground', face=None, distance=d_ground)

        d_roof = abs(float(point_up - self.roof_plane_z))
        if best is None or d_roof < best.distance:
            best = PlaneHit(kind='roof', face=None, distance=d_roof)

        # If the nearest wall hit is within a small epsilon of the roof plane,
        # reclassify as roof to avoid ambiguous ground-wall-roof decisions for
        # targets on parapets, antennas, or edge-mounted targets at roof height.
        if best is not None and best.kind == "wall":
            wall_dist = best.distance
            if wall_dist < 0.05 and d_roof < 0.3:
                roof_hit = PlaneHit(kind='roof', face=None, distance=d_roof)
                best = roof_hit

        return best  # type: ignore[return-value]

    def get_nearest_wall_face(self,
                              point_east: float,
                              point_north: float) -> Tuple[Face, float]:
        """Return the closest wall face to a horizontal point position."""
        ranked = [
            (face, self._distance_to_wall_plane(point_east, point_north, face))
            for face in self.faces
        ]
        ranked.sort(key=lambda item: item[1])
        return ranked[0]

    # ------------------------------------------------------------ #
    #  Drone-relative queries
    # ------------------------------------------------------------ #
    def _get_face_from_local_pose(self,
                                  drone_east: float,
                                  drone_north: float,
                                  drone_heading_deg: float) -> Optional[Face]:
        """Pick the visible face from a drone pose already in local ENU."""
        to_drone = np.array(
            [drone_east - self.centroid_east, drone_north - self.centroid_north],
            dtype=float,
        )

        best_face: Optional[Face] = None
        best_score = -math.inf

        for face in self.faces:
            normal_vec = np.array([face.normal_east, face.normal_north], dtype=float)

            # Drone must be on the outward-normal side of this face.
            side_score = float(np.dot(to_drone, normal_vec))
            if side_score < 0:
                continue

            # Drone heading should oppose the face normal (looking into the wall).
            drone_look_rad = math.radians(90.0 - drone_heading_deg)
            drone_look = np.array([math.cos(drone_look_rad), math.sin(drone_look_rad)])
            alignment = -float(np.dot(drone_look, normal_vec))

            combined = alignment * 2.0 + side_score * 0.5
            if combined > best_score:
                best_score = combined
                best_face = face

        return best_face

    def get_face_from_drone_pose(self,
                                 drone_lat: float,
                                 drone_lon: float,
                                 drone_heading_deg: float) -> Optional[Face]:
        """View-direction face selection from drone GPS + heading."""
        drone_east, drone_north = gps_to_local(
            drone_lat, drone_lon, self.center_lat, self.center_lon
        )
        return self._get_face_from_local_pose(
            drone_east, drone_north, drone_heading_deg,
        )

    def get_face_from_local_pose(self,
                                 drone_east: float,
                                 drone_north: float,
                                 drone_heading_deg: float) -> Optional[Face]:
        """View-direction face selection from local ENU pose."""
        return self._get_face_from_local_pose(
            drone_east, drone_north, drone_heading_deg,
        )

    def project_point_onto_face(self,
                                point_east: float,
                                point_north: float,
                                point_up: float,
                                face: Face) -> Tuple[float, float]:
        """Project a 3D point onto a building face.

        Returns (horiz_from_left, height_agl) where horiz_from_left is the
        distance from the face's left corner when viewed from outside, in
        meters; clamped to [0, face.width]. height_agl is the height above
        ground, clamped to [0, building.height].
        """
        cl = np.array([face.corner_left.east, face.corner_left.north])
        cr = np.array([face.corner_right.east, face.corner_right.north])

        face_horiz = cr - cl
        face_length = float(np.linalg.norm(face_horiz))
        if face_length < 0.01:
            return 0.0, max(0.0, min(float(point_up), self.height))
        face_horiz_unit = face_horiz / face_length

        point_2d = np.array([point_east, point_north])
        offset = point_2d - cl
        horiz_from_left = float(np.dot(offset, face_horiz_unit))
        horiz_from_left = max(0.0, min(horiz_from_left, face_length))
        # Use a small epsilon (0.05m) so targets slightly above roof height
        # are clamped to roof rather than being excluded or producing >height AGL.
        height_agl = max(0.0, min(float(point_up), self.height + 0.05))
        return float(horiz_from_left), float(height_agl)

    def find_nearest_reference(self,
                               face: Face,
                               horiz_from_left: float,
                               height_agl: float) -> Tuple[str, float, str]:
        """Find the nearest face-corner for a point on a face, and return a
        human-readable phrase.

        Returns:
            (ref_name, horizontal_distance_m, full_phrase)
        """
        dist_left = abs(horiz_from_left)
        dist_right = abs(horiz_from_left - face.width)

        if dist_left <= dist_right:
            ref_name = face.corner_left.description_name
            horiz_dist = dist_left
        else:
            ref_name = face.corner_right.description_name
            horiz_dist = dist_right

        horiz_dist_rounded = round(horiz_dist, 1)
        phrase = f"{horiz_dist_rounded}m from the {ref_name}"
        return ref_name, horiz_dist_rounded, phrase

    # ------------------------------------------------------------ #
    #  Diagnostics
    # ------------------------------------------------------------ #
    def get_summary(self) -> str:
        lines = [
            f"Polygon footprint: {len(self.corners)} corners, height {self.height}m",
            f"Center (GPS origin): ({self.center_lat:.6f}, {self.center_lon:.6f})",
            f"Centroid (local ENU): ({self.centroid_east:.2f}, {self.centroid_north:.2f})",
            "",
            "Corners:",
        ]
        for c in self.corners:
            lines.append(f"  {c.name}: E={c.east:.2f}m, N={c.north:.2f}m")
        lines.append("")
        lines.append("Faces:")
        for f in self.faces:
            lines.append(
                f"  {f.name} face: width={f.width:.2f}m, "
                f"normal_heading={f.outward_normal_deg:.1f} deg, "
                f"left={f.corner_left.name}, right={f.corner_right.name}"
            )
        return "\n".join(lines)
