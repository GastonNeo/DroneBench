from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import numpy as np


UNIT_TO_METER = {
    "m": 1.0, "meter": 1.0, "meters": 1.0,
    "mm": 1e-3, "millimeter": 1e-3, "millimeters": 1e-3,
    "cm": 1e-2, "centimeter": 1e-2, "centimeters": 1e-2,
    "in": 0.0254, "inch": 0.0254, "inches": 0.0254,
}


@dataclass
class Strip3DData:
    radius_m: float
    chord_m: float
    pitch_deg: float
    points_xy: List[Tuple[float, float]]


class JSONReader3D:
    """
    Read propeller geometry from a 3D strip JSON file.

    Expected format:
        {"strip0": {"radius": float, "span": float, "points": [[x,y,z], ...]}, ...}

    Chord length, blade pitch, and 2-D airfoil profile are derived from
    the 3-D geometry:
      - chord  : distance between the two farthest points (LE / TE approximation)
      - pitch  : angle of the chord from the local rotation plane, where the
                 rotation axis is estimated by PCA on the strip centroids
      - profile: projection of strip points onto the local chord-thickness plane,
                 normalised by chord length
    """

    def __init__(self, file_path: str | Path, unit: str = "m"):
        self.file_path = Path(file_path)
        unit_key = unit.lower().strip()
        if unit_key not in UNIT_TO_METER:
            raise ValueError(
                f"Unsupported unit '{unit}'. Supported: {', '.join(sorted(UNIT_TO_METER))}"
            )
        self.scale = UNIT_TO_METER[unit_key]

    @staticmethod
    def _strip_sort_key(name: str):
        match = re.search(r"\d+", name)
        return (int(match.group()), name) if match else (10**9, name)

    @staticmethod
    def _farthest_pair(pts: np.ndarray):
        """Return the pair of points with maximum mutual distance."""
        centroid = pts.mean(axis=0)
        p1 = pts[np.argmax(np.linalg.norm(pts - centroid, axis=1))]
        p2 = pts[np.argmax(np.linalg.norm(pts - p1, axis=1))]
        return p1, p2

    @staticmethod
    def _identify_le_te(
        pts: np.ndarray, p1: np.ndarray, p2: np.ndarray
    ):
        """
        Return (le, te) with le at x=0 and te at x=1 (XFOIL convention).

        The leading edge is the rounder end → larger local opening angle.
        The trailing edge is the sharper end → smaller local opening angle.
        """
        n = len(pts)
        k = max(5, n // 30)

        def opening_angle(center: np.ndarray):
            idx = int(np.argmin(np.linalg.norm(pts - center, axis=1)))
            before = pts[(idx - k) % n]
            after = pts[(idx + k) % n]
            v1 = before - pts[idx]
            v2 = after - pts[idx]
            n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
            if n1 < 1e-12 or n2 < 1e-12:
                return 180.0
            cos_a = np.dot(v1, v2) / (n1 * n2)
            return float(np.degrees(np.arccos(np.clip(cos_a, -1.0, 1.0))))

        angle1 = opening_angle(p1)
        angle2 = opening_angle(p2)
        # LE is rounder → larger opening angle
        if angle1 >= angle2:
            return p1, p2  # p1 = LE, p2 = TE
        return p2, p1      # p2 = LE, p1 = TE

    @staticmethod
    def _find_rotation_axis(centroids: np.ndarray, chord_vecs: np.ndarray):
        """
        Estimate the propeller shaft (rotation) axis from the strip centroids.

        Default behaviour is the orientation-agnostic minimum-variance PCA normal
        (best-fit-plane normal) - it handles tilted exports and flat/2-D-style
        blades, and is what determines pitch geometrically.

        The PCA normal is only ambiguous in ONE case: a single straight blade
        whose centroids form a near-1-D line (root -> tip).  There no best-fit
        plane exists, so the in-plane datum can flip by tens of degrees on noise,
        shifting EVERY section's pitch by a constant (seen as a ~29 deg offset
        between two VX4 blades that are otherwise the same shape).  ONLY in that
        degenerate case do we pin the datum with the chords, assuming an
        axis-aligned export:
          - spanwise (radial) = global axis most aligned with the centroid line,
          - tangential        = global axis the chords most align with (a prop
                                chord is mostly tangential),
          - shaft axis        = the remaining global axis.
        """
        _, S, Vt = np.linalg.svd(centroids - centroids.mean(axis=0))

        # A clear best-fit plane exists when the smallest singular value is much
        # smaller than the middle one (a well-defined normal direction). It is a
        # 1-D line instead when the two smallest are comparable (no plane).
        degenerate = S[1] < 1e-12 or (S[2] / S[1]) > 0.30
        if not degenerate:
            return Vt[-1]   # orientation-agnostic plane normal (the usual path)

        span_axis = int(np.argmax(np.abs(Vt[0])))
        chord_mag = np.abs(np.asarray(chord_vecs, dtype=float)).mean(axis=0)
        others = [a for a in (0, 1, 2) if a != span_axis]
        tang_axis = max(others, key=lambda a: chord_mag[a])
        shaft_axis = next(a for a in others if a != tang_axis)

        axis = np.zeros(3)
        axis[shaft_axis] = 1.0
        return axis

    @staticmethod
    def _spanwise_dir(centroids: np.ndarray, i: int):
        """Local spanwise direction at strip i (finite-difference of centroids)."""
        n = len(centroids)
        if i == 0:
            v = centroids[1] - centroids[0]
        elif i == n - 1:
            v = centroids[-1] - centroids[-2]
        else:
            v = centroids[i + 1] - centroids[i - 1]
        norm = np.linalg.norm(v)
        return v / norm if norm > 1e-12 else np.array([0.0, 0.0, 1.0])

    @staticmethod
    def _extract_2d_profile(
        pts: np.ndarray,
        le: np.ndarray,
        te: np.ndarray,
        spanwise: np.ndarray,
    ):
        """Project 3-D strip points onto the chord-thickness plane, normalised by chord."""
        chord_vec = te - le
        chord_len = float(np.linalg.norm(chord_vec))
        if chord_len < 1e-9:
            return []
        chord_dir = chord_vec / chord_len

        thickness_dir = np.cross(spanwise, chord_dir)
        t_norm = np.linalg.norm(thickness_dir)
        if t_norm < 1e-9:
            fallback = np.array([1.0, 0.0, 0.0]) if abs(chord_dir[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
            thickness_dir = np.cross(chord_dir, fallback)
            thickness_dir /= np.linalg.norm(thickness_dir)
        else:
            thickness_dir /= t_norm

        shifted = pts - le
        x2d = (shifted @ chord_dir) / chord_len
        y2d = (shifted @ thickness_dir) / chord_len
        return list(zip(x2d.tolist(), y2d.tolist()))

    @staticmethod
    def _compute_pitch(
        chord_vec: np.ndarray,
        rot_axis: np.ndarray,
        centroid: np.ndarray,
    ):
        """
        Blade pitch angle = angle of the chord from the local rotation plane.
        The rotation plane at this strip is spanned by the tangential direction
        (rot_axis × radial) and the radial direction.
        """
        radial = centroid - np.dot(centroid, rot_axis) * rot_axis
        r_norm = np.linalg.norm(radial)
        if r_norm < 1e-12:
            return 0.0
        radial /= r_norm

        tangential = np.cross(rot_axis, radial)
        t_norm = np.linalg.norm(tangential)
        if t_norm < 1e-12:
            return 0.0
        tangential /= t_norm

        c_tan = float(np.dot(chord_vec, tangential))
        c_axial = float(np.dot(chord_vec, rot_axis))
        return float(np.degrees(np.arctan2(abs(c_axial), abs(c_tan))))

    PITCH_KEYS = ("pitch", "aoa")

    @staticmethod
    def _is_normalized_profile(pts: np.ndarray):
        """True when points are an already chord-normalised 2-D airfoil
        (x spans ~[0, 1], out-of-plane coordinate ~0) rather than real 3-D
        coordinates of the blade."""
        x_extent = float(pts[:, 0].max() - pts[:, 0].min())
        return abs(x_extent - 1.0) < 0.05 and float(np.abs(pts[:, 2]).max()) < 0.01

    def _explicit_strip(self, strip: dict, pts: np.ndarray):
        """Strip that already carries chord + pitch and a normalised 2-D profile:
        use those values directly instead of re-deriving them from the points
        (re-deriving would give chord ~1 and pitch ~0 on a normalised profile)."""
        pitch_key = next((k for k in self.PITCH_KEYS if k in strip), None)
        if "chord" not in strip or pitch_key is None:
            return None
        if pts.ndim != 2 or pts.shape[1] < 3 or not self._is_normalized_profile(pts):
            return None
        return Strip3DData(
            radius_m=float(strip["radius"]) * self.scale,
            chord_m=float(strip["chord"]) * self.scale,
            pitch_deg=float(strip[pitch_key]),
            points_xy=list(zip(pts[:, 0].tolist(), pts[:, 1].tolist())),
        )

    def read_strips(self):
        if not self.file_path.exists():
            raise FileNotFoundError(f"Geometry JSON not found: {self.file_path}")

        with self.file_path.open("r", encoding="utf-8") as fh:
            payload = json.load(fh)

        if not isinstance(payload, dict):
            raise ValueError("JSON root must be an object whose values are strips.")

        sorted_keys = sorted(payload.keys(), key=self._strip_sort_key)

        explicit = [
            self._explicit_strip(payload[k], np.array(payload[k].get("points", []), dtype=float))
            for k in sorted_keys
        ]
        if all(s is not None for s in explicit):
            return explicit

        all_pts: List[np.ndarray] = []
        for key in sorted_keys:
            strip = payload[key]
            if "points" not in strip:
                raise ValueError(f"Strip '{key}' is missing the 'points' field.")
            pts = np.array(strip["points"], dtype=float)
            if pts.ndim != 2 or pts.shape[1] < 3:
                raise ValueError(f"Strip '{key}': each point must have at least 3 coordinates [x, y, z].")
            all_pts.append(pts * self.scale)

        centroids = np.array([p.mean(axis=0) for p in all_pts])

        # First pass: leading/trailing edge + chord vector per strip. These are
        # needed to estimate the shaft axis (the chords pin the pitch datum).
        le_te = [self._identify_le_te(pts, *self._farthest_pair(pts)) for pts in all_pts]
        chord_vecs = np.array([te - le for le, te in le_te])
        rot_axis = self._find_rotation_axis(centroids, chord_vecs)

        strips: List[Strip3DData] = []
        for i, key in enumerate(sorted_keys):
            pts = all_pts[i]
            radius = float(payload[key]["radius"]) * self.scale

            le, te = le_te[i]
            chord_vec = chord_vecs[i]
            chord_len = float(np.linalg.norm(chord_vec))
            pitch = self._compute_pitch(chord_vec, rot_axis, centroids[i])
            span_dir = self._spanwise_dir(centroids, i)
            points_2d = self._extract_2d_profile(pts, le, te, span_dir)

            strips.append(Strip3DData(
                radius_m=radius,
                chord_m=chord_len,
                pitch_deg=pitch,
                points_xy=points_2d,
            ))

        return strips
