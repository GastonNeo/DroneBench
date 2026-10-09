from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Tuple


UNIT_TO_METER = {
    "m": 1.0,
    "meter": 1.0,
    "meters": 1.0,
    "mm": 1e-3,
    "millimeter": 1e-3,
    "millimeters": 1e-3,
    "cm": 1e-2,
    "centimeter": 1e-2,
    "centimeters": 1e-2,
    "in": 0.0254,
    "inch": 0.0254,
    "inches": 0.0254,
}


@dataclass
class StripData:
    radius_m: float
    chord_m: float
    pitch_deg: float
    points_xy: List[Tuple[float, float]]


class JSONReader:
    """Read strip-based geometry JSON files and normalize values in meters."""

    def __init__(self, file_path: str | Path, unit: str = "m"):
        self.file_path = Path(file_path)
        unit_key = unit.lower().strip()
        if unit_key not in UNIT_TO_METER:
            valid_units = ", ".join(sorted(UNIT_TO_METER))
            raise ValueError(f"Unsupported unit '{unit}'. Supported units: {valid_units}")
        self.scale_to_meter = UNIT_TO_METER[unit_key]

    def _strip_sort_key(self, name: str):
        match = re.search(r"\d+", name)
        if match:
            return int(match.group()), name
        return 10**9, name

    def _reduce_points(self, points_xy: Sequence[Tuple[float, float]], max_points: int = 1000):
        if len(points_xy) <= max_points:
            return list(points_xy)
        if max_points < 2:
            raise ValueError("max_points must be >= 2.")

        stride = (len(points_xy) - 1) / (max_points - 1)
        reduced = []
        for i in range(max_points):
            idx = int(round(i * stride))
            idx = min(idx, len(points_xy) - 1)
            reduced.append(points_xy[idx])
        return reduced

    def read_strips(self, max_points: int = 1000):
        if not self.file_path.exists():
            raise FileNotFoundError(f"Geometry JSON not found: {self.file_path}")

        with self.file_path.open("r", encoding="utf-8") as handle:
            payload = json.load(handle)

        if not isinstance(payload, dict):
            raise ValueError("Geometry JSON root must be an object containing strips.")

        strips: List[StripData] = []
        for strip_name in sorted(payload.keys(), key=self._strip_sort_key):
            strip = payload[strip_name]

            for required_key in ("radius", "chord", "aoa", "points"):
                if required_key not in strip:
                    raise ValueError(f"Missing key '{required_key}' in strip '{strip_name}'.")

            points_raw = strip["points"]
            points_xy: List[Tuple[float, float]] = []
            for idx, point in enumerate(points_raw):
                if not isinstance(point, (list, tuple)) or len(point) < 2:
                    raise ValueError(
                        f"Invalid point at index {idx} in strip '{strip_name}'. "
                        "Expected at least [x, y]."
                    )
                points_xy.append((float(point[0]), float(point[1])))

            strips.append(
                StripData(
                    radius_m=float(strip["radius"]) * self.scale_to_meter,
                    chord_m=float(strip["chord"]) * self.scale_to_meter,
                    pitch_deg=float(strip["aoa"]),
                    points_xy=self._reduce_points(points_xy, max_points=max_points),
                )
            )

        if not strips:
            raise ValueError("No strips found in geometry JSON.")
        return strips
