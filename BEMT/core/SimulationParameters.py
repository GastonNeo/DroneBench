from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path


@dataclass
class SimulationParameters:
    """Operating conditions and global rotor parameters for one simulation run."""

    v_inf: float
    rpm: float
    blade_count: int
    rotor_diameter: float
    rho: float = 1.225
    mu: float = 1.81e-5
    unlinear: bool = False
    azimuth: float = 0.0
    a_disk: float = 0.0

    @property
    def omega(self):
        return self.rpm * 2.0 * math.pi / 60.0

    @classmethod
    def from_json(cls, file_path: str | Path):
        path = Path(file_path)
        with path.open("r", encoding="utf-8") as handle:
            payload = json.load(handle)

        return cls(
            v_inf=float(payload["v_inf"]),
            rpm=float(payload["rpm"]),
            blade_count=int(payload["blade_count"]),
            rotor_diameter=float(payload["rotor_diameter"]),
            rho=float(payload.get("rho", 1.225)),
            mu=float(payload.get("mu", 1.81e-5)),
            unlinear=bool(payload.get("unlinear", False)),
            azimuth=float(payload.get("azimuth", 0.0)),
            a_disk=float(payload.get("a_disk", 0.0)),
        )
