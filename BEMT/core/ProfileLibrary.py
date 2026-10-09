from __future__ import annotations

from pathlib import Path
from typing import Dict

import numpy as np

from core.Profil import Profil


class AirfoilLibrary:
    """Singleton-like cache for aerodynamic profiles."""

    _profiles: Dict[str, Profil] = {}
    _sources: Dict[str, Path] = {}

    @staticmethod
    def _resolve_file_path(file_name: str | Path):
        candidate = Path(file_name)
        if candidate.is_absolute():
            return candidate

        core_dir = Path(__file__).resolve().parent
        root_dir = core_dir.parent
        root_candidate = (root_dir / candidate).resolve()
        if root_candidate.exists():
            return root_candidate

        return candidate.resolve()

    @staticmethod
    def load_profile(name: str, file_name: str | Path, re_ref: float | None = None):
        file_path = AirfoilLibrary._resolve_file_path(file_name)
        if not file_path.exists():
            raise FileNotFoundError(f"Airfoil data file not found: {file_path}")

        if name in AirfoilLibrary._profiles:
            same_source = AirfoilLibrary._sources.get(name) == file_path
            if same_source:
                profile = AirfoilLibrary._profiles[name]
                if re_ref is not None:
                    profile.re_ref = re_ref
                return profile

        alpha, cl, cd = np.loadtxt(file_path, unpack=True)
        profile = Profil([alpha, cl, cd], re_ref=re_ref)
        AirfoilLibrary._profiles[name] = profile
        AirfoilLibrary._sources[name] = file_path
        return profile

    @staticmethod
    def get_profile(name: str):
        if name not in AirfoilLibrary._profiles:
            raise ValueError(f"Profile {name} is not loaded. Use `load_profile` first.")
        return AirfoilLibrary._profiles[name]
