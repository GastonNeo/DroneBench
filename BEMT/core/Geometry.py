from __future__ import annotations

from dataclasses import dataclass
import logging
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from core.JSONReader import JSONReader
from core.ProfileLibrary import AirfoilLibrary
from core.SimulationParameters import SimulationParameters
from utils.XFOIL import XFOILPolarGenerator


@dataclass
class GeometrySection:
    radius: float
    chord: float
    pitch: float
    profile_name: str
    points_xy: Optional[List[Tuple[float, float]]] = None
    polar_path: Optional[Path] = None
    re_ref: Optional[float] = None  # Reynolds number at which this polar was computed


class Geometry:
    """
    Geometry holder for one propeller:
    radius, chord, pitch, profile definition (manual or from JSON strips).
    """

    def __init__(self, name: str, sections: List[GeometrySection], geometry_root: str | Path = "geometries"):
        if not sections:
            raise ValueError("Geometry requires at least one section.")

        self.name = name
        self.sections = sections
        self.geometry_root = Path(geometry_root)
        self.geometry_dir = (self.geometry_root / self.name).resolve()

        self._validate_sections()

    def _validate_sections(self):
        for idx, section in enumerate(self.sections):
            if section.radius <= 0:
                raise ValueError(f"Section {idx} has a non-positive radius ({section.radius}).")
            if section.chord <= 0:
                raise ValueError(f"Section {idx} has a non-positive chord ({section.chord}).")

    @property
    def radii(self):
        return [section.radius for section in self.sections]

    @property
    def chords(self):
        return [section.chord for section in self.sections]

    @property
    def pitches(self):
        return [section.pitch for section in self.sections]

    @property
    def profile_names(self):
        return [section.profile_name for section in self.sections]

    @property
    def section_count(self):
        return len(self.sections)

    @property
    def polars_dir(self):
        return self.geometry_dir / "Polars"

    @classmethod
    def from_manual(
        cls,
        name: str,
        radii: Sequence[float],
        chords: Sequence[float],
        pitches: Sequence[float],
        profiles: str | Sequence[str],
        geometry_root: str | Path = "geometries",
    ):
        if not (len(radii) == len(chords) == len(pitches)):
            raise ValueError("radii, chords and pitches must have the same size.")

        if isinstance(profiles, str):
            profile_names = [profiles] * len(radii)
        else:
            profile_names = list(profiles)
            if len(profile_names) != len(radii):
                raise ValueError("profiles must be one name or a list matching section count.")

        sections = [
            GeometrySection(
                radius=float(radius),
                chord=float(chord),
                pitch=float(pitch),
                profile_name=profile_name,
                points_xy=None,
                polar_path=None,
            )
            for radius, chord, pitch, profile_name in zip(radii, chords, pitches, profile_names)
        ]
        return cls(name=name, sections=sections, geometry_root=geometry_root)

    @classmethod
    def from_json(
        cls,
        propeller_name: str,
        json_file: str | Path | None = None,
        unit: str = "m",
        geometry_root: str | Path = "geometries",
        max_points: int = 1000,
    ):
        geometry_root_path = Path(geometry_root)
        geometry_dir = (geometry_root_path / propeller_name).resolve()

        if json_file is None:
            candidates = sorted(geometry_dir.glob("*.json"))
            if not candidates:
                raise FileNotFoundError(f"No JSON file found in {geometry_dir}")
            json_path = candidates[0]
        else:
            json_path = Path(json_file)
            if not json_path.is_absolute():
                json_path = (geometry_dir / json_path).resolve()

        strips = JSONReader(json_path, unit=unit).read_strips(max_points=max_points)
        sections: List[GeometrySection] = []
        for index, strip in enumerate(strips, start=1):
            sections.append(
                GeometrySection(
                    radius=strip.radius_m,
                    chord=strip.chord_m,
                    pitch=strip.pitch_deg,
                    profile_name=f"{propeller_name}_strip_{index:03d}",
                    points_xy=strip.points_xy,
                    polar_path=None,
                )
            )

        return cls(name=propeller_name, sections=sections, geometry_root=geometry_root)

    @classmethod
    def from_json3d(
        cls,
        propeller_name: str,
        json_file: str | Path,
        unit: str = "m",
        geometry_root: str | Path = "geometries",
    ):
        """Load geometry from a 3D strip JSON file (radius + 3D points per strip).

        Chord, pitch, and 2-D airfoil profile are derived automatically from
        the 3-D geometry via JSONReader3D.
        """
        from core.JSONReader3D import JSONReader3D

        geometry_root_path = Path(geometry_root)
        geometry_dir = (geometry_root_path / propeller_name).resolve()

        json_path = Path(json_file)
        if not json_path.is_absolute():
            json_path = (geometry_dir / json_path).resolve()

        strips = JSONReader3D(json_path, unit=unit).read_strips()
        sections: List[GeometrySection] = [
            GeometrySection(
                radius=strip.radius_m,
                chord=strip.chord_m,
                pitch=strip.pitch_deg,
                profile_name=f"{propeller_name}_strip_{i + 1:03d}",
                points_xy=strip.points_xy,
                polar_path=None,
            )
            for i, strip in enumerate(strips)
        ]
        return cls(name=propeller_name, sections=sections, geometry_root=geometry_root)

    def _resolve_polar_path(self, polar_file: str | Path):
        candidate = Path(polar_file)
        if candidate.is_absolute():
            return candidate

        scoped_candidates = [
            self.polars_dir / candidate,
            self.geometry_dir / candidate,
            Path.cwd() / candidate,
        ]
        for scoped in scoped_candidates:
            if scoped.exists():
                return scoped.resolve()

        return (self.polars_dir / candidate).resolve()

    def set_shared_polar(self, polar_file: str | Path):
        path = self._resolve_polar_path(polar_file)
        for section in self.sections:
            section.polar_path = path
            section.profile_name = path.stem

    def set_section_polar(self, section_index: int, polar_file: str | Path):
        if section_index < 0 or section_index >= self.section_count:
            raise IndexError(f"Section index {section_index} is out of bounds.")
        path = self._resolve_polar_path(polar_file)
        section = self.sections[section_index]
        section.polar_path = path
        section.profile_name = path.stem

    def set_section_polars(self, polar_mapping: Dict[int, str | Path] | Sequence[str | Path]):
        if isinstance(polar_mapping, dict):
            for idx, path in polar_mapping.items():
                self.set_section_polar(int(idx), path)
            return

        files = list(polar_mapping)
        if len(files) != self.section_count:
            raise ValueError("section_polar_files list must match section count.")
        for idx, path in enumerate(files):
            self.set_section_polar(idx, path)

    def autodiscover_polars(self):
        if not self.polars_dir.exists():
            return

        dat_files = sorted(self.polars_dir.glob("*.dat"))
        if not dat_files:
            return
        if len(dat_files) == 1:
            self.set_shared_polar(dat_files[0])
            return

        indexed = {file_path.name.lower(): file_path for file_path in dat_files}
        for idx, section in enumerate(self.sections, start=1):
            candidates = [
                f"{section.profile_name}.dat".lower(),
                f"{self.name}_strip_{idx:03d}.dat".lower(),
                f"{self.name}_section_{idx:03d}.dat".lower(),
                f"section_{idx:03d}.dat".lower(),
            ]
            for candidate in candidates:
                if candidate in indexed:
                    section.polar_path = indexed[candidate]
                    section.profile_name = indexed[candidate].stem
                    break

    def _compute_reynolds(self, section: GeometrySection, simulation: SimulationParameters):
        v_local = simulation.omega * section.radius
        return simulation.rho * v_local * section.chord / simulation.mu

    def calculate_polars_if_needed(
        self,
        simulation: SimulationParameters,
        shared_polar: bool = False,
        force_recompute: bool = False,
        aoa_start: float = 0,
        aoa_end: float = 20.0,
        aoa_step: float = 0.5,
        iterations: int = 100,
        xcrit: float = 1,
        xfoil_executable: str | Path | None = None,
        xfoil_timeout: int = 120,
    ):
        self.polars_dir.mkdir(parents=True, exist_ok=True)
        generator = XFOILPolarGenerator(xfoil_executable=xfoil_executable)

        if shared_polar:
            target = self.polars_dir / f"{self.name}_shared.dat"
            if force_recompute or not target.exists():
                source = self.sections[0]
                if not source.points_xy:
                    raise ValueError(
                        "Cannot generate shared polar: no profile points available in geometry."
                    )
                re_value = self._compute_reynolds(source, simulation)
                generator.generate_from_points(
                    points_xy=source.points_xy,
                    reynolds=re_value,
                    output_file=target,
                    aoa_start=aoa_start,
                    aoa_end=aoa_end,
                    aoa_step=aoa_step,
                    iterations=iterations,
                    xcrit=xcrit,
                    max_points=999,
                    timeout_seconds=xfoil_timeout,
                )
            self.set_shared_polar(target)
            return

        for idx, section in enumerate(self.sections, start=1):
            has_existing_polar = section.polar_path is not None and section.polar_path.exists()
            if has_existing_polar and not force_recompute:
                continue

            target = self.polars_dir / f"{self.name}_strip_{idx:03d}.dat"
            if target.exists() and not force_recompute:
                section.polar_path = target
                section.profile_name = target.stem
                continue

            if not section.points_xy:
                raise ValueError(
                    f"Section {idx} has no profile points and no existing polar. "
                    "Provide a manual polar file for this section."
                )

            re_value = self._compute_reynolds(section, simulation)
            try:
                generator.generate_from_points(
                    points_xy=section.points_xy,
                    reynolds=re_value,
                    output_file=target,
                    aoa_start=aoa_start,
                    aoa_end=aoa_end,
                    aoa_step=aoa_step,
                    iterations=iterations,
                    xcrit=xcrit,
                    max_points=999,
                    timeout_seconds=xfoil_timeout,
                )
                section.polar_path = target
                section.profile_name = target.stem
                section.re_ref = re_value
            except Exception as exc:
                fallback = next(
                    (
                        s.polar_path
                        for s in self.sections
                        if s.polar_path is not None and s.polar_path.exists()
                    ),
                    None,
                )
                if fallback is None:
                    raise RuntimeError(
                        f"XFOIL failed for section {idx} and no fallback polar is available."
                    ) from exc

                section.polar_path = fallback
                section.profile_name = fallback.stem
                logging.warning(
                    "XFOIL failed for section %s (%s). Falling back to polar '%s'.",
                    idx,
                    exc,
                    fallback.name,
                )

    def ensure_polars(
        self,
        simulation: SimulationParameters,
        shared_polar: bool = False,
        shared_polar_file: str | Path | None = None,
        section_polar_files: Dict[int, str | Path] | Sequence[str | Path] | None = None,
        generate_missing: bool = True,
        force_recompute: bool = False,
        aoa_start: float = 0,
        aoa_end: float = 20.0,
        aoa_step: float = 0.5,
        iterations: int = 100,
        xcrit: float = 1,
        xfoil_executable: str | Path | None = None,
        xfoil_timeout: int = 120,
    ):
        self.autodiscover_polars()

        if shared_polar_file is not None:
            self.set_shared_polar(shared_polar_file)
        if section_polar_files is not None:
            self.set_section_polars(section_polar_files)

        missing_indices = [
            idx
            for idx, section in enumerate(self.sections)
            if section.polar_path is None or not section.polar_path.exists()
        ]

        if missing_indices and generate_missing:
            self.calculate_polars_if_needed(
                simulation=simulation,
                shared_polar=shared_polar,
                force_recompute=force_recompute,
                aoa_start=aoa_start,
                aoa_end=aoa_end,
                aoa_step=aoa_step,
                iterations=iterations,
                xcrit=xcrit,
                xfoil_executable=xfoil_executable,
            )
            missing_indices = [
                idx
                for idx, section in enumerate(self.sections)
                if section.polar_path is None or not section.polar_path.exists()
            ]

        if missing_indices:
            # Last safety net: if some sections are still missing, reuse nearest
            # available section polar to keep the simulation runnable.
            available = [
                (idx, section.polar_path)
                for idx, section in enumerate(self.sections)
                if section.polar_path is not None and section.polar_path.exists()
            ]
            if available:
                for missing_idx in missing_indices:
                    nearest_idx, nearest_path = min(
                        available, key=lambda item: abs(item[0] - missing_idx)
                    )
                    self.sections[missing_idx].polar_path = nearest_path
                    self.sections[missing_idx].profile_name = nearest_path.stem
                    logging.warning(
                        "Missing polar for section %s. Reusing nearest section %s polar '%s'.",
                        missing_idx + 1,
                        nearest_idx + 1,
                        nearest_path.name,
                    )

                missing_indices = [
                    idx
                    for idx, section in enumerate(self.sections)
                    if section.polar_path is None or not section.polar_path.exists()
                ]

        if missing_indices:
            missing_text = ", ".join(str(i + 1) for i in missing_indices)
            raise FileNotFoundError(
                "Missing polar files for sections: "
                f"{missing_text}. Provide `shared_polar_file`, `section_polar_files`, "
                "or enable generation with profile points."
            )

    def load_profiles(self):
        profiles = []
        for section in self.sections:
            if section.polar_path is None or not section.polar_path.exists():
                raise FileNotFoundError(
                    f"No polar file available for profile '{section.profile_name}'."
                )
            AirfoilLibrary.load_profile(section.profile_name, section.polar_path,
                                        re_ref=section.re_ref)
            profiles.append(AirfoilLibrary.get_profile(section.profile_name))
        return profiles
