"""High-level BEMT entry point backed by the bundled aerodynamic surrogate."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from core.Geometry import Geometry
from core.Simulation import SingleProp
from core.SimulationParameters import SimulationParameters

from ._polarmodel import load_model, model_profiles

_PKG_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _PKG_DIR.parent
GEOMETRIES_DIR = _REPO_ROOT / "geometries"
MODEL_PATH = _PKG_DIR / "polar_model.joblib"

_G = 9.81  # m/s^2, for thrust-in-grams reporting


@dataclass
class BEMTResult:
    """Outcome of a single BEMT run."""

    thrust: float          # N
    torque: float          # N.m
    power: float           # W
    geometry: str
    rpm: float
    sections: list[dict[str, Any]]

    @property
    def thrust_grams(self):
        return self.thrust * 1000.0 / _G

    def __str__(self):
        return (f"BEMT[{self.geometry} @ {self.rpm:.0f} rpm]  "
                f"T={self.thrust:.3f} N ({self.thrust_grams:.0f} g)  "
                f"Q={self.torque:.4f} Nm  P={self.power:.2f} W")


def _resolve_params(sim, rpm, v_inf, blade_count, rotor_diameter):
    """Build the operating point from a sim file and/or explicit kwargs.

    Explicit keyword arguments always win over values read from ``sim``.
    """
    if sim is not None:
        params = SimulationParameters.from_json(sim)
    else:
        if None in (rpm, blade_count, rotor_diameter):
            raise ValueError(
                "Provide either sim=<SimParam.json> or all of "
                "rpm, blade_count, rotor_diameter."
            )
        params = SimulationParameters(
            v_inf=0.0 if v_inf is None else float(v_inf),
            rpm=float(rpm),
            blade_count=int(blade_count),
            rotor_diameter=float(rotor_diameter),
        )
    if rpm is not None:
        params.rpm = float(rpm)
    if v_inf is not None:
        params.v_inf = float(v_inf)
    if blade_count is not None:
        params.blade_count = int(blade_count)
    if rotor_diameter is not None:
        params.rotor_diameter = float(rotor_diameter)
    return params


def _resolve_geometry(geometry, json_file, unit, geometry_root):
    """Load a Geometry from a folder name under geometries/ OR a direct .json path.

    Returns (geometry, label). A direct path ignores ``json_file``/``geometry_root``
    and labels the result by the file stem.
    """
    geom_path = Path(geometry)
    if geom_path.suffix.lower() == ".json" or geom_path.is_file():
        json_path = geom_path.resolve()
        if not json_path.exists():
            raise FileNotFoundError(f"Geometry JSON not found: {json_path}")
        label = json_path.stem
        # Standalone JSON: cache any XFOIL polars next to it (<json_dir>/<stem>/Polars).
        return Geometry.from_json3d(label, json_path, unit=unit,
                                    geometry_root=json_path.parent), label
    root = Path(geometry_root) if geometry_root is not None else GEOMETRIES_DIR
    return Geometry.from_json3d(geometry, json_file, unit=unit, geometry_root=root), str(geometry)


def run(
    geometry: str,
    *,
    json_file: str = "BladeData-1.json",
    unit: str = "m",
    geometry_root: str | Path | None = None,
    sim: str | Path | None = None,
    rpm: float | None = None,
    v_inf: float | None = None,
    blade_count: int | None = None,
    rotor_diameter: float | None = None,
    model: str | Path | None = None,
):
    """Run BEMT on a propeller geometry using the bundled Cl/Cd surrogate.

    Parameters
    ----------
    geometry        Either the name of a folder under ``geometries/``
                    (e.g. "VX4_HOVER_OPT"), OR a direct path to a blade-data
                    ``.json`` file anywhere on disk (3-D strip / CADO format).
                    With a direct path, ``json_file`` and ``geometry_root`` are
                    ignored and the result is labelled by the file stem.
    json_file       Blade-data JSON inside that folder (used only when
                    ``geometry`` is a folder name).
    sim             Optional path to a SimParam.json giving rpm / v_inf /
                    blade_count / rotor_diameter.  Explicit keyword arguments
                    override anything read from this file.
    rpm, v_inf, blade_count, rotor_diameter
                    Operating point.  Required unless supplied via ``sim``.
    model           Optional override for the model bundle path.

    Returns
    -------
    BEMTResult with thrust (N), torque (N.m), power (W) and per-section data.

    Example
    -------
    >>> import BEMT
    >>> res = BEMT.run("VX4_HOVER_OPT", rpm=8300, v_inf=0,
    ...                blade_count=5, rotor_diameter=0.30)
    >>> # ...or straight from a JSON file anywhere on disk:
    >>> res = BEMT.run("/path/to/MyBlade.json", sim="SimParam.json")
    >>> print(res)
    """
    params = _resolve_params(sim, rpm, v_inf, blade_count, rotor_diameter)
    geom, label = _resolve_geometry(geometry, json_file, unit, geometry_root)

    bundle = load_model(model)
    profiles = model_profiles(geom, params, bundle)

    rotor = SingleProp(
        v_inf=params.v_inf, rpm=params.rpm, blade_count=params.blade_count,
        rotor_diameter=params.rotor_diameter, chords=geom.chords,
        radii=geom.radii, pitches=geom.pitches, profile=profiles,
    )
    rotor.calculate()

    return _result(rotor, label, params)


def run_xfoil(
    geometry: str,
    *,
    json_file: str = "BladeData-1.json",
    unit: str = "m",
    geometry_root: str | Path | None = None,
    sim: str | Path | None = None,
    rpm: float | None = None,
    v_inf: float | None = None,
    blade_count: int | None = None,
    rotor_diameter: float | None = None,
    generate_missing: bool = True,
    force_recompute: bool = False,
    aoa_start: float = -5.0,
    aoa_end: float = 20.0,
    aoa_step: float = 0.5,
    iterations: int = 200,
    xcrit: float = 0.6,
    xfoil_executable: str | Path | None = None,
):
    """Run BEMT using direct XFOIL polars instead of the trained model.

    Same call style and return type as :func:`run` (so ``res.thrust`` etc. work),
    but the aerodynamics come from XFOIL: any section without a cached polar is
    computed on the fly and saved to ``geometries/<name>/Polars/`` for reuse.

    ``geometry`` may be a folder name under ``geometries/`` or a direct path to a
    blade-data ``.json`` file. Set ``generate_missing=False`` to use only polars
    already on disk (never launch XFOIL); ``force_recompute=True`` regenerates all.

    Example
    -------
    >>> import BEMT
    >>> res = BEMT.run_xfoil("MyBlade.json", sim="SimParam.json")
    >>> print(res.thrust, res.power)
    """
    params = _resolve_params(sim, rpm, v_inf, blade_count, rotor_diameter)
    geom, label = _resolve_geometry(geometry, json_file, unit, geometry_root)

    rotor = SingleProp.from_geometry(
        geometry=geom,
        simulation=params,
        generate_missing_polars=generate_missing,
        force_recompute_polars=force_recompute,
        aoa_start=aoa_start,
        aoa_end=aoa_end,
        aoa_step=aoa_step,
        iterations=iterations,
        xcrit=xcrit,
        xfoil_executable=xfoil_executable,
    )
    rotor.calculate()

    return _result(rotor, label, params)


def _result(rotor, label, params):
    return BEMTResult(
        thrust=float(rotor.T),
        torque=float(rotor.Q),
        power=float(rotor.P),
        geometry=label,
        rpm=params.rpm,
        sections=rotor.get_section_aerodynamic_data(),
    )
