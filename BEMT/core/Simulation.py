from __future__ import annotations

import logging
import math
import csv
from typing import Sequence
from pathlib import Path

import numpy as np
from scipy import optimize

from core.Geometry import Geometry
from core.ProfileLibrary import AirfoilLibrary
from core.Propeller import Propeller
from core.Profil import Profil
from core.SimulationParameters import SimulationParameters


class SingleProp:
    """
    Simulator for one propeller using Blade Element Momentum Theory.
    """

    def __init__(
        self,
        v_inf,
        rpm,
        blade_count,
        rotor_diameter,
        chords,
        radii,
        pitches,
        profile,
        unlinear: bool = False,
    ):
        self.v_inf = float(v_inf)
        self.rpm = float(rpm)
        self.blade_count = int(blade_count)
        self.rotor_diameter = float(rotor_diameter)
        self.chords = [float(v) for v in chords]
        self.radii = [float(v) for v in radii]
        self.pitches = [float(v) for v in pitches]

        if not (len(self.chords) == len(self.radii) == len(self.pitches)):
            raise ValueError("chords, radii and pitches must have the same size.")

        self.section_profiles = self._resolve_profiles(profile, len(self.chords))

        self.unlinear = bool(unlinear)

        self.azimuth = 0.0
        self.a_disk = 0.0

        self.rho = self.compute_rho(0.0)

        self.load_simulator()

    @staticmethod
    def _resolve_profiles(profile, section_count: int):
        if isinstance(profile, str):
            base_profile = AirfoilLibrary.get_profile(profile)
            return [base_profile] * section_count

        if isinstance(profile, Profil):
            return [profile] * section_count

        if isinstance(profile, Sequence):
            if len(profile) != section_count:
                raise ValueError("Profile list size must match section count.")
            resolved = []
            for item in profile:
                if isinstance(item, str):
                    resolved.append(AirfoilLibrary.get_profile(item))
                elif isinstance(item, Profil):
                    resolved.append(item)
                else:
                    raise TypeError(
                        "Each profile entry must be a profile name or a Profil instance."
                    )
            return resolved

        raise TypeError(
            "profile must be a profile name, Profil instance, or a section-sized list."
        )

    @classmethod
    def from_geometry(
        cls,
        geometry: Geometry,
        simulation: SimulationParameters,
        shared_polar: bool = False,
        shared_polar_file=None,
        section_polar_files=None,
        generate_missing_polars: bool = True,
        force_recompute_polars: bool = False,
        aoa_start: float = -15.0,
        aoa_end: float = 20.0,
        aoa_step: float = 0.5,
        iterations: int = 100,
        xcrit: float = 9.0,
        xfoil_executable=None,
    ):
        geometry.ensure_polars(
            simulation=simulation,
            shared_polar=shared_polar,
            shared_polar_file=shared_polar_file,
            section_polar_files=section_polar_files,
            generate_missing=generate_missing_polars,
            force_recompute=force_recompute_polars,
            aoa_start=aoa_start,
            aoa_end=aoa_end,
            aoa_step=aoa_step,
            iterations=iterations,
            xcrit=xcrit,
            xfoil_executable=xfoil_executable,
        )
        profiles = geometry.load_profiles()

        rotor = cls(
            v_inf=simulation.v_inf,
            rpm=simulation.rpm,
            blade_count=simulation.blade_count,
            rotor_diameter=simulation.rotor_diameter,
            chords=geometry.chords,
            radii=geometry.radii,
            pitches=geometry.pitches,
            profile=profiles,
            unlinear=simulation.unlinear,
        )
        rotor.azimuth = simulation.azimuth
        rotor.a_disk = simulation.a_disk
        rotor.load_simulator()
        return rotor

    def load_simulator(self):
        """Initialize the propeller and per-run accumulators."""
        self.T = 0.0
        self.Q = 0.0
        self.P = 0.0
        self.induced_velocities = []
        self.rotor_surface = math.pi * (self.rotor_diameter / 2) ** 2
        self.propeller = Propeller(
            self.rotor_diameter,
            self.blade_count,
            len(self.chords),
            self.radii,
            self.chords,
            self.pitches,
            self.section_profiles,
            self.rho,
            unlinear=self.unlinear,
            azimuth=self.azimuth,
            a_disk=self.a_disk,
        )
        self.advance_ratio = self.v_inf / (self.rotor_diameter * (self.rpm / 60))

        if self.unlinear and not getattr(self, "_unlinear_warned", False):
            self._unlinear_warned = True
            logging.warning(
                "You are in unlinear model. Set azimuth and a_disk before calculate()."
            )

    def compute_rho(self, altitude):
        """Compute air density as a function of altitude."""
        return 352.995 * ((1 - 0.0000225577 * altitude) ** 5.25516) / (
            288.15 - 0.0065 * altitude
        )

    def calculate(self):
        """Compute thrust, torque, and power."""
        self.load_simulator()
        omega = self.rpm * 2 * math.pi / 60
        T, Q = 0.0, 0.0

        for section in self.propeller.sections:
            v = self.v_inf if section.radius < self.rotor_diameter / 2 else 0.0
            phi = self.solve_phi(section, v, omega)

            dT, dQ = section.forces(phi, v, omega)

            T += dT
            Q += dQ

        self.Q, self.T, self.P = Q, T, Q * omega
        return T, Q, self.P

    def calculate_azimuthal_average(self, n_azimuth: int = 12):
        """Average thrust/torque/power over n_azimuth equally spaced blade positions.

        Forces unlinear mode so that azimuth and a_disk are active in phi_function.
        Restores original azimuth and unlinear state afterwards.
        """
        saved_azimuth  = self.azimuth
        saved_unlinear = self.unlinear
        self.unlinear  = True

        T_sum = Q_sum = P_sum = 0.0
        for az in np.linspace(0, 2 * math.pi, n_azimuth, endpoint=False):
            self.azimuth = az
            T, Q, P = self.calculate()
            T_sum += T
            Q_sum += Q
            P_sum += P

        self.azimuth  = saved_azimuth
        self.unlinear = saved_unlinear
        return T_sum / n_azimuth, Q_sum / n_azimuth, P_sum / n_azimuth

    def compute_induced_velocity(self, dT, radius):
        return math.sqrt((2 * dT) / (self.rho * (math.pi * radius**2)))

    # Physical inflow-angle window for a propeller in hover/climb: the relative
    # wind comes from rotation + (downward) induced axial velocity, so the inflow
    # angle satisfies 0 < phi < 90deg and alpha = pitch - phi stays physical.
    # Restricting the solver to (PHI_MIN, PHI_MAX) excludes the spurious phi≈pi
    # root (alpha≈-130deg, negative torque) the wide bracket used to admit.
    PHI_MIN = 1.0e-3
    PHI_MAX = 0.5 * math.pi

    def solve_phi(self, section, velocity, omega):
        """Solve phi_function for the physical inflow angle in (0, pi/2).

        Prefer a bracketed bisection; if no sign change exists in the window
        (stalled / tip-loss-dominated section) fall back to the least-residual
        phi within the SAME physical window - never negative, never > pi/2.
        """
        args = (omega, velocity, self.azimuth, self.a_disk)
        lo, hi = self.PHI_MIN, self.PHI_MAX
        try:
            f_lo = section.phi_function(lo, *args)
            f_hi = section.phi_function(hi, *args)
            if math.isfinite(f_lo) and math.isfinite(f_hi) and f_lo * f_hi < 0.0:
                return optimize.bisect(section.phi_function, lo, hi, args=args)
        except Exception:
            pass
        return self.brute_force_solver(section, velocity, omega)

    def brute_force_solver(self, section, velocity, omega, n=1800):
        phis = np.linspace(self.PHI_MIN, self.PHI_MAX, n)
        residuals = np.array(
            [
                section.phi_function(phi, omega, velocity, self.azimuth, self.a_disk)
                for phi in phis
            ],
            dtype=float,
        )
        residuals = np.where(np.isfinite(residuals), np.abs(residuals), np.inf)
        return phis[int(np.argmin(residuals))]

    def get_section_aerodynamic_data(self):
        """
        Return per-section aerodynamic quantities after `calculate()`.
        """
        rotor_radius = self.rotor_diameter / 2.0
        data = []
        for i, section in enumerate(self.propeller.sections):
            pitch_deg = math.degrees(section.pitch)
            phi_deg = math.degrees(getattr(section, "phi", 0.0))
            aoa_deg = float(getattr(section, "aoa", pitch_deg - phi_deg))
            data.append(
                {
                    "section_index": i,
                    "radius_m": float(section.radius),
                    "r_over_R": float(section.radius / rotor_radius),
                    "pitch_deg": float(pitch_deg),
                    "phi_deg": float(phi_deg),
                    "aoa_deg": aoa_deg,
                    "cl": float(getattr(section, "Cl", 0.0)),
                    "cd": float(getattr(section, "Cd", 0.0)),
                    "ct": float(getattr(section, "CT", 0.0)),
                    "cq": float(getattr(section, "CQ", 0.0)),
                    "re": float(getattr(section, "Re", 0.0)),
                    "dT_N": float(getattr(section, "T", 0.0)),
                    "dQ_Nm": float(getattr(section, "Q", 0.0)),
                }
            )
        return data

    def save_section_aerodynamic_data_csv(self, csv_path):
        """
        Save per-section aerodynamic quantities, including `aoa_deg`, to CSV.
        """
        rows = self.get_section_aerodynamic_data()
        if not rows:
            return

        csv_path = Path(csv_path)
        if csv_path.parent and not csv_path.parent.exists():
            csv_path.parent.mkdir(parents=True, exist_ok=True)

        fieldnames = list(rows[0].keys())
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)


class CoaxialProp:
    def __init__(
        self,
        v_inf,
        rpm_upper,
        rpm_lower,
        blade_count,
        rotor_diameter,
        chords,
        radii,
        pitches,
        profile_name,
    ):
        self.v_inf = v_inf
        self.rpm_upper = rpm_upper
        self.rpm_lower = rpm_lower
        self.blade_count = blade_count
        self.rotor_diameter = rotor_diameter
        self.chords = chords
        self.radii = radii
        self.pitches = pitches
        self.profile_name = profile_name
        self.load_simulator()

    def load_simulator(self):
        self.upper_rotor = SingleProp(
            self.v_inf,
            self.rpm_upper,
            self.blade_count,
            self.rotor_diameter,
            self.chords,
            self.radii,
            self.pitches,
            self.profile_name,
        )

        self.lower_rotor = SingleProp(
            self.v_inf,
            self.rpm_lower,
            self.blade_count,
            self.rotor_diameter,
            self.chords,
            self.radii,
            self.pitches,
            self.profile_name,
        )
