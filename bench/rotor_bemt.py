"""Adaptateur : BEMT utilisateur (BEMT/) -> interface rotor(b, Om, V, alpha) -> T, H, Q.

Polaires XFOIL en cache (X500_classic). Le modèle de substitution n'est pas utilisé
car il est entraîné sur VX4 : à 6300 tr/min, il sous-estime T d'environ 26 %.
H est obtenu en projetant la force tangentielle de chaque section sur l'axe vent,
puis en moyennant sur l'azimut (la BEMT ne fournit que T et Q).
"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "BEMT"))
from BEMT._api import _resolve_geometry                       # noqa: E402
from core.Simulation import SingleProp                       # noqa: E402
from core.SimulationParameters import SimulationParameters   # noqa: E402

GEOM = _resolve_geometry("X500_classic", "x500_strips.json", "m", None)[0]
D, NB = 0.26, 2   # m, pales (X500 V2)


def rotor_bemt(b, Om, V, alpha, n_az=12):
    """T, H, Q [N, N, N·m]. alpha > 0 nez en bas ; a_disk = angle axe rotor / vent."""
    p = SimulationParameters(v_inf=V, rpm=Om * 30 / np.pi, blade_count=NB,
                             rotor_diameter=D, a_disk=np.pi / 2 - alpha)
    rot = SingleProp.from_geometry(geometry=GEOM, simulation=p,
                                   generate_missing_polars=False)
    if V == 0:
        T, Q, _ = rot.calculate()
        return T, 0.0, Q
    rot.unlinear = True
    T = H = Q = 0.0
    for psi in np.linspace(0, 2 * np.pi, n_az, endpoint=False):
        rot.azimuth = psi
        t, q, _ = rot.calculate()
        T, Q = T + t, Q + q
        H += sum(s.Q / s.radius for s in rot.propeller.sections) * np.sin(psi)  # dF_t·sinψ
    return T / n_az, H / n_az, Q / n_az
