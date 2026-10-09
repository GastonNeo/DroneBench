"""
Self-contained Cl/Cd surrogate inference (no XFOIL, no training deps).

This module holds everything the BEMT package needs *at run time* to turn a
blade's airfoil point clouds into Profil-compatible Cl/Cd objects, using the
gradient-boosted model bundled in this package (polar_model.joblib).

The training script (train_polar_model.py, run once) is the only place XFOIL is
touched; importing BEMT never imports XFOIL.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from core.Profil import Profil

RHO = 1.225
MU = 1.81e-5
N_CHORD = 90  # chordwise stations used to decompose an airfoil


# ---------------------------------------------------------------------------
# Airfoil decomposition: closed loop -> camber(x) + thickness(x)
# (kept identical to experiment_neighbour_polars.decompose_airfoil so the model
#  sees the same shape descriptor it was trained on)
# ---------------------------------------------------------------------------
def decompose_airfoil(points_xy, n: int = N_CHORD):
    pts = np.asarray(points_xy, dtype=float)
    x, y = pts[:, 0], pts[:, 1]

    i_le = int(np.argmin(x))
    rolled = np.roll(np.arange(len(pts)), -i_le)
    xr, yr = x[rolled], y[rolled]

    i_te = int(np.argmax(xr))
    s1x, s1y = xr[: i_te + 1], yr[: i_te + 1]
    s2x, s2y = xr[i_te:], yr[i_te:]

    xs = (1.0 - np.cos(np.linspace(0.0, np.pi, n))) / 2.0

    def interp_surface(sx, sy):
        order = np.argsort(sx)
        return np.interp(xs, sx[order], sy[order])

    ya = interp_surface(s1x, s1y)
    yb = interp_surface(s2x, s2y)

    y_up = np.maximum(ya, yb)
    y_lo = np.minimum(ya, yb)
    camber = 0.5 * (y_up + y_lo)
    thickness = y_up - y_lo
    return xs, camber, thickness


def shape_features(points_xy, feature_x, cs: float = 1.0, ts: float = 1.0):
    """Geometry-agnostic shape descriptor: camber + thickness at fixed stations."""
    xs, camber, thickness = decompose_airfoil(points_xy)
    cam_k = np.interp(feature_x, xs, camber) * cs
    thk_k = np.interp(feature_x, xs, thickness) * ts
    return np.concatenate([cam_k, thk_k])


def compute_re(rpm: float, radius: float, chord: float):
    omega = rpm * 2.0 * math.pi / 60.0
    v_local = omega * radius
    return RHO * v_local * chord / MU


# ---------------------------------------------------------------------------
# Profil-compatible wrapper backed by the trained models
# ---------------------------------------------------------------------------
class ModelProfile(Profil):
    """Predicts Cl/Cd from the trained GBT models for a fixed airfoil shape.

    A Cl/Cd(alpha) curve is pre-evaluated on a grid at a representative Re for
    speed, and re-evaluated if the BEMT queries a Re far from the cached one.
    """

    def __init__(self, shape, cl_model, cd_model, re_repr,
                 alpha_grid=np.arange(-15, 25.01, 0.5)):
        self.shape = np.asarray(shape, float)
        self.cl_model = cl_model
        self.cd_model = cd_model
        self.alpha_grid = alpha_grid
        self.re_ref = None
        self.donnees = None
        self._set_curves(re_repr)

    def _set_curves(self, re):
        n = len(self.alpha_grid)
        feats = np.column_stack([
            np.tile(self.shape, (n, 1)),
            np.full(n, re),
            self.alpha_grid,
        ])
        self._cl = self.cl_model.predict(feats)
        self._cd = np.maximum(self.cd_model.predict(feats), 0.0)
        self._re = re

    def calcul(self):
        pass

    def getCl(self, alpha, re=None):
        if re is not None and abs(re - self._re) / max(self._re, 1.0) > 0.15:
            self._set_curves(re)
        a = np.degrees(self.angle_normalise(alpha))
        return float(np.interp(a, self.alpha_grid, self._cl))

    def getCd(self, alpha, re=None):
        if re is not None and abs(re - self._re) / max(self._re, 1.0) > 0.15:
            self._set_curves(re)
        a = np.degrees(self.angle_normalise(alpha))
        return max(float(np.interp(a, self.alpha_grid, self._cd)), 0.0)


# ---------------------------------------------------------------------------
# Model loading + per-section profile construction
# ---------------------------------------------------------------------------
_DEFAULT_MODEL_PATH = Path(__file__).resolve().parent / "polar_model.joblib"


def load_model(model_path: str | Path | None = None):
    """Load the bundled (or a given) Cl/Cd model bundle."""
    import joblib

    path = Path(model_path) if model_path is not None else _DEFAULT_MODEL_PATH
    if not path.exists():
        raise FileNotFoundError(
            f"Trained model not found: {path}\n"
            "Train it once with:  python train_polar_model.py"
        )
    return joblib.load(path)


def model_profiles(geometry, params, bundle):
    """Build one ModelProfile per blade section from a loaded model bundle."""
    cl_model, cd_model = bundle["cl"], bundle["cd"]
    feature_x = np.asarray(bundle["feature_x"], float)
    profs = []
    for s in geometry.sections:
        shape = shape_features(s.points_xy, feature_x, 1.0, 1.0)
        re_repr = compute_re(params.rpm, s.radius, s.chord)
        profs.append(ModelProfile(shape, cl_model, cd_model, re_repr))
    return profs
