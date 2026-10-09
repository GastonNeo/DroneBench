"""
BEMT - Blade Element Momentum Theory for drone propellers.
==========================================================

A self-contained module to run BEMT with a bundled, XFOIL-free aerodynamic
surrogate (gradient-boosted Cl/Cd model trained around the VX4_HOVER blade).

Copy this folder onto any machine, put its parent on the path, and run::

    import BEMT

    res = BEMT.run("VX4_HOVER_OPT", rpm=8300, v_inf=0,
                   blade_count=5, rotor_diameter=0.30)
    print(res)                  # Thrust / Torque / Power
    print(res.thrust_grams)     # thrust in grams
    for s in res.sections:      # per-section breakdown
        ...

    # or drive it from a SimParam.json:
    res = BEMT.run("VX4_HOVER_OPT", sim="SimParam.json")

No XFOIL is required at run time; the model ships inside the package
(``BEMT/polar_model.joblib``).  Retrain it (once) with
``python train_polar_model.py``.
"""
from __future__ import annotations

import sys
from pathlib import Path

# Make the sibling `core` / `utils` packages importable no matter the cwd, so
# `import BEMT` works after copying the project folder onto another machine.
_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from core.Geometry import Geometry
from core.Simulation import SingleProp
from core.SimulationParameters import SimulationParameters

from ._api import BEMTResult, GEOMETRIES_DIR, MODEL_PATH, run, run_xfoil
from ._polarmodel import ModelProfile, load_model, model_profiles, shape_features

__version__ = "1.0.0"

__all__ = [
    "run",
    "run_xfoil",
    "BEMTResult",
    "Geometry",
    "SimulationParameters",
    "SingleProp",
    "ModelProfile",
    "load_model",
    "model_profiles",
    "shape_features",
    "GEOMETRIES_DIR",
    "MODEL_PATH",
]
