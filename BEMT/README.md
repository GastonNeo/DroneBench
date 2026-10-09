# BEMT Simulator

A Python implementation of Blade Element Momentum Theory (BEMT) for analyzing the
aerodynamic performance of drone propellers.

Given a propeller geometry and an operating point, it computes **thrust, torque and
power** by splitting the blade into radial sections and solving the BEMT equations
for each one. It accounts for airfoil Cl/Cd polars, the Prandtl tip-loss factor,
axial/tangential induction, the hover singularity (V∞ = 0), and azimuth-dependent
loading at non-zero disk angle of attack.

---

## Two ways to get airfoil aerodynamics

1. **Trained surrogate model (recommended, no XFOIL).** A gradient-boosted Cl/Cd
   model ships inside the `BEMT/` package (`BEMT/polar_model.joblib`). It was trained
   on the neighbourhood of the **VX4_HOVER** blade and generalises to nearby blades
   such as **VX4_HOVER_OPT**. This is what makes the module portable: copy the folder
   to another machine and run, with **no XFOIL and no crashes**.

2. **Direct XFOIL.** The classic path still works (`SingleProp.from_geometry(...,
   generate_missing_polars=True)`), useful for generating fresh training data.

---

## Quick start — `import BEMT`

The whole thing is an importable package. Put the project folder on your path and:

```python
import BEMT

# Operating point given inline:
res = BEMT.run("VX4_HOVER_OPT", rpm=8300, v_inf=0, blade_count=5, rotor_diameter=0.30)
print(res)                       # BEMT[VX4_HOVER_OPT @ 8300 rpm]  T=... N  Q=... Nm  P=... W
print(res.thrust, res.power)     # plain floats (N, W)
print(res.thrust_grams)          # thrust in grams

# ...or driven from a SimParam.json:
res = BEMT.run("VX4_HOVER_OPT", sim="SimParam.json")

# Per-section breakdown:
for s in res.sections:
    print(s["r_over_R"], s["aoa_deg"], s["cl"], s["cd"], s["dT_N"])
```

`BEMT.run` loads the geometry from `geometries/<name>/`, builds one surrogate polar
per section from the bundled model, solves the rotor, and returns a `BEMTResult`
(`thrust`, `torque`, `power`, `thrust_grams`, `sections`).

### Command-line equivalent

```
python run_bemt_model.py --geometry VX4_HOVER_OPT --sim SimParam.json
python run_bemt_model.py --geometry VX4_HOVER_OPT --rpm 7000 --sections
```

---

## Operating point — `SimParam.json`

```json
{
    "rpm": 8300,
    "v_inf": 0,
    "blade_count": 5,
    "rotor_diameter": 0.3,
    "unit": "m"
}
```

---

## Geometry definition

Geometries live in `geometries/<name>/`. Each propeller folder contains a blade-data
JSON (3-D strip / CADO format, e.g. `BladeData-1.json`) describing per-section radius,
chord, pitch and airfoil point clouds, plus a `Polars/` subfolder for any cached XFOIL
polars. `BEMT.run` only needs the JSON — the aerodynamics come from the trained model.

---

## Training / refreshing the surrogate model

The model only needs to be (re)trained when you change the baseline blade or want a
different XFOIL setting (e.g. `xcrit`). Two steps:

```bash
# 1. Generate the Cl/Cd neighbourhood of the baseline blade with XFOIL.
#    (camber_scale x thickness_scale x Re x alpha, cached + resumable)
python experiment_neighbour_polars.py --geometry VX4_HOVER --build-only

# 2. Train the gradient-boosted Cl/Cd model and bundle it into BEMT/.
python train_polar_model.py
```

Step 1 sweeps each section's airfoil over camber/thickness scales and Reynolds numbers
and stores clean polars in `geometries/VX4_HOVER/NeighbourPolars/` (crashed XFOIL cases
are quarantined and replaced by the nearest working polar, so the table is "steady").
The XFOIL `xcrit` and alpha sweep are set at the top of
`experiment_neighbour_polars.py` (currently `XCRIT = 0.6`).

Step 2 reads those polars, turns each airfoil into a geometry-agnostic shape descriptor
(camber + thickness at fixed chord stations) plus `(Re, alpha)`, fits two
`HistGradientBoostingRegressor` models (Cl and Cd), saves them to
`BEMT/polar_model.joblib`, and prints a held-out check against **VX4_HOVER_OPT**.

---

## Requirements

```
numpy
scipy
scikit-learn      # surrogate model
joblib            # model (de)serialisation
matplotlib        # plotting / GUI
PyQt5             # only for the GUI (bemt_gui.py)
```

XFOIL (`utils/xfoil.exe`) is only needed to (re)train the model, never to run it.

---

## Files overview

| Path | Purpose |
|---|---|
| `BEMT/` | Importable package: `BEMT.run(...)` + bundled surrogate model |
| `BEMT/polar_model.joblib` | Trained Cl/Cd model (ships with the package) |
| `run_bemt_model.py` | Command-line wrapper around `BEMT.run` |
| `train_polar_model.py` | Train the surrogate model (run once) |
| `experiment_neighbour_polars.py` | Build the XFOIL Cl/Cd neighbourhood matrix |
| `core/Simulation.py` | BEMT solver (`SingleProp`) |
| `core/Section.py` | Per-section BEMT equations |
| `core/Geometry.py` | Geometry loading + polar management |
| `core/SimulationParameters.py` | Operating-conditions dataclass |
| `bemt_gui.py` | PyQt5 graphical interface |

---

## Notes on the physics

- The φ equilibrium is reformulated to avoid the standard BEMT singularity at hover
  (V∞ = 0): the hover solution is `sin²(φ) = σ·CT / (4·F)`.
- In moderate descent (negative V∞), thrust typically increases because the reduced
  inflow angle raises the effective angle of attack. This is physically correct.
- At descent speeds near the induced velocity (~2–5 m/s) the rotor enters the vortex
  ring state, where BEMT is not valid — disregard results there.
