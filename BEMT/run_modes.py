#!/usr/bin/env python3
"""
run_modes.py  -  every way to run the BEMT, in one place.
=========================================================

Both back-ends return the SAME result object, so `res.thrust`, `res.power`,
`res.thrust_grams`, `res.sections` work either way.

  (A) The bundled TRAINED MODEL  -> BEMT.run(...)        no XFOIL, portable.
  (B) Direct XFOIL POLARS        -> BEMT.run_xfoil(...):
        B1. use polars already cached on disk      (no XFOIL run)
        B2. generate the missing polars with XFOIL (runs utils/xfoil.exe)
        B4. run XFOIL straight from a .json file path

Run it:
    python examples/run_modes.py            # all the no-XFOIL modes (fast)
    python examples/run_modes.py --xfoil    # also run the XFOIL-generating modes (slow)

Everything is anchored to the repo root, so it works from any directory.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

# --- make the project importable no matter where this is launched from -------
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import BEMT

GEOM = "VX4_HOVER_OPT"                       # has both a JSON and cached Polars/
JSON = "BladeData-1.json"
SIM = ROOT / "SimParam.json"                 # rpm / v_inf / blade_count / diameter
XFOIL = ROOT / "utils" / "xfoil.exe"


def header(title: str):
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


# =============================================================================
# (A) TRAINED MODEL  -  the everyday path, no XFOIL
# =============================================================================
def a1_model_inline():
    """A1. Trained model, operating point given inline."""
    header("A1  trained model | inline operating point")
    res = BEMT.run(GEOM, rpm=8300, v_inf=0, blade_count=5, rotor_diameter=0.30)
    print(res)
    print(f"   thrust={res.thrust:.3f} N   power={res.power:.1f} W   "
          f"({res.thrust_grams:.0f} g)")


def a2_model_from_simfile():
    """A2. Trained model, operating point read from a SimParam.json."""
    header("A2  trained model | operating point from SimParam.json")
    res = BEMT.run(GEOM, sim=SIM)
    print(res)


def a3_model_direct_json_path():
    """A3. Trained model, geometry given as a direct .json path (no folder needed)."""
    header("A3  trained model | direct path to a .json file")
    json_path = ROOT / "geometries" / GEOM / JSON
    res = BEMT.run(str(json_path), sim=SIM)
    print(res)


def a4_model_sections():
    """A4. Trained model, per-section aerodynamic breakdown."""
    header("A4  trained model | per-section breakdown")
    res = BEMT.run(GEOM, sim=SIM)
    print(res)
    print("\n  sec  r/R   pitch   aoa    phi    Cl     Cd      Re      dT(N)")
    for d in res.sections:
        print(f"  {d['section_index']:>3} {d['r_over_R']:.2f} "
              f"{d['pitch_deg']:6.1f} {d['aoa_deg']:6.1f} {d['phi_deg']:6.1f} "
              f"{d['cl']:6.3f} {d['cd']:6.4f} {d['re']:8.0f} {d['dT_N']:7.3f}")


def a5_json_section_table():
    """A5. Run on a .json file and print pitch / aoa / dT / dQ / Re per section."""
    header("A5  run on a .json file | per-section pitch / aoa / dT / dQ / Re")
    json_path = ROOT / "geometries" / GEOM / JSON        # any 3-D blade-data .json
    res = BEMT.run(str(json_path), sim=SIM)              # use run_xfoil(...) for XFOIL
    print(res)
    print("\n  sec   pitch     aoa      dT(N)      dQ(Nm)        Re")
    print("  ---  -------  -------  ---------  ----------  ---------")
    for d in res.sections:
        print(f"  {d['section_index']:>3}  {d['pitch_deg']:7.2f}  {d['aoa_deg']:7.2f}  "
              f"{d['dT_N']:9.4f}  {d['dQ_Nm']:10.5f}  {d['re']:9.0f}")
    print(f"  ---  totals  -> T={res.thrust:.3f} N   Q={res.torque:.4f} Nm   "
          f"P={res.power:.1f} W")


# =============================================================================
# (B) DIRECT XFOIL POLARS  -  same res.thrust result, via BEMT.run_xfoil
# =============================================================================
def b1_existing_polars():
    """B1. Use polars already cached in geometries/<geom>/Polars/ (no XFOIL run).

    generate_missing=False means: never launch XFOIL; just read the
    <geom>_strip_NNN.dat files already on disk (auto-discovered)."""
    header("B1  XFOIL polars | use existing cached .dat (no XFOIL run)")
    res = BEMT.run_xfoil(GEOM, sim=SIM, generate_missing=False)
    print(res)
    print(f"   thrust={res.thrust:.3f} N   power={res.power:.1f} W")


def b2_generate_with_xfoil():
    """B2. Generate the MISSING polars with XFOIL, then run.

    generate_missing=True: for any section without a cached .dat, XFOIL is invoked
    at the local Reynolds number and saved to geometries/<geom>/Polars/ for reuse.
    Existing files are kept; pass force_recompute=True to overwrite. SLOW."""
    header("B2  XFOIL polars | generate missing with XFOIL (SLOW)")
    if not XFOIL.exists():
        print(f"   skipped: XFOIL not found at {XFOIL}")
        return
    res = BEMT.run_xfoil(
        GEOM, sim=SIM,
        generate_missing=True,        # <-- call XFOIL for missing sections
        force_recompute=False,        # set True to regenerate ALL of them
        aoa_start=-5.0, aoa_end=20.0, aoa_step=0.5,
        iterations=200, xcrit=0.6,
        xfoil_executable=XFOIL,
    )
    print(res)
    print(f"   thrust={res.thrust:.3f} N   power={res.power:.1f} W")


def b4_xfoil_from_json_path():
    """B4. Run XFOIL on just a .json file (direct path, no folder needed)."""
    header("B4  XFOIL polars | straight from a .json file")
    json_path = ROOT / "geometries" / GEOM / JSON
    res = BEMT.run_xfoil(str(json_path), sim=SIM, generate_missing=True,
                         xfoil_executable=XFOIL if XFOIL.exists() else None)
    print(res)
    print(f"   thrust={res.thrust:.3f} N   power={res.power:.1f} W")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--xfoil", action="store_true",
                    help="also run the XFOIL-generating modes B2/B4 (slow)")
    args = ap.parse_args()

    # (A) trained model -- always fast, no XFOIL
    a1_model_inline()
    a2_model_from_simfile()
    a3_model_direct_json_path()
    a4_model_sections()
    a5_json_section_table()

    # (B) direct XFOIL polars
    b1_existing_polars()
    if args.xfoil:
        b2_generate_with_xfoil()
        b4_xfoil_from_json_path()
    else:
        header("B2/B4  XFOIL polars | generate with XFOIL  (SKIPPED)")
        print("   re-run with  --xfoil  to actually invoke XFOIL for missing polars")

    print("\nDone.")


if __name__ == "__main__":
    main()
