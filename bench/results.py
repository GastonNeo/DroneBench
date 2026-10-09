"""Balayage en régime : efforts locaux au drone (rotor) et au banc (cellule, base).

Sorties dans Results/ : PNG + CSV. Modèle : BEMT utilisateur + sillage avant (E21–E26).
α = 0, poids taré. Base = centre de l'articulation robolink (CAO), poids propre du banc exclu.
"""
import sys, warnings
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
from loads import Bench, wrench, LIMITS          # noqa: E402
from rotor_bemt import rotor_bemt                # noqa: E402
from wake import dv_rear                         # noqa: E402

warnings.filterwarnings("ignore")
OUT = Path(__file__).resolve().parents[1] / "Results"
RPM = np.arange(3000, 9001, 500)
VS = [0, 5, 9, 13]                               # m/s
COL = ["#6da7ec", "#2a78d6", "#1c5cab", "#0d366b"]  # rampe ordinale, validée
R_BO = np.array([0.337, -0.136, 1.037])          # m, base -> cellule (repère drone, CAO)

b = Bench()
rot = {}                                          # (V, rpm) -> [T, H, Q, Mr, T_ar]
cell, base = {}, {}
for V in VS:
    for n in RPM:
        Om = n * np.pi / 30
        T, H, Q, (Mr, _, _) = rotor_bemt(b, Om, V, 0.0)
        dv = dv_rear(b, T, V)[0] if V > 0 else 0.0
        T_ar = rotor_bemt(b, Om, V, 0.0, dv=dv)[0] if dv else T
        w = wrench(b, Om, V, rot=rotor_bemt, dv=dv)
        rot[V, n] = [T, H, Q, Mr, T_ar]
        cell[V, n] = w
        base[V, n] = np.r_[w[:3], w[3:] + np.cross(R_BO, w[:3])]   # M_B = M_O + r_BO × F
    print(f"V = {V:2d} m/s  ok")


def sweep(data, idx):
    return {V: np.array([data[V, n][idx] for n in RPM]) for V in VS}


def grid(panels, data, fname, title, nrow, ncol, lim=None):
    fig, axs = plt.subplots(nrow, ncol, figsize=(4.2 * ncol, 3.3 * nrow), sharex=True)
    for ax, (i, lab, unit) in zip(axs.flat, panels):
        for V, c in zip(VS, COL):
            ax.plot(RPM, sweep(data, i)[V], color=c, lw=2, label=f"V = {V} m/s")
        if max(np.abs(sweep(data, i)[V]).max() for V in VS) < 1e-6:     # bruit numérique
            for ln in ax.lines:
                ln.set_ydata(0 * ln.get_ydata())
            ax.set_ylim(-1, 1)
            ax.text(0.5, 0.6, "≡ 0 (symétrie)", transform=ax.transAxes, ha="center", color="#666")
        if lim is not None:
            ax.text(0.02, 0.04, f"pleine échelle ±{lim[i]:.0f}", transform=ax.transAxes,
                    fontsize=8, color="#666")
        ax.set_title(lab, loc="left", fontsize=10)
        ax.set_ylabel(unit)
        ax.grid(color="#e5e5e5", lw=0.6); ax.spines[["top", "right"]].set_visible(False)
        ax.axhline(0, color="#999", lw=0.6)
    for ax in axs.flat[len(panels):]:
        ax.set_visible(False)
    for ax in axs[-1]:
        ax.set_xlabel("régime [tr/min]")
    axs.flat[0].legend(frameon=False, fontsize=8, loc="best")
    fig.suptitle(title, x=0.01, ha="left", fontsize=12, fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT / fname, dpi=130)
    plt.close(fig)


def csv(data, cols, fname):
    rows = [[V, n, *data[V, n]] for V in VS for n in RPM]
    np.savetxt(OUT / fname, rows, delimiter=",", fmt="%.5g",
               header="V_ms,rpm," + ",".join(cols), comments="")


OUT.mkdir(exist_ok=True)
grid([(0, "T rotor avant", "N"), (4, "T rotor arrière (sillage avant)", "N"),
      (1, "H rotor (dans le plan)", "N"), (2, "Q couple rotor", "N·m"),
      (3, "M_r roulis de moyeu", "N·m")],
     rot, "1_drone_rotor_vs_rpm.png", "Drone — efforts locaux par rotor", 2, 3)
comp = [("Fx", "N"), ("Fy", "N"), ("Fz", "N"), ("Mx", "N·m"), ("My", "N·m"), ("Mz", "N·m")]
grid([(i, f"{c} cellule", u) for i, (c, u) in enumerate(comp)], cell,
     "2_cellule_vs_rpm.png", "Banc — torseur au centre de la cellule O (taré)", 2, 3, LIMITS)
grid([(i, f"{c} base", u) for i, (c, u) in enumerate(comp)], base,
     "3_base_vs_rpm.png", "Banc — torseur à la base (articulation robolink), poids du banc exclu", 2, 3)
csv(rot, ["T_av_N", "H_N", "Q_Nm", "Mr_Nm", "T_ar_N"], "1_drone_rotor_vs_rpm.csv")
csv(cell, [c for c, _ in comp], "2_cellule_vs_rpm.csv")
csv(base, [c for c, _ in comp], "3_base_vs_rpm.csv")
print(f"-> {OUT}")
