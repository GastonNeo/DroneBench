"""Attitude imposée par le banc (ψ, φ, θ) à 13 m/s, Ω_hover BEMT, sillage par rotor (E28–E32).

Sortie : Results/4_attitude_13ms.png + CSV. Cellule : poids taré à attitude nulle ;
moteurs (E27, E32) : poids du drone inclus, poids du banc exclu.
"""
import sys, warnings
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
from loads import Bench, wrench, LIMITS, JOINTS, joint_torques, omega_hover, G0, rotor_frame, hubs  # noqa: E402
from wake import dv_rotors                       # noqa: E402
from rotor_bemt import rotor_bemt                # noqa: E402

warnings.filterwarnings("ignore")
OUT = Path(__file__).resolve().parents[1] / "Results"
V = 13.0
SWEEPS = {"ψ (yaw)": (0, [0, 15, 30, 45, 90]), "φ (roll)": (1, [-10, 0, 10]), "θ (pitch)": (2, [-10, 0, 10])}
COL = ["#2a78d6", "#eb6834", "#1baf7a"]          # catégorielles, validées

b = Bench()
Om = omega_hover(b, rotor_bemt)
print(f"Ω_hover BEMT = {Om*30/np.pi:.0f} tr/min, V = {V:.0f} m/s, sillage par rotor (E31)")
print(f"{'cas':>10} {'β°':>5} {'Δv1..4 [m/s]':>23} {'Fx':>6} {'Fy':>6} {'Fz':>6} {'Mx':>6} {'My':>6} {'Mz':>6}"
      f" {'τyaw':>6} {'τroll':>6} {'τpit':>6} {'max%':>5}")
res, cache = {}, {}
for name, (k, angs) in SWEEPS.items():
    for a in angs:
        att = np.zeros(3); att[k] = np.radians(a)
        key = tuple(att)
        if key not in cache:
            _, Vr, ar, beta = rotor_frame(V, att)
            dv = dv_rotors(b, [rotor_bemt(b, Om, Vr, ar)[0]] * 4, hubs(b), Vr, ar, beta)  # E31
            wg = wrench(b, Om, V, rot=rotor_bemt, att=att, dv=dv, tare=False)   # poids réel
            w = wg + np.array([0, 0, b.m * G0, 0, 0, 0])                         # tare attitude nulle
            tau = joint_torques(wg, att)
            u = 100 * np.max(np.abs(w) / LIMITS)
            cache[key] = (w, tau, u)
            print(f"{name[0]}={a:4d}°   {np.degrees(beta):5.0f} " + " ".join(f"{x:5.2f}" for x in dv)
                  + " " + " ".join(f"{x:6.2f}" for x in (*w, *tau)) + f" {u:5.1f}")
        res[name, a] = cache[key]

# Figure : 6 composantes cellule + 3 couples moteurs + taux d'utilisation
panels = [(0, "Fx cellule", "N"), (1, "Fy cellule", "N"), (2, "Fz cellule", "N"), (6, "τ yaw", "N·m"),
          (7, "τ roll", "N·m"), (3, "Mx cellule", "N·m"), (4, "My cellule", "N·m"), (5, "Mz cellule", "N·m"),
          (8, "τ pitch", "N·m"), (9, "Gamma SI-130-10, max ∣w∣/pleine échelle", "%")]
fig, axs = plt.subplots(2, 5, figsize=(19, 6.6))
for ax, (i, lab, unit) in zip(axs.flat, panels):
    for (name, (_, angs)), c in zip(SWEEPS.items(), COL):
        y = [np.concatenate([res[name, a][0], res[name, a][1], [res[name, a][2]]])[i] for a in angs]
        ax.plot(angs, y, "o-", color=c, lw=2, ms=4, label=name)
    ax.set_title(lab, loc="left", fontsize=10)
    ax.set_ylabel(unit); ax.set_xlabel("angle [°]")
    ax.grid(color="#e5e5e5", lw=0.6); ax.spines[["top", "right"]].set_visible(False)
    ax.axhline(0, color="#999", lw=0.6)
axs.flat[0].legend(frameon=False, fontsize=8, loc="best")
fig.suptitle(f"Banc — attitude imposée, V = {V:.0f} m/s, Ω_hover = {Om*30/np.pi:.0f} tr/min "
             "(cellule tarée à attitude nulle ; moteurs : poids drone inclus)", x=0.01, ha="left",
             fontsize=12, fontweight="bold")
fig.tight_layout()
OUT.mkdir(exist_ok=True)
fig.savefig(OUT / "4_attitude_13ms.png", dpi=130)
rows = [[k, a, *res[n, a][0], *res[n, a][1], res[n, a][2]] for n, (k, angs) in SWEEPS.items() for a in angs]
np.savetxt(OUT / "4_attitude_13ms.csv", rows, delimiter=",", fmt="%.5g", comments="",
           header="axe(0=psi;1=phi;2=theta),angle_deg,Fx,Fy,Fz,Mx,My,Mz," +
           ",".join(f"tau_{k}_Nm" for k in JOINTS) + ",util_SI130_pct")
print(f"-> {OUT / '4_attitude_13ms.png'}")
