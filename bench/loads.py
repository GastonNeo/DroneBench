"""Efforts sur la cellule 6 axes du banc DroneBench (Holybro X650).

Modèle quasi-statique : BEMT à inflow uniforme (Leishman, 2006, ch. 2-3)
+ traînée de cellule ½ρV²·CdA + poids projeté. Repère capteur = repère drone :
x avant, y gauche, z haut (axe poussée), origine au centre de la cellule.
Vent amont, face au nez (entre bras 1 et 3) ; alpha > 0 = nez en bas (convention Leishman).
"""
import numpy as np
from dataclasses import dataclass

G0, RHO = 9.81, 1.225

@dataclass
class Bench:
    # --- Drone : masse confirmée ; hélices X500 V2 (1045) ---
    m: float = 2.5          # kg, masse au-dessus de la cellule
    R: float = 0.130        # m, rayon (D = 0.26 m, cf. BEMT/X500.py)
    c: float = 0.018        # m, corde moyenne (mesurée)
    Nb: int = 2             # pales
    th75: float = np.radians(10.8)  # pas 4.5" à 0.75R : atan(p / 2π·0.75R)
    a: float = 5.7          # 1/rad, pente Cl
    cd0: float = 0.015      # traînée profil
    kappa: float = 1.15     # facteur puissance induite
    # CdA a priori = Cd·S, S projetée CAO (drone + interface), Cd ≈ 1.1 corps
    # non profilé, Re ~ 1e3–2e4 ; à remplacer par la mesure (bench/cda.py)
    CdA: tuple = (0.037, 0.037, 0.079)  # m² (x, y, z) ; S = 0.034, 0.034, 0.072
    # --- Géométrie CAO (mm -> m), réf. centre cellule ---
    d: float = 0.230        # m, bras moteur projeté sur x et y (X, empattement 650)
    h_r: float = 0.194      # m, plan rotor au-dessus de la cellule
    h_f: float = 0.072      # m, centroïde surface frontale CAO (≈ centre de poussée)
    h_g: float = 0.074      # m, CdG (centroïde CAO, à peser)

    @property
    def sigma(self):        # solidité
        return self.Nb * self.c / (np.pi * self.R)

    @property
    def A(self):
        return np.pi * self.R**2


def rotor(b, Om, V, alpha, dv=0.0):
    """T, H, Q, (M_r, M_p, Y) d'un rotor CCW. Inflow uniforme, pas de battement. dv : sillage amont (E26)."""
    VR = Om * b.R
    mu, lc = V * np.cos(alpha) / VR, (V * np.sin(alpha) + dv) / VR
    s, lam = b.sigma, 0.05
    for _ in range(200):                                  # point fixe sur λ
        CT = s * b.a / 2 * (b.th75 / 3 * (1 + 1.5 * mu**2) - lam / 2)
        lam_new = lc + CT / (2 * np.hypot(mu, lam))
        if abs(lam_new - lam) < 1e-10:
            break
        lam = 0.5 * lam + 0.5 * lam_new
    li = lam - lc
    CH = s * mu / 4 * (b.cd0 + b.a * b.th75 * lam)        # profil + portance inclinée
    CQ = b.kappa * li * CT + lc * CT + s * b.cd0 / 8 * (1 + 4.65 * mu**2)
    CMr = s * b.a * mu / 4 * (2 * b.th75 / 3 - lam / 2)  # E18 : roulis de moyeu
    q = RHO * b.A * VR**2
    return CT * q, CH * q, CQ * q * b.R, (CMr * q * b.R, 0.0, 0.0)


def omega_hover(b, rot=rotor):
    """Ω tel que 4T = mg à V = 0 (dichotomie)."""
    lo, hi = 50.0, 3000.0
    for _ in range(30):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if 4 * rot(b, mid, 0, 0)[0] < b.m * G0 else (lo, mid)
    return 0.5 * (lo + hi)


def wrench(b, Om, V, alpha=0.0, tare=True, rot=rotor, dv=0.0):
    """[Fx, Fy, Fz, Mx, My, Mz] exercé par le drone sur la cellule. Om : scalaire ou 4 valeurs.
    dv : Δv vers le bas sur les rotors arrière (sillage avant, E26)."""
    cache = {}                                              # un appel rotor par régime distinct
    hubs = b.d * np.array([[1, -1], [-1, 1], [1, 1], [-1, -1]])  # PX4 quad-X : 1 AvD, 2 ArG, 3 AvG, 4 ArD
    spin = [1, 1, -1, -1]                                   # +1 = CCW vu de dessus
    F, M = np.zeros(3), np.zeros(3)
    for (x, y), s, om in zip(hubs, spin, np.broadcast_to(Om, 4)):
        if om <= 0:                                         # rotor arrêté : ignoré
            continue
        k = (om, dv if x < 0 else 0.0)
        if k not in cache:
            cache[k] = rot(b, om, V, alpha, dv=k[1]) if k[1] else rot(b, om, V, alpha)
        T, H, Q, (Mr, Mp, Y) = (*cache[k], (0, 0, 0))[:4]   # 3 retours tolérés
        f = np.array([-H, s * Y, T])                        # H opposé à l'avance ; Y : E20
        r = np.array([x, y, b.h_r])
        F += f
        M += np.cross(r, f) + np.array([-s * Mr, Mp, -s * Q])  # E19 moyeu ; réaction couple
    va = -V * np.array([np.cos(alpha), 0, np.sin(alpha)])   # air relatif (repère drone)
    D = 0.5 * RHO * np.abs(va) * np.array(b.CdA) * va       # traînée cellule
    W = b.m * G0 * np.array([np.sin(alpha), 0, -np.cos(alpha)])
    F += D + W
    M += np.cross([0, 0, b.h_f], D) + np.cross([0, 0, b.h_g], W)
    if tare:                                                # tare à vide, alpha = 0
        F -= np.array([0, 0, -b.m * G0])
    return np.concatenate([F, M])


# Gamma SI-130-10 supposé (À CONFIRMER) : Fxy, Fz, Mxy, Mz
LIMITS = np.array([130, 130, 400, 10, 10, 10])

if __name__ == "__main__":
    from rotor_bemt import rotor_bemt
    b = Bench()
    models = {"analytique": rotor, "BEMT": rotor_bemt}
    Om = {k: omega_hover(b, r) for k, r in models.items()}
    for k in models:
        print(f"Ω_hover {k:10s} = {Om[k]*30/np.pi:5.0f} tr/min")
    for alpha in (0.0, np.radians(10)):
        print(f"\nα = {np.degrees(alpha):.0f}°  Ω fixe = Ω_hover, poids taré à α = 0   [N, N·m]")
        print(f"{'':4} {'analytique':^20} | {'BEMT':^20}")
        print(f"{'V':>4} {'Fx':>6} {'Fz':>6} {'My':>6} | {'Fx':>6} {'Fz':>6} {'My':>6}  max%")
        for V in [0, 1, 3, 5, 7, 9, 11, 13]:
            wa, wb = (wrench(b, Om[k], V, alpha, rot=r) for k, r in models.items())
            u = 100 * np.max(np.abs(wb) / LIMITS)
            print(f"{V:4.0f} {wa[0]:6.2f} {wa[2]:6.2f} {wa[4]:6.2f} | {wb[0]:6.2f} {wb[2]:6.2f} {wb[4]:6.2f}  {u:4.0f}")
    print("\nFy = Mx = Mz = 0 par symétrie. max% : Gamma SI-130-10 (capteur À CONFIRMER)")

    # Moments de moyeu (E18–E19), α = 0, BEMT. Cas : avant −5 % (rotors 1,3), CCW −5 % (1,2)
    cases = {"égal": [1, 1, 1, 1], "av−5%": [.95, 1, .95, 1], "CCW−5%": [.95, .95, 1, 1]}
    print(f"\nMoments de moyeu, α = 0°, BEMT   [N·m]\n{'V':>4} {'M_r(an)':>8} {'M_r':>7} {'Mx_1':>7}"
          + "".join(f" {'Mx '+k:>10} {'My '+k:>10}" for k in cases))
    for V in [0, 1, 3, 5, 7, 9, 11, 13]:
        Ob = Om["BEMT"]
        Mr = rotor_bemt(b, Ob, V, 0)[3][0]
        Mr_an = rotor(b, Om["analytique"], V, 0)[3][0]
        w = {k: wrench(b, Ob * np.array(c), V, rot=rotor_bemt) for k, c in cases.items()}
        w1 = wrench(b, Ob * np.array([1, 0, 0, 0]), V, rot=rotor_bemt, tare=False)  # rotor 1 seul
        print(f"{V:4.0f} {Mr_an:8.4f} {Mr:7.4f} {w1[3]:7.3f}"
              + "".join(f" {w[k][3]:10.4f} {w[k][4]:10.3f}" for k in cases))
    print("M_r : par rotor ; Mx_1 : rotor 1 seul (bras y·T inclus) ; M_p = Y = 0 (symétrie ψ ↔ π−ψ)")

    # Ingestion du sillage avant par les rotors arrière (E22–E26), α = 0, BEMT, Ω = Ω_hover
    from wake import dv_rear
    Ob = Om["BEMT"]
    vh = np.sqrt(rotor_bemt(b, Ob, 0, 0)[0] / (2 * RHO * b.A))
    print(f"\nSillage avant -> rotors arrière, α = 0°, BEMT, v_h = {vh:.1f} m/s   [m/s, N, N·m]")
    print(f"{'V':>4} {'χ°':>5} {'v0':>5} {'η':>6} {'Δv':>5} {'ΔT_ar':>6} {'My':>6} {'My_s':>6} {'ΔMy':>6}")
    for V in [1, 3, 5, 7, 9, 11, 13]:
        dv, chi, v0 = dv_rear(b, rotor_bemt(b, Ob, V, 0)[0], V)
        dT = rotor_bemt(b, Ob, V, 0, dv=dv)[0] - rotor_bemt(b, Ob, V, 0)[0]
        w0, w1 = wrench(b, Ob, V, rot=rotor_bemt), wrench(b, Ob, V, rot=rotor_bemt, dv=dv)
        print(f"{V:4.0f} {np.degrees(chi):5.1f} {v0:5.2f} {dv/v0:6.3f} {dv:5.2f} {dT:6.3f} "
              f"{w0[4]:6.3f} {w1[4]:6.3f} {w1[4]-w0[4]:6.3f}")
    print("ΔT_ar par rotor ; My_s : avec sillage. Sillage rigide valable pour V ≳ v_h (en dessous, effet ≈ 0)")
