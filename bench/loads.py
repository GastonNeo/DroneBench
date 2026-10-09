"""Efforts sur la cellule 6 axes du banc DroneBench (Holybro X650).

Modèle quasi-statique : BEMT à inflow uniforme (Leishman, 2006, ch. 2-3)
+ traînée de cellule ½ρV²·CdA + poids projeté. Repère capteur = repère drone :
x avant, y gauche, z haut (axe poussée), origine au centre de la cellule.
Le vent arrive de l'avant ; alpha > 0 = nez en bas (convention Leishman).
"""
import numpy as np
from dataclasses import dataclass

G0, RHO = 9.81, 1.225

@dataclass
class Bench:
    # --- Drone (À CONFIRMER : masse pesée, hélices réelles) ---
    m: float = 2.5          # kg, masse au-dessus de la cellule
    R: float = 0.1905       # m, rayon hélice 15"
    c: float = 0.022        # m, corde moyenne
    Nb: int = 2             # pales
    th75: float = np.radians(8.9)  # pas à 0.75R (pas 5.5" -> atan(p/2πr))
    a: float = 5.7          # 1/rad, pente Cl
    cd0: float = 0.015      # traînée profil
    kappa: float = 1.15     # facteur puissance induite
    CdA: tuple = (0.030, 0.030, 0.060)  # m², traînée cellule (x, y, z)
    # --- Géométrie CAO (mm -> m), réf. centre cellule ---
    d: float = 0.230        # m, bras moteur projeté sur x et y (X, empattement 650)
    h_r: float = 0.194      # m, plan rotor au-dessus de la cellule
    h_f: float = 0.132      # m, centre de poussée aéro de la cellule (plaques)
    h_g: float = 0.074      # m, CdG (centroïde CAO, à peser)

    @property
    def sigma(self):        # solidité
        return self.Nb * self.c / (np.pi * self.R)

    @property
    def A(self):
        return np.pi * self.R**2


def rotor(b, Om, V, alpha):
    """T, H, Q d'un rotor (N, N, N·m). Inflow uniforme, pas de battement."""
    VR = Om * b.R
    mu, lc = V * np.cos(alpha) / VR, V * np.sin(alpha) / VR
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
    q = RHO * b.A * VR**2
    return CT * q, CH * q, CQ * q * b.R


def omega_hover(b):
    """Ω tel que 4T = mg à V = 0 (dichotomie)."""
    lo, hi = 50.0, 3000.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if 4 * rotor(b, mid, 0, 0)[0] < b.m * G0 else (lo, mid)
    return 0.5 * (lo + hi)


def wrench(b, Om, V, alpha=0.0, tare=True):
    """[Fx, Fy, Fz, Mx, My, Mz] exercé par le drone sur la cellule."""
    T, H, Q = rotor(b, Om, V, alpha)
    ex = np.array([1.0, 0, 0])
    hubs = b.d * np.array([[1, -1], [-1, 1], [1, 1], [-1, -1]])  # PX4 quad-X : 1 AvD, 2 ArG, 3 AvG, 4 ArD
    spin = [1, 1, -1, -1]                                   # +1 = CCW vu de dessus
    F, M = np.zeros(3), np.zeros(3)
    for (x, y), s in zip(hubs, spin):
        f = np.array([-H, 0, T])                            # H opposé à l'avance
        r = np.array([x, y, b.h_r])
        F += f
        M += np.cross(r, f) + np.array([0, 0, -s * Q])      # réaction couple
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
    b = Bench()
    Om = omega_hover(b)
    print(f"Ω_hover = {Om:.0f} rad/s ({Om*30/np.pi:.0f} tr/min)  σ = {b.sigma:.3f}  μ_max = {13/(Om*b.R):.3f}")
    for alpha in (0.0, np.radians(10)):
        print(f"\nα = {np.degrees(alpha):.0f}°  (Ω fixe, poids taré à α = 0)")
        print(f"{'V':>4} {'Fx':>7} {'Fy':>6} {'Fz':>7} {'Mx':>6} {'My':>6} {'Mz':>7} {'max%':>5}")
        for V in [0, 1, 3, 5, 7, 9, 11, 13]:
            w = wrench(b, Om, V, alpha)
            u = 100 * np.max(np.abs(w) / LIMITS)
            print(f"{V:4.0f} {w[0]:7.2f} {w[1]:6.2f} {w[2]:7.2f} {w[3]:6.2f} {w[4]:6.2f} {w[5]:7.3f} {u:5.0f}")
    print("\n[N, N·m]  max% = taux d'utilisation pleine échelle")
