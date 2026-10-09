"""Détermination de CdA du drone en soufflerie (hélices à l'arrêt).

Principe : D = q·CdA, avec q = ½ρV² lu directement au Pitot (Δp).
Régression aux moindres carrés de D sur q, puis correction de blocage (Maskell).
"""
import numpy as np


def drag_lift(Fx, Fz, alpha):
    """Repère capteur -> repère vent (α > 0 nez en bas). Renvoie D, L."""
    D = -(Fx * np.cos(alpha) + Fz * np.sin(alpha))
    L = -Fx * np.sin(alpha) + Fz * np.cos(alpha)
    return D, L


def fit_cda(q, D, uF):
    """D = CdA·q + D0. Renvoie CdA, D0 (dérive du zéro), u(CdA) à 1σ."""
    X = np.column_stack([q, np.ones_like(q)])
    (cda, d0), *_ = np.linalg.lstsq(X, D, rcond=None)
    cov = uF**2 * np.linalg.inv(X.T @ X)        # bruit capteur uniquement
    return cda, d0, np.sqrt(cov[0, 0])


def maskell(cda_u, C, eps=2.5):
    """Correction de blocage, corps non profilé (Maskell 1963). C = section veine [m²]."""
    return cda_u / (1 + eps * cda_u / C)


def cp_height(Fx, My):
    """Hauteur du centre de poussée au-dessus de la cellule : h = My / Fx."""
    return My / Fx


if __name__ == "__main__":
    # Démonstration sur un essai synthétique (remplacer par les mesures)
    rng = np.random.default_rng(0)
    CdA_true, h_true, uF = 0.037, 0.072, 0.03         # m², m, N
    C = np.pi * 1.5**2                                  # m², veine Ø3 m
    V = np.array([5, 7, 9, 11, 13.0])
    q = 0.5 * 1.225 * V**2
    Fx = -q * CdA_true * (1 + CdA_true * 2.5 / C) + rng.normal(0, uF, V.size)
    My = h_true * Fx + rng.normal(0, 0.003, V.size)

    D, _ = drag_lift(Fx, 0 * Fx, 0.0)
    cda, d0, u = fit_cda(q, D, uF)
    print(f"CdA brut     = {cda:.4f} ± {u:.4f} m²  (D0 = {d0:+.3f} N)")
    print(f"CdA corrigé  = {maskell(cda, C):.4f} m²  (veine C = {C:.2f} m²)")
    print(f"h_cp         = {np.mean(cp_height(Fx, My))*1e3:.0f} mm")
    print("\n V    q[Pa]  D[N]  u(D)/D")
    for v, qi, di in zip(V, q, D):
        print(f"{v:3.0f} {qi:7.1f} {di:5.2f}  {uF/di:5.1%}")
