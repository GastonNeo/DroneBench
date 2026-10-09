"""Sillage oblique rigide d'un rotor à charge uniforme (Castles & De Leeuw, NACA Rep. 1184).

Cylindre oblique d'angle χ (par rapport à la normale au disque), formé d'anneaux
tourbillonnaires circulaires parallèles au disque, d'intensité uniforme (E22–E23).
Repère du disque, unités R : X > 0 aval (plan du rotor), Y latéral, Z haut ; v > 0 vers le bas.
Biot-Savart vectorisé ; v/v₀ normalisé à 1 au centre du disque.
"""
import numpy as np
from functools import lru_cache

RHO = 1.225


def _rings(chi, n_s=400, n_th=64, s_max=100.0):
    """Segments (A, B) et poids Δs des anneaux le long de l'axe oblique s ∈ [0, s_max]."""
    sn = np.concatenate([[0.0], np.geomspace(1e-3, s_max, n_s)])
    s, ds = 0.5 * (sn[1:] + sn[:-1]), np.diff(sn)          # milieux, poids
    th = np.linspace(0, 2 * np.pi, n_th + 1)
    c = np.stack([s * np.sin(chi), 0 * s, -s * np.cos(chi)], -1)          # centres
    ring = np.stack([np.cos(th), np.sin(th), 0 * th], -1)                  # cercle unité ⊥ Z
    P = c[:, None, :] + ring[None]                          # (n_s, n_th+1, 3)
    w = np.repeat(ds, n_th)
    return P[:, :-1].reshape(-1, 3), P[:, 1:].reshape(-1, 3), w


def _vz(pts, A, B, w, chunk=32):
    """Composante vers le bas induite en pts (N,3) par les segments AB d'intensité w (facteur 1/4π omis)."""
    d, out = B - A, np.empty(len(pts))
    for i in range(0, len(pts), chunk):
        r1, r2 = pts[i:i + chunk, None, :] - A, pts[i:i + chunk, None, :] - B
        n1, n2 = np.sqrt((r1 * r1).sum(-1)), np.sqrt((r2 * r2).sum(-1))
        x2 = (n1 * n2) ** 2 - ((r1 * r2).sum(-1)) ** 2          # |r1 × r2|²
        xz = r1[..., 0] * r2[..., 1] - r1[..., 1] * r2[..., 0]
        k = ((d * r1).sum(-1) / n1 - (d * r2).sum(-1) / n2) / np.maximum(x2, 1e-12)
        out[i:i + chunk] = -(w * k * xz).sum(-1)
    return out


def v_ratio(chi, X, Y=0.0):
    """v/v₀ au point (X, Y, 0) du plan rotor (unités R) pour le sillage d'angle chi [rad]."""
    X, Y = np.broadcast_arrays(np.atleast_1d(X).astype(float), Y)
    A, B, w = _rings(chi)
    v = _vz(np.stack([X, Y, 0 * X], -1).reshape(-1, 3), A, B, w)
    v0 = _vz(np.zeros((1, 3)), A, B, w)[0]
    return (v / v0).reshape(X.shape)


@lru_cache(maxsize=None)
def _disk(n_r=8, n_az=16):
    """Points et poids d'aire d'un disque unité (milieux en r², azimut uniforme)."""
    r = np.sqrt((np.arange(n_r) + 0.5) / n_r)               # anneaux d'aire égale
    psi = np.linspace(0, 2 * np.pi, n_az, endpoint=False)
    return (r[:, None] * np.cos(psi)).ravel(), (r[:, None] * np.sin(psi)).ravel()


@lru_cache(maxsize=None)
def eta(chi, Xc, Yc=0.0):
    """E24 : moyenne surfacique de v/v₀ sur le disque unité centré en (Xc, Yc)."""
    x, y = _disk()
    return float(v_ratio(chi, Xc + x, Yc + y).mean())


def front_wake(T, V, alpha, A):
    """E25 : v₀ (Glauert, T du rotor avant) et χ. Vitesses en m/s, α > 0 nez en bas."""
    mu, lc = V * np.cos(alpha), V * np.sin(alpha)
    v0 = np.sqrt(T / (2 * RHO * A))                         # départ : v_h
    for _ in range(100):
        v0 = 0.5 * v0 + 0.5 * T / (2 * RHO * A * np.hypot(mu, lc + v0))
    return v0, np.arctan2(mu, lc + v0)


def dv_rear(b, T_front, V, alpha=0.0):
    """E26 : Δv vers le bas sur un rotor arrière, Δv = η(χ)·v₀ ; rotor arrière à X = 2d/R, Y = 0."""
    v0, chi = front_wake(T_front, V, alpha, b.A)
    return eta(round(float(chi), 4), round(2 * b.d / b.R, 3)) * v0, chi, v0


if __name__ == "__main__":
    # Jewel & Heyson (1959) fig. 19(a) : num. = numérisation 300 dpi (bruit ±0.07, témoin χ = 0) ; #6 = lecture BOARD
    chis = [14, 27, 45, 63, 76]
    num = {2: [0.05, 0.12, 0.24, 0.71, 1.25], 2.95: [np.nan, 0.08, 0.12, 0.41, 0.96]}
    b6 = {2: [0.14, 0.24, 0.47, 0.95, 1.3], 2.95: [0.10, 0.15, 0.24, 0.44, 0.95]}  # #6 lu à X = 3
    print("v/v₀ sur l'axe X, plan rotor : modèle / num. / #6")
    print(f"{'χ°':>4} {'X=0':>5} {'X=2':>17} {'X=2.95':>17}")
    for i, ch in enumerate(chis):
        v = v_ratio(np.radians(ch), [0, 2, 2.95])
        print(f"{ch:4d} {v[0]:5.2f} " + " ".join(f"{v[j+1]:5.2f}/{num[X][i]:4.2f}/{b6[X][i]:4.2f}"
                                               for j, X in enumerate(num)))
    print(f"χ = 0° : X=2 -> {v_ratio(0.0, 2.0)[0]:.0e} (cylindre droit : 0 exact)")
    Xc = 0.46 / 0.13
    print(f"\nη(χ), disque arrière centré X/R = {Xc:.2f}, Y = 0 ; croisé : rotor avant opposé (Y/R = {Xc:.2f})")
    for ch in (15, 30, 45, 60, 70, 75, 80, 85):
        print(f"χ = {ch:2d}°  η = {eta(np.radians(ch), Xc):5.3f}   "
              f"η_croisé = {eta(np.radians(ch), Xc, Xc):6.3f}")
