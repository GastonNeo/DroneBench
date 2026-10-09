# Lire une équation en 5 passes

Méthode à appliquer à chaque équation (papier, code, rapport) :

1. **Inventaire** : distinguer les variables, les paramètres et les constantes. Donner l'unité SI de chaque symbole.
2. **Homogénéité** : chaque terme doit avoir la même dimension. Si ce n'est pas le cas, il y a une erreur ou un adimensionnement caché.
3. **Adimensionnels** : repérer les groupes sans unité (μ, λ, C_T, σ…). Ils disent *ce qui compte* physiquement.
4. **Limites** : regarder ce que devient l'équation quand V→0, μ→0 ou α→0. On doit retrouver un cas connu.
5. **Phrase** : traduire l'équation en une phrase physique. Si tu n'y arrives pas, tu ne l'as pas comprise.

Entraînement : pour chaque équation ci-dessous, cache la colonne « Lecture » et reformule-la toi-même.

## Équations du modèle (`bench/loads.py`)

| # | Équation | Lecture |
|---|---|---|
| E1 | σ = N_b c / (πR) | Solidité : fraction du disque occupée par les pales. |
| E2 | μ = V cos α / (ΩR) | Vitesse d'avance : vent dans le plan du disque, rapporté à la vitesse en bout de pale. |
| E3 | λ_c = V sin α / (ΩR) | Part du vent qui traverse le disque (montée ou descente). |
| E4 | C_T = (σa/2)[θ₀.₇₅/3·(1 + 3/2 μ²) − λ/2] | Élément de pale : la portance croît avec le pas et l'avance, et décroît avec l'inflow (l'inflow réduit l'incidence). |
| E5 | λ = λ_c + C_T / (2√(μ² + λ²)) | Glauert : la vitesse induite est d'autant plus faible que le débit d'air à travers le disque est grand. C'est l'origine de la *portance de translation*. |
| E6 | C_H = (σμ/4)(c_d0 + a θ λ) | Force H dans le plan : traînée de profil + portance inclinée par l'inflow, asymétrisée par l'avance. |
| E7 | C_Q = κλ_i C_T + λ_c C_T + (σc_d0/8)(1 + 4.65μ²) | Couple = puissance induite + puissance de montée + puissance de profil. |
| E8 | T = C_T ρA(ΩR)², H = C_H ρA(ΩR)², Q = C_Q ρA(ΩR)²R | Retour aux grandeurs dimensionnelles. |
| E9 | D_i = ½ρ\|v_i\| (C_dA)_i v_i | Traînée de la cellule, axe par axe, toujours opposée au mouvement relatif. |
| E10 | M = Σ r × F | Moment au centre de la cellule. Le bras de levier, c'est la hauteur h. |

Vérification des limites : avec V = 0, E5 donne λ = √(C_T/2), soit la vitesse induite en vol stationnaire v_h = √(T/2ρA).

Référence : J. G. Leishman, *Principles of Helicopter Aerodynamics*, 2ᵉ éd., CUP 2006, ch. 2–3 et 5.
