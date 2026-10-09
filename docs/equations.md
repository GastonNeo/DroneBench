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

## Déterminer CdA (`bench/cda.py`)

**Pourquoi CdA et pas Cd seul ?** Pour un drone, la surface de référence S est arbitraire (frontale ? disque ?). Le produit Cd·S, en m², est la seule grandeur mesurée sans ambiguïté. C'est la « surface équivalente de plaque » qui, multipliée par q, donne la traînée.

| # | Équation | Lecture |
|---|---|---|
| E11 | D = q·CdA, avec q = ½ρV² = Δp_Pitot | La traînée est proportionnelle à la pression dynamique. Avec q lu au Pitot, ρ et V n'interviennent plus. |
| E12 | D = −(F_x cos α + F_z sin α), L = −F_x sin α + F_z cos α | Rotation du repère capteur (lié au drone) vers le repère vent. À α = 0, D = −F_x. |
| E13 | D_i = CdA·q_i + D₀ (moindres carrés) | La pente donne CdA. L'ordonnée D₀ ≠ 0 révèle une dérive du zéro. |
| E14 | u(CdA)/CdA = √[(u_F/D)² + (u_q/q)²] | L'incertitude relative explose quand D est petit, d'où l'intérêt des hautes vitesses. |
| E15 | CdA_c = CdA_u / (1 + ε·CdA_u/C), avec ε ≈ 2.5 | Maskell : en veine fermée, le sillage accélère l'écoulement, donc la traînée mesurée est trop forte. C est la section de la veine. |
| E16 | h_cp = M_y / F_x | Le capteur donne à la fois la force et le moment, donc la hauteur où s'applique la traînée. |
| E17 | Re = V·d/ν | Avec un tube de bras d ≈ 20 mm, Re va de 1.3·10³ à 1.7·10⁴. Le régime est sous-critique, donc Cd est à peu près constant : CdA ne doit pas varier avec V. |

**Estimation a priori (CAO).** Surfaces projetées du drone avec l'interface, sans hélices :
- S_x = 0.034 m² (face au vent) ;
- S_y = 0.034 m² (de côté) ;
- S_z = 0.072 m² (vue de dessus).

Avec Cd ≈ 1.1 (assemblage de cylindres et de plaques), on obtient CdA_x ≈ 0.037 m². Le centroïde frontal est à 72 mm au-dessus de la cellule.

**Protocole de mesure, hélices à l'arrêt :**
1. Faire le zéro du capteur à V = 0, puis vérifier le zéro après chaque balayage (dérive).
2. Faire des paliers à V = 5, 7, 9, 11 et 13 m/s, chacun d'au moins 30 s, et moyenner. En dessous de 5 m/s, D < 0.6 N : le signal se noie dans la résolution du capteur.
3. Pour chaque palier, relever q au Pitot, ainsi que F_x, F_z et M_y.
4. Calculer CdA par la pente (E13), corriger le blocage (E15) et tracer D/q en fonction de V. Si D/q varie, il y a un effet de Reynolds ou une vibration.
5. Répéter pour α de −15° à +15° avec le bras pour obtenir CdA(α) et L(α).
6. Hélices en rotation : écrire F_mesuré − F_rotor(modèle) − q·CdA. Ce reste est l'**interaction** rotor/cellule, la grandeur la plus intéressante pour l'étude.

Note : la mesure couvre le drone et l'interface (tout ce qui est au-dessus de la cellule). Le bras et le support ne sont pas mesurés, mais ils perturbent l'écoulement : il faut le noter dans le rapport.

## Moments de moyeu (`bench/rotor_bemt.py`, `bench/loads.py`)

ψ = 0 côté aval, pale avançante à ψ = 90°, s = +1 pour CCW et −1 pour CW (vu de dessus). Le côté avançant est à droite pour un rotor CCW et à gauche pour un rotor CW.

| # | Équation | Lecture |
|---|---|---|
| E18 | M_r = ⟨Σ dT·r·sinψ⟩ ; analytique : C_Mr = (σaμ/4)(2θ/3 − λ/2) | Roulis de moyeu : la pale avançante voit Ωr + V·sinψ, donc porte plus. Le moment croît linéairement avec μ. |
| E19 | ΔM_x = −s·M_r, ΔM_y = M_p = ⟨Σ dT·r·cosψ⟩ | Le sens de rotation fixe le côté qui se soulève. M_p = 0 en inflow uniforme, car dT(ψ) ne dépend que de sinψ (symétrie ψ ↔ π−ψ). |
| E20 | Y = ⟨Σ (dQ/r)·cosψ⟩, F_y = s·Y | Force latérale : traînée des pales projetée sur y. Elle est nulle pour la même raison que M_p ; il faut un battement ou un inflow non uniforme. |
| E21 | Δv_ar = η(χ, 2d/R)·v_i,av ; ΔM_y ≈ 2d·ΔT_ar, avec ΔT_ar ≈ −(σa/4)ρA(ΩR)·Δv_ar (proposition) | Les rotors arrière baignent dans la déflexion vers le bas due aux rotors avant : ils portent moins, et le drone cabre (M_y < 0). η est l'induction normale hors disque d'un rotor à sillage oblique, à l'angle χ = atan(V/v_i). |
