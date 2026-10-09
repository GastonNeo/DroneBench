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
| E1 | $\sigma = \dfrac{N_b\,c}{\pi R}$ | Solidité : fraction du disque occupée par les pales. |
| E2 | $\mu = \dfrac{V\cos\alpha}{\Omega R}$ | Vitesse d'avance : vent dans le plan du disque, rapporté à la vitesse en bout de pale. |
| E3 | $\lambda_c = \dfrac{V\sin\alpha}{\Omega R}$ | Part du vent qui traverse le disque (montée ou descente). |
| E4 | $C_T = \dfrac{\sigma a}{2}\left[\dfrac{\theta_{0.75}}{3}\left(1+\tfrac{3}{2}\mu^2\right)-\dfrac{\lambda}{2}\right]$ | Élément de pale : la portance croît avec le pas et l'avance, et décroît avec l'inflow (l'inflow réduit l'incidence). |
| E5 | $\lambda = \lambda_c + \dfrac{C_T}{2\sqrt{\mu^2+\lambda^2}}$ | Glauert : la vitesse induite est d'autant plus faible que le débit d'air à travers le disque est grand. C'est l'origine de la *portance de translation*. |
| E6 | $C_H = \dfrac{\sigma\mu}{4}\left(c_{d0} + a\,\theta\,\lambda\right)$ | Force H dans le plan : traînée de profil + portance inclinée par l'inflow, asymétrisée par l'avance. |
| E7 | $C_Q = \kappa\lambda_i C_T + \lambda_c C_T + \dfrac{\sigma c_{d0}}{8}\left(1+4.65\mu^2\right)$ | Couple = puissance induite + puissance de montée + puissance de profil. |
| E8 | $T = C_T\,\rho A(\Omega R)^2,\quad H = C_H\,\rho A(\Omega R)^2,\quad Q = C_Q\,\rho A(\Omega R)^2 R$ | Retour aux grandeurs dimensionnelles. |
| E9 | $D_i = \tfrac{1}{2}\rho\,\lvert v_i\rvert\,(C_dA)_i\,v_i$ | Traînée de la cellule, axe par axe, toujours opposée au mouvement relatif. |
| E10 | $\mathbf{M}_O = \sum \mathbf{r}\times\mathbf{F}$ | Moment au centre de la cellule. Le bras de levier, c'est la hauteur h. |

Vérification des limites : avec $V = 0$, E5 donne $\lambda = \sqrt{C_T/2}$, soit la vitesse induite en vol stationnaire $v_h = \sqrt{T/(2\rho A)}$.

Référence : J. G. Leishman, *Principles of Helicopter Aerodynamics*, 2ᵉ éd., CUP 2006, ch. 2–3 et 5.

## Déterminer CdA (`bench/cda.py`)

**Pourquoi CdA et pas Cd seul ?** Pour un drone, la surface de référence S est arbitraire (frontale ? disque ?). Le produit Cd·S, en m², est la seule grandeur mesurée sans ambiguïté. C'est la « surface équivalente de plaque » qui, multipliée par q, donne la traînée.

| # | Équation | Lecture |
|---|---|---|
| E11 | $D = q\,C_DA,\quad q = \tfrac{1}{2}\rho V^2 = \Delta p_{\text{Pitot}}$ | La traînée est proportionnelle à la pression dynamique. Avec q lu au Pitot, ρ et V n'interviennent plus. |
| E12 | $D = -(F_x\cos\alpha + F_z\sin\alpha),\quad L = -F_x\sin\alpha + F_z\cos\alpha$ | Rotation du repère capteur (lié au drone) vers le repère vent. À α = 0, D = −F_x. |
| E13 | $D_i = C_DA\,q_i + D_0$ (moindres carrés) | La pente donne CdA. L'ordonnée D₀ ≠ 0 révèle une dérive du zéro. |
| E14 | $\dfrac{u(C_DA)}{C_DA} = \sqrt{\left(\dfrac{u_F}{D}\right)^2 + \left(\dfrac{u_q}{q}\right)^2}$ | L'incertitude relative explose quand D est petit, d'où l'intérêt des hautes vitesses. |
| E15 | $(C_DA)_c = \dfrac{(C_DA)_u}{1+\varepsilon\,(C_DA)_u/C},\quad \varepsilon \approx 2.5$ | Maskell : en veine fermée, le sillage accélère l'écoulement, donc la traînée mesurée est trop forte. C est la section de la veine. |
| E16 | $h_{cp} = \dfrac{M_y}{F_x}$ | Le capteur donne à la fois la force et le moment, donc la hauteur où s'applique la traînée. |
| E17 | $Re = \dfrac{V d}{\nu}$ | Avec un tube de bras d ≈ 20 mm, Re va de 1.3·10³ à 1.7·10⁴. Le régime est sous-critique, donc Cd est à peu près constant : CdA ne doit pas varier avec V. |

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
| E18 | $M_r = \left\langle \sum dT\,r\sin\psi \right\rangle$ ; analytique : $C_{M_r} = \dfrac{\sigma a\mu}{4}\left(\dfrac{2\theta}{3}-\dfrac{\lambda}{2}\right)$ | Roulis de moyeu : la pale avançante voit Ωr + V·sinψ, donc porte plus. Le moment croît linéairement avec μ. |
| E19 | $\Delta M_x = -s\,M_r,\quad \Delta M_y = M_p = \left\langle \sum dT\,r\cos\psi \right\rangle$ | Le sens de rotation fixe le côté qui se soulève. M_p = 0 en inflow uniforme, car dT(ψ) ne dépend que de sinψ (symétrie ψ ↔ π−ψ). |
| E20 | $Y = \left\langle \sum \dfrac{dQ}{r}\cos\psi \right\rangle,\quad F_y = s\,Y$ | Force latérale : traînée des pales projetée sur y. Elle est nulle pour la même raison que M_p ; il faut un battement ou un inflow non uniforme. |
| E21 | $\Delta v_{ar} = \eta\left(\chi, \tfrac{2d}{R}\right) v_{i,av}$ ; $\Delta M_y \approx 2d\,\Delta T_{ar}$, avec $\Delta T_{ar} \approx -\dfrac{\sigma a}{4}\rho A(\Omega R)\,\Delta v_{ar}$ (proposition) | Les rotors arrière baignent dans la déflexion vers le bas due aux rotors avant : ils portent moins, et le drone cabre (M_y < 0). η est l'induction normale hors disque d'un rotor à sillage oblique, à l'angle χ = atan(V/v_i). |
