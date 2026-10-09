# Tableau d'équipe

Canal de communication commun aux agents. Chaque entrée fait 5 lignes au plus.

Format : `[AGENT] #id statut — message (réf. éq./fichier)`
Statuts : PROPOSE · VERIFIE · REJETE · SOURCE · QUESTION

[ORCH] #0 — Modèle v0 dans `bench/loads.py` (E1–E10, `docs/equations.md`). À vérifier : E6 (H-force), la masse, les hélices, et le capteur (Gamma SI-130-10 ?).
[ORCH] #1 — Données utilisateur : m = 2.5 kg confirmée ; hélices X500 V2 (1045) ; vent face au nez ; capteur inconnu ; CdA à mesurer.
CAO : S_x = 0.034, S_y = 0.034, S_z = 0.072 m² ; h_cp ≈ 72 mm. Ajout de `bench/cda.py` (E11–E17).
À vérifier (PHY) : ε Maskell = 2.5, la rotation E12 et la corde 18 mm.
[ORCH] #2 — Veine Ø3 m (C = 7.07 m²) → correction Maskell ≈ 1.3 %, négligeable. Disques rotor / C ≈ 3 %. Corde 18 mm confirmée.
L'utilisateur fournira sa BEMT : interface attendue `rotor(Om, V, alpha) -> T, H, Q`.
[ORCH] #3 — BEMT utilisateur intégrée via l'adaptateur `bench/rotor_bemt.py` (polaires XFOIL en cache ; modèle de substitution écarté, −26 % sur T).
H est tiré de Σ(dQ/r)·sinψ, moyenné sur l'azimut, avec a_disk = π/2 − α. Ω_hover : BEMT 6740 tr/min, analytique 8030 tr/min.
À 13 m/s : Fx BEMT −5.0 N contre −4.3 N en analytique ; Fz est identique à 1 % près. → PHY : vérifier H et a_disk.
[PHY] #3 VERIFIE — a_disk=π/2−α : V_a0=V·sinα (α>0 → T↓, comme en montée) et V_par=V·cosα ; U_T=Ωr+V·sinψ suppose ψ=0 côté aval (Leishman 2006, ch. 3 ; n° d’éq. non vérifié).
H=⟨Σ(dQ/r)·sinψ⟩ : dQ/r est la force tangentielle des N_b pales, avec H>0 vers l'aval. Le terme sinψ est indispensable, et la moyenne en ψ équivaut à la moyenne temporelle. H est linéaire en μ, comme C_H=σμ/4(c_d0+aθλ) (Leishman 2006 ch. 3, Johnson 1980 ch. 5 ; n° d’éq. non vérifié). H=0.30 N à 13 m/s, contre ≈0.2 N en analytique (écart dû aux c_d bas-Re et au vrillage).
Glauert à inflow uniforme, μ≤0.14 : environ 5 % sur T et 20-30 % sur H (gradient κ_x et effort radial absents) ; le battement est négligeable pour une hélice rigide. Acceptable, mais les moments de hub Mx/My (Σ dT·r·sinψ) ne sont pas modélisés, alors que la cellule 6 axes les mesure.
[INN] #4 PROPOSE — Moments de moyeu : rotor_bemt renvoie (M_r, M_p, Y) en 4e retour (E18–E20) ; loads.wrench les ajoute par rotor (ΔMx = −s·M_r, F_y = s·Y) et accepte Om[4].
Test (`python3 bench/loads.py`, α = 0°) : M_r par rotor BEMT = 0.044 / 0.085 / 0.131 N·m à 5 / 9 / 13 m/s (analytique E18 : 0.157 à 13 m/s). M_p = Y ≈ 1e-17, ce qui confirme la symétrie.
Mx total = 0 à régimes égaux et pour les rotors avant à −5 % (paire CCW/CW) ; CCW à −5 % : Mx = 0.009 N·m à 13 m/s. Rotor 1 seul : M_r ≈ 6 % du bras y·T (−2.03 N·m).
Ingestion du sillage (E21, non implémentée) : Δv_ar = η·v_i,av, η tiré de Castles & De Leeuw (NACA Rep. 1184, 1954 ; à vérifier). On obtient ΔMy ≈ −2 N·m par unité de η, soit environ −0.1 à −0.2 N·m pour η ≈ 0.05–0.1 (borne haute, sans relaxation de l'inflow arrière). Le couplage à η est une idée originale.
→ PHY : vérifier les signes de E19/E20 (côté avançant à droite pour un rotor CCW) et E18.
[PHY] #4 VERIFIE — E18–E20 ; E21 reste une proposition non vérifiée.
Signes : CCW avec ψ=0 à l'aval donne y = −s·r·sinψ, donc côté avançant à droite (y<0) et Mx = y·dT < 0, d'où ΔMx = −s·M_r ✓. On trouve ΔMy = +M_p et F_y = s·Y (traînée opposée à la vitesse de pale) ✓. M_p = Y = 0 en inflow uniforme sans battement ✓ (12 azimuts symétriques ψ↔π−ψ).
E18 : C_Mr = σaμ/4·(2θ/3 − λ/2) ✓, recalculé par intégration numérique (écart 2e-10). C'est exact avec θ75 pour un vrillage linéaire. Hypothèses : pale rigide, sans battement, λ uniforme, μ ≲ 0.3. Réf. : même démarche que Johnson 1980 ch. 5 / Leishman 2006 ch. 4 (moments 1/rev), n° d'équation non vérifié.
E21 : la réf. Castles & De Leeuw, NACA Rep. 1184 (1954, ex-TN 2912) existe ; elle tabule bien l'induction normale hors disque (plan longitudinal et axe latéral). Le contenu exact des tables n'a pas été lu.
[RES] #5 Castles & De Leeuw NACA Rep. 1184 — NON LU : ntrs.nasa.gov et repository.gatech.edu injoignables depuis la sandbox (ENOTFOUND). Aucune valeur v/v₀ à x/R=3.5 n'a été extraite : η de E21 reste non sourcé, ne pas coder 0.05–0.1 comme donnée.
Catalogue : tables et graphes de v/v₀ pour des points du plan de symétrie longitudinal et de l'axe latéral (disque uniforme, sillage cylindrique) ; la convention de χ et la normalisation sont à vérifier dans le PDF. Repli : Heyson & Katzoff, NACA TR 1319 (https://ntrs.nasa.gov/citations/19930091009) ; Jewel & Heyson, NASA Memo 4-15-59L, graphiques (https://ntrs.nasa.gov/api/citations/19980228400/downloads/19980228400.pdf). Thèse De Leeuw 1951 : https://repository.gatech.edu/entities/publication/0deae4af-ed39-46c8-ad1c-59a05de3bf44
