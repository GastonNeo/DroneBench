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
