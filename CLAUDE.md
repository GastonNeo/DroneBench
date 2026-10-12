# DroneBench

Banc d'efforts du VKI (EAU 2505) : un drone Holybro X650 sur une cellule 6 axes ATI Gamma, au bout d'un bras à 3 axes motorisés (yaw à la base, roll, pitch).
Deux cas : sans vent, et en soufflerie de 1 à 13 m/s.

- Modèle : `bench/loads.py`
- Équations décodées : `docs/equations.md`
- CAO : `Robotic_arm_assy_sep26.stp` (Y CAO = −vertical)

## Rôle de la session principale : chef d'orchestre
1. Découpe la demande, puis délègue : `innovator` propose et teste, `physics` valide, `research` alimente.
2. Communication : `team/BOARD.md` est le seul canal partagé. Tu relaies une entrée d'un agent à l'autre quand c'est nécessaire.
3. Boucle : INN PROPOSE → PHY VERIFIE/REJETE → (RES si besoin). Au plus 2 itérations par sujet, ensuite tu arbitres.
4. Lance un agent seulement s'il a une tâche précise. Lance-les en parallèle quand ils sont indépendants.

## Règles
- Économiser les tokens : réponses courtes, lecture ciblée, pas de récapitulatif redondant.
- Python de niveau ingénieur aéro : SI, commentaires succincts, prints épurés.
- L'utilisateur s'entraîne à lire les équations : pour toute nouvelle équation, donner une « Lecture » d'une ligne.
- Hypothèses non confirmées (marquées « À CONFIRMER » dans le code) : CdA, calibration du Gamma (SI-32-2.5, SI-65-5 ou SI-130-10), masses du banc.
- Équations en LaTeX (`$...$`, rendu GitHub) ; dans les tableaux, utiliser `\lvert x\rvert` et non `|x|`.
