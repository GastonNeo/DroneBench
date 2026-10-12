# Résultats du modèle (avant essais)

Généré par `python3 bench/results.py` (environ 75 s). Conditions : α = 0, poids taré, BEMT X500 + sillage avant (E21–E26).

| Fichier | Contenu |
|---|---|
| `1_drone_rotor_vs_rpm` | Efforts locaux par rotor : T avant, T arrière (avec sillage), H, Q, M_r |
| `2_cellule_vs_rpm` | Torseur au centre O de la cellule (point de mesure) |
| `3_articulations_vs_rpm` | Couple à tenir par les moteurs yaw, roll et pitch (E27), poids du drone inclus, poids du banc exclu |

Axes (CAO) : yaw = robolink, vertical, en X = 0 ; roll = arbre Ø20, longitudinal, h = 605 mm ; pitch = arbre Ø20, latéral, en X = 130 mm et h = 864 mm.
Les CSV contiennent les mêmes données (colonnes `V_ms`, `rpm`, …), à superposer aux mesures.
