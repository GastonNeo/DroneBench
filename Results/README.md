# Résultats du modèle (avant essais)

Généré par `python3 bench/results.py` (environ 75 s). Conditions : α = 0, poids taré, BEMT X500 + sillage avant (E21–E26).

| Fichier | Contenu |
|---|---|
| `1_drone_rotor_vs_rpm` | Efforts locaux par rotor : T avant, T arrière (avec sillage), H, Q, M_r |
| `2_cellule_vs_rpm` | Torseur au centre O de la cellule (point de mesure) |
| `3_base_vs_rpm` | Torseur à la base (articulation robolink), poids propre du banc exclu |

Base : $\mathbf{M}_B = \mathbf{M}_O + \mathbf{r}_{BO}\times\mathbf{F}$, avec $\mathbf{r}_{BO} = (0.337,\,-0.136,\,1.037)$ m dans le repère drone (CAO).
Les CSV contiennent les mêmes données (colonnes `V_ms`, `rpm`, …), à superposer aux mesures.
