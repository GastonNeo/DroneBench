# Projet FlightStream → efforts au Gamma

**Objectif :** prédire avec FlightStream (FS) les 6 composantes mesurées par l'ATI Gamma SI-130-10, à V = 0 puis de 1 à 13 m/s, avec 1 ou 4 rotors actifs.
**Le cœur du projet :** passer correctement des efforts calculés sur le drone au torseur lu par le capteur.

## 1. Ce que mesure le Gamma

Le capteur coupe le banc en deux. Il mesure le torseur exercé par **tout ce qui est au-dessus de lui** (le drone et la plaque d'interface) sur le banc. Ce torseur est réduit à **son origine de mesure $O_G$** et exprimé dans **son repère**.

Équilibre statique du corps au-dessus du capteur :

$$\mathbf{F}_G = \mathbf{F}_{aero} + \mathbf{F}_{rot} + m\,\mathbf{g} - \mathbf{F}_{tare}, \qquad \mathbf{M}_G = \mathbf{M}_{aero,O_G} + \mathbf{M}_{rot,O_G} + \mathbf{r}_{G\,O_G}\times m\mathbf{g} - \mathbf{M}_{tare}$$

| Terme | Lecture |
|---|---|
| $\mathbf{F}_{aero}$ | Efforts de l'air sur les surfaces du drone et de l'interface, c'est-à-dire la sortie FS intégrée. |
| $\mathbf{F}_{rot}$ | Efforts des rotors ($T$, $H$, $Q$, moments de moyeu). Ils sont à ajouter si FS ne les inclut pas (voir §4). |
| $m\mathbf{g}$ | Poids du drone et de l'interface, projeté dans le repère capteur. Il dépend de l'attitude. |
| tare | Valeurs mesurées à attitude nulle, rotors arrêtés et V = 0. On les retranche, et ce qui reste est l'effort utile. |

## 2. La chaîne de transport (à démontrer par l'étudiant)

| # | Étape | Équation | Lecture |
|---|---|---|---|
| T1 | Repère FS = repère CAO | — | On importe le STEP sans le déplacer : toutes les coordonnées restent celles de la CAO. |
| T2 | Point de réduction FS | $P_{FS} = O_G$ (coordonnées CAO) | Si on place le point de référence des moments de FS sur l'origine ATI, aucun transport n'est nécessaire. |
| T3 | Sinon, transport | $\mathbf{M}_{O_G} = \mathbf{M}_{P} + (\mathbf{P}-\mathbf{O}_G)\times\mathbf{F}$ | Changer de point ne change pas la force. Cela ajoute un moment égal à force × bras de levier. |
| T4 | CAO → repère drone | $x = -X,\; y = -Z,\; z = -Y$ | Matrice de rotation propre ($\det = +1$). Le nez est vers $-X$ et le haut vers $-Y$. |
| T5 | Repère drone → repère ATI | $\mathbf{F}_{ATI} = \mathbf{R}_{d\to ATI}\,\mathbf{F}$ | À relever sur le capteur monté : sens de $z$ (gravé sur le boîtier) et orientation de $x$. |
| T6 | Attitude du banc | $\mathbf{g}_d = \mathbf{R}^{T}(\psi,\varphi,\theta)\,\mathbf{g}$ | Le poids tourne dans le repère capteur. À attitude non nulle, il reste un résidu après la tare (E28 dans `equations.md`). |
| T7 | Signe | FS donne l'effort **de l'air sur le corps** | C'est bien ce que le corps transmet au capteur. Il ne faut pas changer le signe. |

**Point de référence CAO** (mm) : centre géométrique de la cellule $(-337, -1206, 170)$, face drone $Y = -1223$, face banc $Y = -1189$.
L'origine ATI exacte est donnée par le plan du Gamma et par le fichier de calibration. Un décalage de 17 mm change $M_y$ d'environ 10 % à 13 m/s.

**Test de validation du transport** (à faire avant toute simulation) : appliquer une force connue en un point connu, puis comparer le calcul à la main au script. Les équations E10 et E27 du dépôt servent de référence.

## 3. Ce qu'il faut intégrer (ou non) dans FS

- **À intégrer :** les surfaces du drone et de l'interface, c'est-à-dire tout ce qui est au-dessus du capteur.
- **À modéliser sans l'intégrer :** le bras, la poutre et le montant. Ils dévient l'écoulement et reçoivent le sillage, mais le capteur ne les voit pas.
- **Attention :** un solveur à panneaux prédit mal la traînée de corps non profilés, car il ne modélise pas les décollements. Le $C_DA$ de la cellule calculé par FS est donc à comparer à la mesure, hélices à l'arrêt (`bench/cda.py`), et non à prendre tel quel.

## 4. Rotors : actuator disk

À clarifier par la littérature (voir la section Research ci-dessous) :
1. **Les entrées du disque**, à définir de façon cohérente avec la BEMT de `BEMT/` au même régime : poussée, couple, distribution radiale, swirl.
2. **Les efforts rapportés par FS** incluent-ils $T$ et $Q$ du disque ? Si ce n'est pas le cas, on les ajoute :

$$\mathbf{F}_{rot} = \sum_i T_i\,\mathbf{n}_i,\qquad \mathbf{M}_{rot} = \sum_i \left[\mathbf{r}_i\times T_i\mathbf{n}_i - s_i Q_i\,\mathbf{n}_i\right]$$

   - $\mathbf{r}_i$ : position du moyeu $i$ par rapport à $O_G$ ;
   - $\mathbf{n}_i$ : axe du rotor ;
   - $s_i = +1$ pour un rotor CCW vu de dessus. Un rotor CCW exerce sur le drone un couple de réaction CW, d'où le signe $-$.

3. **Le cas V = 0** : FS a-t-il besoin d'une vitesse de référence non nulle ?

## 5. Matrice de cas (de la plus simple à la plus complète)

| # | Cas | Ce qu'on apprend | Référence de contrôle |
|---|---|---|---|
| 1 | V = 0, 1 rotor | Poussée, couple, **déportance du sillage sur le bras du drone** (Fz mesuré < T) | BEMT : T, Q |
| 2 | V = 0, 4 rotors | Interaction entre rotors, Mz ≈ 0 | `Results/2_cellule_vs_rpm.csv` |
| 3 | V = 1–13 m/s, rotors arrêtés | $C_DA$ et $h_{cp} = M_y/F_x$ | `bench/cda.py`, puis la mesure |
| 4 | V = 1–13 m/s, 4 rotors, attitude nulle | Portance de translation, H, sillage avant → arrière | `Results/2_*.csv` (My) |
| 5 | Attitudes ψ, φ, θ | Projection du poids, dérapage | `Results/4_attitude_13ms.csv` |

## 6. Livrables de l'étudiant

1. La démonstration T1–T7 et le test de validation du transport.
2. La revue de littérature FS + actuator disk : hypothèses, pratiques, limites.
3. Les cas 1 à 5, comparés au modèle analytique et BEMT du dépôt, puis aux essais en soufflerie.
