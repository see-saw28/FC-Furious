# Tuto
## Prérequis :
### Modules:  
* matplotlib
* numpy
* pygame
* tensorflow
* pyserial
* time
* random

## Lancement d'une partie
### Sélection équipe
commenter/decommenter les équipes à jouer dans master_all.py (env.L150)

### Lancement match
RUN  master_all.py


## Contrôles manette
Si une manette est détectée:
* CARRE : stopper tous les robots
* CROIX : relance les robots
* R1 : but pour les jaunes
* L1 : but pour les bleus
* R2 : engagement pour les jaunes
* L2 : engagement pour les bleus
* OPTIONS : reset match
* flèche du bas : arrêt du programme python


## IA  
ia_v3 + model-ia3: dribble tout terrain
ia_v4 + model-ia4: dribble x>-400
ia_v5 + model-ia5: dribble moitié adverse
