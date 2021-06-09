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
sélectionner le(s) équipe(s) à jouer dans master_all.py (env.L130) en modifiant le paramètre 'controlledTeams':  
* 'B' to control blue  
* 'Y' to control yellow  
* 'BY' to control both   

### Lancement match
RUN  master_all.py


## Contrôles manette
Si une manette est détectée:
* CARRE : remise en formation PSL des robots
* CROIX : relance les robots
* ROND : freeze, arrête tous les joueurs sans arrêter le programme
* R1 : but pour les jaunes
* L1 : but pour les bleus
* R2 : préparation engagement pour les jaunes
* L2 : préparation engagement pour les bleus
* OPTIONS : reset match
* flèche du bas : arrêt du programme python
* flèche droite : stratégie + offensive
* flèche de gauche : stratégie + défensive


## Lancement d'un affrontement contre le coach
RUN  player_vs_ordi.py

