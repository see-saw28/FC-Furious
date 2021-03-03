# FC Furious
 
## Partie Coach

### Commandes élémentaires :
* Commande en position avec une orientation par rapport à un point
* Passe (statique)
* Réception : calcul du vecteur directeur de la balle et ajustement de la position
* Orientation avec la balle : tourner autour de la balle (CIR) pour s'orienter
* Tir


### Postes :
* Shooter : on tire si le but est ouvert
* Dribble : utilisation de l'ia pour dribbler vers une postion où le but ouvert
* Demarquage : utilisation de l'ia pour se demarquer vers une postion où le but ouvert
* Passeur/receveur : réalisation d'une passe statique avec ajustement du receveur //a revoir
* Goal : être entre la balle et le but, devant la surface
* Tackle/Chaser : récupération de la balle 




### Création d'un champ pour aller à l'objectif en évitant les autres robots :
L'idée de base est de créer un champ répulsif en faisant le gradient de la somme des potentiels, avec un potentiel positif pour les robots à éviter et un potentiel négatif pour l'objectif. Le potentiel étant calculé avec une loi normale :   
P(M)=Pt.e<sup>(||M-M<sub>0</sub>||²/&sigma;²)</sup> avec M<sub>0</sub> le point d'origine du potentiel  
On peut observer les potentiels :  
![potentiel](img/potentiel.png)  
On obtient un champ de cette forme :
![grad](img/grad.png)  
On voit que le champ répulsif ne permet que d'éviter le robot mais pas de le contourner, de plus il y a des changements brusques de directions.  

On a alors créé un mix d'un champ tournant (rotation de +/-90° du champ répulsif en fonction de la position du robot répulsif) et du champ répulsif E<sub>robot</sub>=E<sub>tournant</sub>+k*E<sub>répulsif</sub> avec k=facteur de répulsion.  
De plus, on prend sigma du potentiel pour le champ tournant plus grand donc son effet apparait en premier.  
Ainsi, on obtient :  
![champ tournant](img/champ.png)  

De plus, on ne  prend pas en compte les robots qui sont derrière succeptible de créer des interférences.

![robot arriere](img/shéma_robot_arrière.png)  

### Asservissement en position
Ainsi, pour la commande du robot on utilise la direction du vecteur (qui est normé) à la position du robot.  Donc la commande en vitesse est toujours à saturation (env. 0.9 m/s) sauf à partir d´une certaine distance où la distance devient proportionnelle à la distance restante.  
<img src="img/Vitesse.png" align="center" width="60%">


Si on est proche de l'objectif et qu'il y a un robot proche aussi, cela fait des interférences. Dans ce cas, on ne sert plus des champs et on réalise un asservissement classique (proportionnel + saturation) à vitesse réduite.  


### Méthode de visualisation :
Visualisation en direct des positions ainsi que les états des robots, plusieurs solutions :    
* mode 0: affichage dans la console des changements de postes  
* mode 1: affichage sur un graphique des postes et du status des robots
* mode 2: affichage sur un graphique d'une reproduction du terrain 

### Stratégie:
Pas encore totalement définie.  
Pour le moment, le coach réalise 2 actions:  
* Lecture des données du terrain et attribution des postes
* Ordre aux robots en fonction du poste  

L´attribution se fait grâce à un arbre qui teste un certain nombre de conditions. On remarque que cette méthode présente ses limites. On pourra peut être utiliser un diagramme d´états pour avoir un meilleur contrôle des transitions de poste.  

Une autre amélioration sera de prendre en compte l'autoreferee et donc de pouvoir réaliser de vrais matchs. De plus, on pourra adapter la stratégie en focntion du score et du temps restant.  

--------------
## Partie intelligence artificielle

### Objectif :
Le but de cette intelligence artificielle est de déplacer un attaquant vers un endroit où le but est ouvert en un minimum de déplacements.

------------
### Agent :
Dans ce cas, on traité un réseau de neurone profond, c'est à dire avec plusieurs couches. On peut le representer sous cette forme :
![Reseau](img/reseau.png)
* En entrée, on rentre un état représentatif de la position des robots sous la forme d'un vecteur de dimension 4x9x9.  
* En sortie, on obtient une prédiction pour chacune des actions qui est proportionnelle à la récompense qu'on peut espérer à long terme. On choisit donc l'action pour laquelle l'espérance est la plus grande.  

---
### Entrainement :  
#### Théorie :  
Voici l'algorithme général d'apprentissage dans le cas du deep Q learning :

![Deep Q learning](img/algo_600x480.png) 


Il peut être simplifié sous la forme suivante :  

![Deep q simplifie](img/algo_simplifie1.png)  

On calcule une fonction d'erreur *[(r+&gamma;max Q'<sub>a'</sub>(s',a';&theta;<sub>i</sub>)) - Q(s,a;&theta;<sub>i</sub>]²* à partir d'un batch de transitions enregistrées grâce aux 2 réseaux de neurones. L'objectif étant de diminuer cette erreur, on utilise une descente de gradient. Comme ça,on peut espérer converger vers une solution.  

#### Mise en pratique :  
Tout d'abord, il a fallu coder une Classe pour représenter l'environnement dans lequel l'agent va évoluer.  
* Les actions possibles : HAUT, BAS, DROITE, GAUCHE
* Le terrain avec ses limites
* L'objectif pour finir une partie : but ouvert
* Les récompenses :   
  * -1 par déplacement effectué pour forcer l'agent à trouver le plus court chemin
  * -3 s'il sort du terrain ou s'il se prend un autre robot (la valeur étant plus élévé, l'agent devrait éviter les robots plus facilement)
  * +10 si l'objectif est accompli  
  
Ensuite, il a fallu coder une Classe pour l'agent grâce à la bibliothèque TensorFlow avec :
* Les deux réseaux de neurone Q et Q' (noté q_network et target_network dans le code)
* Une fonction qui choisit la meilleure action
* Les fonctions nécéssaires pour entrainer l'agent

Après de nombreux tests, modifications du programme d'entrainement etc.. Voici les résultats du meilleur entrainement (pour 10000 parties jouées) :
![Entrainement](img/training.png)

On observe effectivement que la fonction Loss function *[(r+&gamma;max Q'<sub>a'</sub>(s',a';&theta;<sub>i</sub>)) - Q(s,a;&theta;<sub>i</sub>]²* diminue bien sur le graphique du bas.  
De plus, le score des parties augmentent jusqu'à atteindre quasiment 8, cela correspond à atteindre l'objectif en 3 mouvements seulement. C'est très bon, l'algorithme d'entrainement est éfficace et on a obtenu un bon agent.

Exemple de génération de partie :  
![Partie](img/ini_pos.png)
* X représentant l'attaquant qui veut marquer, c'est lui qui est controlé par notre agent.
* M représentant son coéquipier.
* G représentant le gardien
* D représentant le 2ème defenseur.

Sur cette situation, on obtient au final cette position pour l'attaquant :
![Position finale](img/final_pos.png)  
* Mouvements : 2  
* Score : 9

De même, on obtient ce résultat :  
![Partie 2](img/partie_2.png)  
* Mouvements : 5  
* Score : 6  

La génération étant aléatoire pour couvrir toutes les configurations de terrains possibles, parfois l'issue est très simple (1 mouvement suffit), d'autres fois elle l'est beaucoup moins.  
Animation :  
![gif](img/move3.gif)

---
### Validation de l'agent obtenu :
Après plusieurs séries de tests sur 10000 générations de parties aléatoires. Le score moyen est de 8.6 avec un taux d'échec inférieur à 0.6%. Un échec est comptabilisé si l'agent n'atteint pas l'objectif en moins de 20 coups.  
Les résultats sont très satisfaisants et donc utilisables pour le coach.   
De plus le temps d'execution de 10000 parties est seulement d'une trentaine de seconde donc assez rapide pour être implémenté. (Sur ma machine)

---
### Implémentation dans le code du coach :
Cette ia intervient lorsque un des attaquants n'a pas le but libre devant lui. Dans ce cas, on fait appel à cette ia qui calcule la position finale et le score pour s'ouvrir le chemin du but. Ainsi, on peut commander le robot vers cette position.  
La limite de cette ia c'est qu'elle ne prend pas en compte les déplacements des joueurs et donc la solution ne menera pas toujours à un but.    
On verra par la suite comment l'utiliser :
* Prédiction de la position finale
* Prédiction des déplacements un par un pour ainsi actualiser l'état du terrain entre deux déplacements.
