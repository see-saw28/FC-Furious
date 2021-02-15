# FC Furious
 
## Partie Coach

## Partie intelligence artificielle

### Agent :
Dans ce cas, on traité un réseau de neurone profond, c'est à dire avec plusieurs couches. On peut le representer sous cette forme :
![Reseau](img/reseau.png)
* En entrée, on rentre un état représentatif de la position des robots sous la forme d'un vecteur de dimension 4x9x9.  
* En sortie, on obtient une prédiction pour chacune des actions qui est proportionnelle à la récompense qu'on peut espérer à long terme. On choisit donc l'action pour laquelle l'espérance est la plus grande.  

-------
### Entrainement :
Voici l'algorithme général d'apprentissage dans le cas du deep Q learning :
![Deep Q learning](img/algo.png)

Il peut être simplifié sous la forme suivante :
![Deep q simplifie](img/algo_simplifie.png)  

Tout d'abord, il a fallu coder une Classe pour représenter l'environnement dans lequel l'agent va évoluer.  
* Les actions possibles : HAUT, BAS, DROITE, GAUCHE
* Le terrain avec ses limites
* L'objectif pour finir une partie : but ouvert
* Les récompenses :   
  * -1 par déplacement effectué pour forcer l'agent à trouver le plus court chemin
  * -3 s'il sort du terrain ou s'il se prend un autre robot (la valeur étant plus élévé, l'agent devrait éviter les robots plus facilement)
  * +10 si l'objectif est accompli  
  
Ensuite, il a fallu coder une Classe pour l'agent avec :
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
* D représentatn le 2ème defenseur.

Sur cette situation, on obtient au final cette position pour l'attaquant :
![Position finale](img/final_pos.png)  
* Mouvements : 2  
* Score : 9

De même, on obtient ce résultat :  
![Partie 2](img/partie_2.png)  
* Mouvements : 5  
* Score : 6  

La génération étant aléatoire pour couvrir toutes les configurations de terrains possibles, parfois l'issue est très simple (1 mouvement suffit), d'autres fois elle l'est beaucoup moins.  

======
### Validation de l'agent obtenu :
Après plusieurs séries de tests sur 10000 générations de parties aléatoires. Le score moyen est de 8.6 avec un taux d'échec inférieur à 0.6%. Un échec est comptabilisé si l'agent n'atteint pas l'objectif en moins de 20 coups.  
Les résultats sont très satisfaisants et donc utilisables pour le coach.   
De plus le temps d'execution de 10000 parties est seulement d'une cinquantaine de seconde donc assez rapide pour être implémenté.


