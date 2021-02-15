# FC Furious
 
bonjour
## Partie intelligence artificielle
Dans ce cas, on traité un réseau de neurone profond, c'est à dire avec plusieurs couches. On peut le representer sous cette forme :
![Reseau](img/reseau.png)
* En entrée, on rentre un état représentatif de la position des robots sous la forme d'un vecteur de dimension 4x9x9.  
* En sortie, on obtient une prédiction pour chacune des actions qui est proportionnelle à la récompense qu'on peut espérer à long terme. On choisit donc l'action pour laquelle l'espérance est la plus grande.  

Voici l'algorithme général d'apprentissage dans le cas du deep Q learning :
![Deep Q learning](img/algo.png)

Il peut être simplifié sous la forme suivante :
![Deep q simplifie](img/algo_simplifie.png)

Résultats de l'entrainement (pour 10000 parties jouées) :
![Entrainement](img/training.png)

On observe effectivement que la fonction Loss function [(r+&gamma;max Q'<sub>a'</sub>(s',a';&theta;<sub>i</sub>)) - Q(s,a;&theta;<sub>i</sub>]² diminue bien sur le graphique du bas.  
De plus, le score des parties augmentent jusqu'à atteindre quasiment 8, cela correspond à atteindre l'objectif en 2 mouvements seulement. C'est très bon, l'algorithme d'entrainement est éfficace et on a obtenu un bon agent.

Exemple de génération de partie :  
![Partie](img/ini_pos.png)
* X représentant l'attaquant qui veut marquer, c'est lui qui est controlé par notre agent.
* M représentant son coéquipier.
* G représentant le gardien
* D représentatn le 2ème defenseur.

Sur cette situation, on obtient au final cette position pour l'attaquant :
![Position finale](img/final_pos.png)

De même, on obtient ce résultat :
![Partie 2](img/partie_2.png)
