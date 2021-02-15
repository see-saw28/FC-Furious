# -*- coding: utf-8 -*-
"""
Created on Sun Feb 14 14:04:39 2021

@author: paulg
"""
# -*- coding: utf-8 -*-
"""
Created on Fri Feb 12 23:43:18 2021

@author: paulg
"""
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Feb  7 16:03:46 2021

@author: psl
"""
import random
import numpy as np
import tensorflow as tf #module ia
import time
import os
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as lines
from collections import deque #module pour créer une pile
import copy

# flatten = lambda l: [item for sublist in l for item in sublist]
longueur=1350
largeur=1000

#class qui permet de modéliser l'environnement de l'agent
class Game:

    ACTION_UP = 0
    ACTION_LEFT = 1
    ACTION_DOWN = 2
    ACTION_RIGHT = 3
    

    ACTIONS = [ACTION_DOWN, ACTION_LEFT, ACTION_RIGHT, ACTION_UP]

    ACTION_NAMES = ["UP   ", "LEFT ", "DOWN ", "RIGHT"]

    MOVEMENTS = {
        ACTION_UP: (0, 1),
        ACTION_RIGHT: (1, 0),
        ACTION_LEFT: (-1, 0),
        ACTION_DOWN: (0, -1)
    }

    num_actions = len(ACTIONS)

    def __init__(self, n=9, m=9, alea=False,terrain=None):
        #dimmension de la grille
        self.n = n 
        self.m = m
        
        #paramètre si on veut s'entrainer sur un terrain aléatoire ou non
        self.alea = alea
        
        #Le terrain et donc l'emplacement des différents robots peut être généré aléatoirement (utile pour l'entrainement)
        if terrain==None: 
            self.generate_game()
        
        #ou il est possible de donner un terrain (obligatoire dans le cas pratique de la robocup)
        else:
            self.position = terrain[0]
            xb,yb=self.position_to_xy(terrain[0][0],terrain[0][1])
            self.positionxy=(xb,yb)
            
            self.goal = terrain[1]
            self.goalxy=(terrain[2][0],terrain[2][1])
            
            self.defenseur = terrain[3]
            xdef,ydef=self.position_to_xy(terrain[3][0], terrain[3][1])
            self.defenseurxy=(xdef,ydef)
            
            self.mate = terrain[4]
            xm,ym=self.position_to_xy(terrain[4][0], terrain[4][1])
            self.matexy=(xm,ym)
           
            self.start = terrain[0]
            
            self.counter = 0
            
            #création de l'ensemble des cases du terrrain (grille de 9x9)
            cases = [(x, y) for x in range(self.n) for y in range(self.m)]
            
            #on enlève toutes les cases correspondant aux surfaces
            cases.remove((0,3))
            cases.remove((0,4))
            cases.remove((0,5))
            cases.remove((8,3))
            cases.remove((8,4))
            cases.remove((8,5))
            
            #puis on enlève les cases correspondant aux autres robots
            for case in (self.goal,self.defenseur,self.mate,self.start):
                if case in cases:
                    cases.remove(case)
            self.cases=cases
            
            
    
    #fonction pour obtenir les cordonnées x,y d'un robot à partir de ses coordonées discrètes sur la grille
    def position_to_xy(self,xd,yd):
        return(-longueur+(xd+0.5)*(2*longueur)/self.m,-largeur+(yd+0.5)*(2*largeur)/self.n)
    
    #fonction inverse de la précédente
    def xy_to_position(self,x,y):
        return(int(int(x+longueur)//(2*longueur/self.m)),int(int(y+largeur)//(2*largeur/self.n)))
    
    #fonction qui génère aléatoirement une disposition des robots sur le terrain
    def generate_game(self):
        #création de l'ensemble des cases pas interdites
        cases = [(x, y) for x in range(self.n) for y in range(self.m)]
        
        cases.remove((0,3))
        cases.remove((0,4))
        cases.remove((0,5))
        cases.remove((8,3))
        cases.remove((8,4))
        cases.remove((8,5))
        
        #On enlève les cases au bord de la surface adverse, si le robot avec la balle s'y trouve 
        #c'est que le but est ouvert car le gardien ne peut pas se placer entre l'attaquant et la surface
        cases_prime=copy.deepcopy(cases)
        cases_prime.remove((8,2))
        cases_prime.remove((7,2))
        cases_prime.remove((7,3))
        cases_prime.remove((7,4))
        cases_prime.remove((7,5))
        cases_prime.remove((7,6))
        cases_prime.remove((8,6))
        
        #on choisit aléatoirement la position de l'attaquant parmis les cases possibles pour lui
        baller = random.choice(cases_prime)
        
        #on enlève la case pour que les autres robots ne puissent y aller
        cases.remove(baller)
        
        #DETERMINATION DE LA POSITION DU GARDIEN POUR EMPECHER LE TIR
        xbut=1350
        ybut=0
        
        xb,yb=self.position_to_xy(baller[0], baller[1])
        
        #coefficients de la droite entre le but et l'attaquant
        a,b=np.polyfit([xb,xbut],[yb,ybut],1)
        
        if baller[0]<7:
            #on détermine la position x du goal aléatoirement entre l'attaquant et la surface
            xdg=random.randrange(baller[0]+1,self.n-1)
            xg,yg=self.position_to_xy(xdg,0)
            
            #on détermine la position y du goal grace aux coefficients de la droite, ainsi le gardien empeche l'attaquant de marquer
            yg=a*xg+b
            xdg,ydg=self.xy_to_position(xg, yg)
        
        
        else :#on regarde si l'attaquant est proche des corners, situation un peu particulière
            if baller[1]<3:
                ydg=2
            elif baller[1]>5:
                ydg=6
            xg,yg=self.position_to_xy(0,ydg)
            xg=(yg-b)/a
            xdg,ydg=self.xy_to_position(xg, yg)
        
        
        goal=(xdg,ydg)
        cases.remove(goal)
        
        #DETERMINATION DES POSITIONS DES DEUX DERNIERS ROBOTS
        defenseur = random.choice(cases)
        cases.remove(defenseur)
        xdef,ydef=self.position_to_xy(defenseur[0], defenseur[1])
       
        mate = random.choice(cases)
        cases.remove(mate)
        xm,ym=self.position_to_xy(mate[0], mate[1])
        
        self.cases=cases
        self.position = baller
        self.positionxy=(xb,yb)
        self.goal = goal
        self.goalxy=(xg,yg)
        self.defenseur = defenseur
        self.defenseurxy=(xdef,ydef)
        self.mate = mate
        self.matexy=(xm,ym)
        self.counter = 0
        
        #si la partie n'est pas aléatoire on conserve la position de départ pour recommencer une partie
        if not self.alea:
            self.start = baller
        return self._get_state()
    
    #reinitialisation du terrain
    def reset(self):
        if not self.alea:
            self.position = self.start
            self.counter = 0
            return self._get_state()
        else:
            return self.generate_game()
    
    #fonction qui retourne la grille complète 9x9 en indiquant la postion x,y donnée
    def _get_grille(self, x, y):
        grille = [
            [0] * self.n for i in range(self.m)
        ]
        grille[x][y] = 1
        return grille
    
    #fonction qui recupère les 4 grilles correspondant aux positions des 4 robots et qui renvoie un vecteur, ce vecteur est l'ETAT de l'environnement
    def _get_state(self):

        return np.reshape([self._get_grille(x, y) for (x, y) in
                    [self.position, self.goal, self.defenseur, self.mate]],(1,4*self.n*self.m))
       
    def get_random_action(self):
        return random.choice(self.ACTIONS)
    
    #fonction pour savoir si le but est ouvert et donc si l'attaquant a reussi son objectif
    def openGoal(self,xd,yd):
        openGoal=True
        x,y=self.position_to_xy(xd,yd)
        
        xbut=1350
        ybut=0
         
        r_robot=170
        
        a,b=np.polyfit([x,xbut],[y,ybut],1)    
        for robot in [self.defenseur,self.goal,self.mate]:
            xr,yr=self.position_to_xy(robot[0], robot[1])
            if abs(yr-(xr*a+b))<r_robot:
                   openGoal=False
                   break
        return openGoal
    
    #fonction qui gère le deplacement du robot en fonction de l'action entrée
    #elle renvoie l'état suivant, la récompense, si la  partie est finie
    def move(self, action):
        """
        takes an action parameter
        :param action : the id of an action
        :return ((state_id, end, hole, block), reward, is_final, actions)
        """
        
        self.counter += 1

        if action not in self.ACTIONS:
            raise Exception("Invalid action")

       
        #obtention de la nouvelle position
        d_x, d_y = self.MOVEMENTS[action]
        x, y = self.position
        new_x, new_y = x + d_x, y + d_y
        
        
        #on vérifie que le deplacement est accepté (respect des limites du terrain, des surfaces et anti collision)
        if (new_x, new_y) not in self.cases:
            return self._get_state(), -3, False, self.ACTIONS 

        #on regarde si le but est libre dans la nouvelle position
        elif self.openGoal(new_x,new_y):
            self.position = new_x, new_y
            self.positionxy = self.position_to_xy(new_x, new_y)
            return self._get_state(), 10, True, self.ACTIONS
        
        
        #pour éviter d'avoir des parties trop longues
        elif self.counter > 100:
            self.position = new_x, new_y
            self.positionxy = self.position_to_xy(new_x, new_y)
            return self._get_state(), -1, True, self.ACTIONS
        
        #sinon on continu, récompense négative pour favoriser les plus courts chemins
        else:
            self.position = new_x, new_y
            self.positionxy = self.position_to_xy(new_x, new_y)
            return self._get_state(), -1, False, self.ACTIONS
    
    #fonction d'affichage du terrain (plot)
    def print(self):
        fig=plt.figure()
        ax = fig.add_subplot(111)
        ax.axis([-longueur,longueur,-largeur,largeur])
        grid_x=np.arange(-longueur,longueur,2*longueur/self.m)
        grid_y=np.arange(-largeur,largeur,2*largeur/self.n)
        ax.set_xticks(grid_x)
        ax.set_yticks(grid_y)
        ax.grid()
        
        ax.add_artist(lines.Line2D((-longueur, -longueur+350, -longueur+350,-longueur), (-350, -350,350,350), color = 'green'))
        ax.add_artist(lines.Line2D((longueur, longueur-350, longueur-350,longueur), (-350, -350,350,350), color = 'green'))
        ax.add_artist(lines.Line2D((0,0), (largeur,-largeur), color = 'green'))
        ax.add_artist(patches.Circle((0, 0), 500,facecolor='None', edgecolor = 'green'))
        
        ax.plot(self.goalxy[0],self.goalxy[1],'ro',ms=20)
        ax.text(self.goalxy[0],self.goalxy[1],'G',horizontalalignment='center',verticalalignment='center',color='w')
        
        ax.plot(self.matexy[0],self.matexy[1],'bo',ms=20)
        ax.text(self.matexy[0],self.matexy[1],'M',horizontalalignment='center',verticalalignment='center',color='w')
        
        ax.plot(self.positionxy[0],self.positionxy[1],'bo',ms=20)
        ax.text(self.positionxy[0],self.positionxy[1],'X',horizontalalignment='center',verticalalignment='center',color='w')
        
        ax.plot(self.defenseurxy[0],self.defenseurxy[1],'ro',ms=20)
        ax.text(self.defenseurxy[0],self.defenseurxy[1],'D',horizontalalignment='center',verticalalignment='center',color='w')
    
    
    #fonction d'affichage moins lourde
    def soft_print(self):
        terrain=''
        for j in range (self.n):
            ligne=''
            for i in range (self.m):
                if self.defenseur==(i,j):
                    car='-D-'
                elif self.goal==(i,j):
                    car='-G-'
                elif self.mate==(i,j):
                    car='-M-'
                elif self.position==(i,j):
                    car='-X-'
                else:
                    car='---'
                ligne+=car
            terrain+=ligne
            terrain+='\n'
        print(terrain)
    
    #sauvegarde du terrain grâce à un vecteur pour pouvoir réutiliser le terrain
    def save(self):
        return[self.position,self.goal,self.goalxy,self.defenseur,self.mate]
        

#definition de la classe de l'agent qui fera les prédictions  
class Agent:
    def __init__(self, name=None, learning_rate=0.001, epsilon_iteration=5000, batch_size=32, memory_size=10000):
        self.state_size = 4*81
        self.action_size = 4
        
        #Ensemble des hyperparamètres
        self.gamma = 0.8
        self.epsilon = 1.0
        self.epsilon_min = 0.1
        self.epsilon_iteration = epsilon_iteration
        self.learning_rate = learning_rate
        self.memory = deque(maxlen=memory_size)
        self.batch_size = batch_size
        
        self.name = name
        self.mse=tf.keras.losses.MeanSquaredError()
        
        #on peut charger un modèle sauvegardé précédement
        if name is not None and os.path.isdir("model-" + name):
            
            self.q_network = tf.keras.models.load_model("model-" + name)
            self.target_network = self.creation_reseau_de_neurone()
            # self.update_target() 
            
        else:
            #création des 2 reseaux de neurone utiles pour l'algorithme de deep Q learning
            self.q_network = self.creation_reseau_de_neurone() 
            self.target_network = self.creation_reseau_de_neurone()
            self.update_target()      
   
    #creation d'un reseau de neurone multicouche 'deep' (le nombre de couche et de neurone par couche sont aussi des hyperparamètres)
    def creation_reseau_de_neurone(self):
        model = tf.keras.models.Sequential()
        # model.add(tf.keras.layers.Flatten())
        model.add(tf.keras.layers.Dense(256,input_dim=self.state_size, activation='relu'))
        model.add(tf.keras.layers.Dense(64, activation='relu'))
        model.add(tf.keras.layers.Dense(64, activation='relu'))
        model.add(tf.keras.layers.Dense(self.action_size, activation='linear'))
        
        model.compile(loss='mse', optimizer=tf.keras.optimizers.Adam(lr=self.learning_rate))
        return model
    
    #update d'epsilon de détermine le rapport exploration/exploitation
    def decay_epsilon(self,i):
        self.epsilon = np.exp(i/self.epsilon_iteration*np.log(self.epsilon_min))
    
    #choix de l'action à faire en fonction d'epsilon et de l'état
    def get_best_action(self, state, rand=True):
        #EXPLORATION avec une proba de epsilon
        if rand and np.random.rand() <= self.epsilon:
            # The agent acts randomly
            return random.randrange(self.action_size)
        
        #EXPLOITATION sinon ie on choisit la meilleure action grace au reseau de neurone
        # Predict the reward value based on the given state
        act_values = self.q_network.predict_step(tf.constant(state))
        # print(tf.constant(state).shape)
       
        # Pick the action based on the predicted reward
        action =  np.argmax(act_values[0])  
        return action
    
    @tf.function #décorateur de fonction qui permet une prédiction plus rapide
    def predict(self,state):
        prediction = self.q_network(state)
        return prediction
    
    
    #on ajoute une transition dans la mémoire
    def remember(self, state, action, reward, next_state, done):
        self.memory.append([state, action, reward, next_state, done])
    
    #fonction qui permet à partir d'un batch de données d'améliorer le modèle
    def replay(self, batch_size):
        batch_size = min(batch_size, len(self.memory))

        minibatch = random.sample(self.memory, batch_size)

        losses=np.zeros(batch_size)
       
        for i, (state, action, reward, next_state, done) in enumerate(minibatch):
            state=tf.convert_to_tensor([state],dtype=tf.float32)
            l=self.optimize(state, action, reward, next_state, done)
            losses[i]=l    
        return np.mean(losses)
    
    
    #fonction qui calcule l'erreur du modèle suivant l'équation de Bellman
    #puis on applique une descente de gradient pour réduire cette erreur
    
    @tf.function
    def optimize(self,state, action, reward, next_state, done):
        with tf.GradientTape() as tape: 
            target = self.q_network(state)[0][0][action]
                    
            # print(target)
            if done:
                target1 = tf.constant(reward,dtype=tf.float32)
            else:
                act=self.target_network(next_state)
                ind=tf.math.argmax(act[0])
                target1 = tf.constant(reward,dtype=tf.float32) + self.gamma* act[0][ind]
                
                
            # print(inputs[i][0])
            # print(target[action])
            # print('target',target)
            # print('target1',target1)
            
            #calcul de l'erreur suivant la methode MeanSquaredError
            loss=(target-target1)**2 
            # print(loss)
            
            
            #calcul des gradients
            gradients = tape.gradient(loss,self.q_network.trainable_variables) 
            # print(gradients)
            
            #optimisation du modele 'q_network' suivant les gradients
            self.q_network.optimizer.apply_gradients(zip(gradients,self.q_network.trainable_variables))
        return 1
    
    #methode beaucoup plus lente
    def replay_2(self, batch_size):
        batch_size = min(batch_size, len(self.memory))

        minibatch = random.sample(self.memory, batch_size)

        inputs = np.zeros((batch_size, self.state_size))
        outputs = np.zeros((batch_size, self.action_size))

        for i, (state, action, reward, next_state, done) in enumerate(minibatch):
            target = self.q_network(state).numpy()[0]
            if done:
                target[action] = reward
            else:
                target[action] = reward+ self.gamma* np.max(self.q_network.predict(next_state))
                pass
            inputs[i] = state
            outputs[i] = target
        # print(inputs)
        
        # self.model.optimizer.minimize(model_loss(inputs,outputs,actions))
        return self.q_network.fit(inputs, outputs, epochs=1, verbose=0, batch_size=batch_size)
    
    
    #sauvegarde d'un modèle
    def save(self, id=None, overwrite=False,training=1,nb=2000):
        name = 'model'
        # if self.name is not None :
        #     name += '-' + self.name
        # else:
        name += '-' +'gamma'+ str(self.gamma) +'-'+'lr'+ str(self.learning_rate)+'-'+ 'method' + str(training) +'-' + 'episodes'+str(nb)
        if id:
            name = str(time.time())+'-' + id
        self.q_network.save(name, overwrite=overwrite)
    
    #mis à jour des poids du 'target_network' avec ceux du 'q_network'
    def update_target(self):
        self.target_network.set_weights(self.q_network.get_weights())
    
        
    #fonction d'entrainement de l'agent
    def train(self,episodes, alea, collecting=False, snapshot=5000):
        self.epsilon_iteration=episodes
        batch_size = 32
        g = Game(9, 9, alea=alea)
        # counter = 1
        scores = []
        global_counter = 0
        losses = [0]
        epsilons = []
        update=100
    
        # we start with a sequence to collect information, without learning pour remplir la mémoire
        if collecting:
            collecting_steps = 10000
            print("Collecting game without learning")
            steps = 0
            while steps < collecting_steps:
                state = g.reset()
                done = False
                print(steps)
                while not done:
                    steps += 1
                    action = g.get_random_action() #les actions sont choisies aléatoirement
                    next_state, reward, done, _ = g.move(action)
                    self.remember(state, action, reward, next_state, done)
                    state = next_state
        
        
        #début du véritable entrainement
        print("Starting training")  
        global_counter = 0
        for step in range(episodes+1):
            #print('Episode :',step)
            state = g.generate_game()
            state = np.reshape(state, (1,self.action_size*g.n*g.m))
            score = 0
            done = False
            steps = 0
            
            while not done:
                steps += 1
                global_counter += 1
                action = self.get_best_action(state)
                
                next_state, reward, done, _ = g.move(action)
                next_state = np.reshape(next_state, (1,self.action_size*g.n*g.m))
                score += reward
                self.remember(state, action, reward, next_state, done)
                state = next_state
                
                
                    
               
                if steps > 50:
                    break
                
                if global_counter%4==0: #toutes les 4 actions on réalise une optimisation du modèle, la fréquence d'optimisation est aussi un hyperparamètre
                    l = self.replay(batch_size)
                    # print(l)
                    losses.append(l)
                #print('Pas:',steps, 'Loss=',losses[-1])
                
               
                if global_counter % update == 0: #toutes les n actions on met à jour le q_network, hyperparamètre
                    self.update_target()
                
                #les 2 paramètres précedents ont une forte influence sur la convergence du modèle
            
            scores.append(score)
            self.decay_epsilon(step)
            epsilons.append(self.epsilon)
            
            
            # print('Episode :',step, 'score:',score)    
            if (step % 50 == 0)&(step!=0):
                print("episode: {}/{}, moves: {}, score: {}, epsilon: {}, loss: {}"
                      .format(step, episodes, steps, score, self.epsilon, losses[-1]))
                print('SCORE :',np.mean(scores[-51:-1]),' LOSS :',np.mean(losses[-51:-1]))
           
            
            
            if step > 0 and step % snapshot == 0:
                self.save(id='iteration-%s' % step)
            
            
        return scores, losses, epsilons
    
    
    
    #fonction plus lente et moins efficace
    def train_2(self,episodes, alea, collecting=False, snapshot=5000):
        self.epsilon_iteration=episodes
        batch_size = 32
        g = Game(9, 9, alea=alea)
        # counter = 1
        scores = []
        global_counter = 0
        losses = [0]
        epsilons = []
        
    
        # we start with a sequence to collect information, without learning
        if collecting:
            collecting_steps = 8000
            print("Collecting game without learning")
            steps = 0
            while steps < collecting_steps:
                state = g.reset()
                done = False
                print(steps)
                while not done:
                    steps += 1
                    action = g.get_random_action()
                    next_state, reward, done, _ = g.move(action)
                    self.remember(state, action, reward, next_state, done)
                    state = next_state

        print("Starting training")  
        global_counter = 0
        for e in range(episodes+1):
            state = g.generate_game()
            state = np.reshape(state, (1,self.action_size*g.n*g.m))
            score = 0
            done = False
            steps = 0
            while not done:
                steps += 1
                global_counter += 1
                action = self.get_best_action(state)
                
                next_state, reward, done, _ = g.move(action)
                next_state = np.reshape(next_state, (1,self.action_size*g.n*g.m))
                score += reward
                self.remember(state, action, reward, next_state, done)
                state = next_state
                
                if global_counter % 100 == 0:
                    l = self.replay_2(batch_size)
                    losses.append(l.history['loss'][0])
               
                if done:
                    scores.append(score)
                    epsilons.append(self.epsilon)
                
                if steps > 200:
                    break
            
            self.decay_epsilon(e)
            
            if (e % 50 == 0)&(e!=0):
                print("episode: {}/{}, moves: {}, score: {}, epsilon: {}, loss: {}"
                      .format(e, episodes, steps, score, self.epsilon, losses[-1]))
                print('SCORE :',np.mean(scores[-51:-1]),' LOSS :',np.mean(losses[-51:-1]))
           
           
            if e > 0 and e % snapshot == 0:
                self.save(id='iteration-%s' % e)
            
        return scores, losses, epsilons


#fonction pour lisser les résultats car très fortes dispersion
def smooth(vector, width=30):
    return np.convolve(vector, [1/width]*width, mode='valid')
    




def play_game(name_trainer=None,agent=None,game=False,disp=False):
    if name_trainer!=None:
        trainer=Agent(name=name_trainer)
    elif agent!=None:
        trainer=agent
    
    if game==False:
        g=Game(9,9,alea=True)
        state=g.generate_game()
        state = np.reshape(state, (1,trainer.action_size*g.n*g.m))
    else :
        g=Game(9,9,terrain=game)
        g.reset()
    done = False
    positions=[g.positionxy]
    score=0
    counter=0
    while not (done or (counter>20)):
        action = trainer.get_best_action(g._get_state(), rand=False)
        next_state, reward, done, _ = g.move(action)
        score+=reward
        counter+=1
        positions.append(g.positionxy)
        # if counter>20:
        #     g.print()
        
    print('score:',score)
    
    if disp:
        fig=plt.figure()
        ax = fig.add_subplot(111)
         
    
        for i in range(1*len(positions)):
            ax.clear()
            ax.axis([-longueur,longueur,-largeur,largeur])
            grid_x=np.arange(-longueur,longueur,2*longueur/9)
            grid_y=np.arange(-largeur,largeur,2*largeur/9)
            ax.set_xticks(grid_x)
            ax.set_yticks(grid_y)
            ax.grid()
            ax.plot(g.goalxy[0],g.goalxy[1],'ro',ms=20)
            ax.text(g.goalxy[0],g.goalxy[1],'G',horizontalalignment='center',verticalalignment='center',color='w')
            
            ax.plot(g.matexy[0],g.matexy[1],'bo',ms=20)
            ax.text(g.matexy[0],g.matexy[1],'M',horizontalalignment='center',verticalalignment='center',color='w')
            
            ax.plot(positions[i%len(positions)][0],positions[i%len(positions)][1],'bo',ms=20)
            ax.text(positions[i%len(positions)][0],positions[i%len(positions)][1],'X',horizontalalignment='center',verticalalignment='center',color='w')
            
            ax.plot(g.defenseurxy[0],g.defenseurxy[1],'ro',ms=20)
            ax.text(g.defenseurxy[0],g.defenseurxy[1],'D',horizontalalignment='center',verticalalignment='center',color='w')
            plt.pause(0.5)
            
        # ani=animation.FuncAnimation(fig,animate,frames=len(positions),interval=10)
        plt.show()
    return (positions[-1],score)

def play_game_2(name_trainer=None,agent=None,game=False,disp=False):
    if name_trainer!=None:
        trainer=Agent(name=name_trainer)
    elif agent!=None:
        trainer=agent
    if game==False:
        g=Game(alea=True)
        state=g.generate_game()
        state = np.reshape(state, (1,trainer.action_size*g.n*g.m))
    else :
        g=Game(terrain=game)
        g.reset()
    done = False
    positions=[g.positionxy]
    score=0
    counter=0
    while not (done or (counter>20)):
        action = np.argmax(trainer.q_network(g._get_state()).numpy())
        next_state, reward, done, _ = g.move(action)
        score+=reward
        counter+=1
        positions.append(g.positionxy)
        # if counter>20:
        #     g.print()
        
    print('score:',score)
    
    if disp:
        fig=plt.figure()
        ax = fig.add_subplot(111)
         
    
        for i in range(1*len(positions)):
            ax.clear()
            ax.axis([-longueur,longueur,-largeur,largeur])
            grid_x=np.arange(-longueur,longueur,2*longueur/11)
            grid_y=np.arange(-largeur,largeur,2*largeur/11)
            ax.set_xticks(grid_x)
            ax.set_yticks(grid_y)
            ax.grid()
            ax.plot(g.goalxy[0],g.goalxy[1],'ro',ms=20)
            ax.text(g.goalxy[0],g.goalxy[1],'G',horizontalalignment='center',verticalalignment='center',color='w')
            
            ax.plot(g.matexy[0],g.matexy[1],'bo',ms=20)
            ax.text(g.matexy[0],g.matexy[1],'M',horizontalalignment='center',verticalalignment='center',color='w')
            
            ax.plot(positions[i%len(positions)][0],positions[i%len(positions)][1],'bo',ms=20)
            ax.text(positions[i%len(positions)][0],positions[i%len(positions)][1],'X',horizontalalignment='center',verticalalignment='center',color='w')
            
            ax.plot(g.defenseurxy[0],g.defenseurxy[1],'ro',ms=20)
            ax.text(g.defenseurxy[0],g.defenseurxy[1],'D',horizontalalignment='center',verticalalignment='center',color='w')
            plt.pause(0.5)
            
        # ani=animation.FuncAnimation(fig,animate,frames=len(positions),interval=10)
        plt.show()
    return (positions[-1],score)


#fonction qui permet de jouer une partie jusqu'à ce que l'attaquant est le but libre
def play_game_3(name_trainer=None,agent=None,game=False,disp=False):
    if name_trainer!=None:
        trainer=Agent(name=name_trainer)
    elif agent!=None:
        trainer=agent
    if game==False:
        g=Game(9,9,alea=True)
        state=g.generate_game()
        state = np.reshape(state, (1,trainer.action_size*g.n*g.m))
    else :
        g=Game(9,9,terrain=game)
        g.reset()
    done = False
    positions=[g.positionxy]
    score=0
    counter=0
    random_action=False
    fail=False
    while not (done or (counter>20)):
        state=g._get_state()
        if not random_action:
             action = np.argmax(trainer.predict(state).numpy())
        else:
            action=random.randint(0,3)
        next_state, reward, done, _ = g.move(action)
        
        #si le robot execute une action interdite alors il revient au meme état, donc il prédira la même action
        #donc dans ce cas on choisit une action aléatoire pour le débloquer
        if list(next_state[0])==list(state[0]):
            random_action=True
        else :
            random_action=False
        score+=reward
        counter+=1
        positions.append(g.positionxy)
        if counter>20:
            fail=True
        
    print('score:',score)
    
    #on peut afficher les deplacements du robot
    if disp & (score<8):
        fig=plt.figure()
        ax = fig.add_subplot(111)
         
    
        for i in range(1*len(positions)):
            ax.clear()
            ax.axis([-longueur,longueur,-largeur,largeur])
            grid_x=np.arange(-longueur,longueur,2*longueur/9)
            grid_y=np.arange(-largeur,largeur,2*largeur/9)
            ax.set_xticks(grid_x)
            ax.set_yticks(grid_y)
            ax.grid()
            ax.plot(g.goalxy[0],g.goalxy[1],'ro',ms=20)
            ax.text(g.goalxy[0],g.goalxy[1],'G',horizontalalignment='center',verticalalignment='center',color='w')
            
            ax.plot(g.matexy[0],g.matexy[1],'bo',ms=20)
            ax.text(g.matexy[0],g.matexy[1],'M',horizontalalignment='center',verticalalignment='center',color='w')
            
            ax.plot(positions[i%len(positions)][0],positions[i%len(positions)][1],'bo',ms=20)
            ax.text(positions[i%len(positions)][0],positions[i%len(positions)][1],'X',horizontalalignment='center',verticalalignment='center',color='w')
            
            ax.plot(g.defenseurxy[0],g.defenseurxy[1],'ro',ms=20)
            ax.text(g.defenseurxy[0],g.defenseurxy[1],'D',horizontalalignment='center',verticalalignment='center',color='w')
            
            ax.add_artist(lines.Line2D((-longueur, -longueur+350, -longueur+350,-longueur), (-350, -350,350,350), color = 'green'))
            ax.add_artist(lines.Line2D((longueur, longueur-350, longueur-350,longueur), (-350, -350,350,350), color = 'green'))
            ax.add_artist(lines.Line2D((0,0), (largeur,-largeur), color = 'green'))
            ax.add_artist(patches.Circle((0, 0), 500,facecolor='None', edgecolor = 'green'))
       
            plt.pause(7.5)
            
        # ani=animation.FuncAnimation(fig,animate,frames=len(positions),interval=10)
        plt.show()
    return (positions[-1],score,fail)#on retourne la position finale, le score total et si l'objectif a été atteint


#fonction pour entrainer un agent et afficher sa progression
def train_ia(nb=2000,training=1):
    trainer =Agent(learning_rate=0.001,epsilon_iteration=nb)
    if training==1:
        scores, losses, epsilons = trainer.train(nb,  True,True, snapshot=2500)
    else:
        scores, losses, epsilons = trainer.train_2(nb,  True,True, snapshot=2500)
    
    trainer.save(training=training,nb=nb)
    sc = smooth(scores, width=500)
    los= smooth(losses, width=500)
    
    fig=plt.figure()
    ax1=fig.add_subplot(211)
    ax1.plot(sc)
    ax3= ax1.twinx()
    ax3.plot(epsilons,color='r')
    ax3.tick_params('y', colors='r')
    ax2 = fig.add_subplot(212)
    ax2.plot(los, color='g')
    ax1.set_ylabel('Score')
    ax2.set_ylabel('Losses', color='g')
    ax2.tick_params('y', colors='g')
    plt.title("Score, and Epsilon over training")
    ax1.set_xlabel("Episodes")
    
         

    plt.savefig(str(time.time())+'.png')
    plt.show()
  
#fonction pour comparer 2 ia sur un ensemble de partie identique
def comparaison_ia(name_ia1,name_ia2,nombre_de_parties,disp=False):
    start_time = time.time()  
  
    ia1=Agent(name_ia1)
    ia2=Agent(name_ia2)
    score1=[]
    score2=[]
    game=[]
    final_pos1=[]
    final_pos2=[]
    echec1=0
    echec2=0
    for i in range(nombre_de_parties):
        g=Game(9,9)
        terrain=g.save()
        game.append(g)
        pos1,sc1,fail1=play_game_3(agent=ia1,game=terrain)
        pos2,sc2,fail2=play_game_3(agent=ia2,game=terrain)
        if fail1:
            echec1+=1
        if fail2:
            echec2+=1
        score1.append(sc1)
        score2.append(sc2)
        final_pos1.append(pos1)
        final_pos2.append(pos2)
    
    interval = time.time()-start_time  
    print ('Total time in seconds:', interval)    
    
    #on compare le cumul des scores, + il grand,+l'ia est rapide et efficace
    #De plus, on calcul de pourcentage d'echec
    print('Score total ia1:',sum(score1),'Score total ia2:',sum(score2))
    print('Echec ia1:',100*echec1/nombre_de_parties,'% ','Echec ia2:',100*echec2/nombre_de_parties,'%')
    
    #affichage des 2 positions finales pour pouvoir les comparer
    if disp:
        fig=plt.figure()
        ax = fig.add_subplot(111)
         
    
        for i in range(nombre_de_parties):
            ax.clear()
            ax.axis([-longueur,longueur,-largeur,largeur])
            grid_x=np.arange(-longueur,longueur,2*longueur/9)
            grid_y=np.arange(-largeur,largeur,2*largeur/9)
            ax.set_xticks(grid_x)
            ax.set_yticks(grid_y)
            ax.grid()
            ax.plot(game[i].goalxy[0],game[i].goalxy[1],'ro',ms=20)
            ax.text(game[i].goalxy[0],game[i].goalxy[1],'G',horizontalalignment='center',verticalalignment='center',color='w')
            
            ax.plot(game[i].matexy[0],game[i].matexy[1],'bo',ms=20)
            ax.text(game[i].matexy[0],game[i].matexy[1],'M',horizontalalignment='center',verticalalignment='center',color='w')
            
            ax.plot(game[i].positionxy[0],game[i].positionxy[1],'bo',ms=20)
            ax.text(game[i].positionxy[0],game[i].positionxy[1],'X',horizontalalignment='center',verticalalignment='center',color='w')
            
            ax.plot(game[i].defenseurxy[0],game[i].defenseurxy[1],'ro',ms=20)
            ax.text(game[i].defenseurxy[0],game[i].defenseurxy[1],'D',horizontalalignment='center',verticalalignment='center',color='w')
            
            ax.plot(final_pos1[i][0],final_pos1[i][1],'go')
            ax.text(final_pos1[i][0],final_pos1[i][1],'ia1')
            
            ax.plot(final_pos2[i][0],final_pos2[i][1],'mo')
            ax.text(final_pos2[i][0],final_pos2[i][1],'ia2')
            
            ax.add_artist(lines.Line2D((-longueur, -longueur+350, -longueur+350,-longueur), (-350, -350,350,350), color = 'green'))
            ax.add_artist(lines.Line2D((longueur, longueur-350, longueur-350,longueur), (-350, -350,350,350), color = 'green'))
            ax.add_artist(lines.Line2D((0,0), (largeur,-largeur), color = 'green'))
            ax.add_artist(patches.Circle((0, 0), 500,facecolor='None', edgecolor = 'green'))
            plt.pause(3.5)    
        plt.show()  
        
