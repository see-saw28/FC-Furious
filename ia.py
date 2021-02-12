#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Feb  7 16:03:46 2021

@author: psl
"""
import random
import numpy as np
import tensorflow as tf
# from keras.models import Sequential, load_model
# from keras.layers.core import Dense, Activation
# from keras.optimizers import Adam
import random
import time
import os
import matplotlib.pyplot as plt
from collections import deque

flatten = lambda l: [item for sublist in l for item in sublist]
longueur=1350
largeur=1000


class Game:
    ACTION_PASSE = 4
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

    def __init__(self, n, m, alea=False,terrain=None):
        self.n = n
        self.m = m
        self.alea = alea
        if terrain==None:
            self.generate_game()
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
        
        
            

    def _position_to_id(self, x, y):
        """Donne l'identifiant de la position entre 0 et 15"""
        return x + y * self.n

    def _id_to_position(self, id):
        """Réciproque de la fonction précédente"""
        return (id % self.n, id // self.n)
    
    def position_to_xy(self,xd,yd):
        return(-longueur+(xd+0.5)*(2*longueur)/self.m,-largeur+(yd+0.5)*(2*largeur)/self.n)
    
    def xy_to_position(self,x,y):
        return(int(int(x+longueur)//(2*longueur/self.m)),int(int(y+largeur)//(2*largeur/self.n)))
    
    def generate_game(self):
        cases = [(x, y) for x in range(self.n-1) for y in range(self.m)]
        xbut=1350
        ybut=0
        
        baller = random.choice(cases)
        while baller[0]>=self.n-2:
            baller = random.choice(cases)
        cases.remove(baller)
        
        xb,yb=self.position_to_xy(baller[0], baller[1])
        a,b=np.polyfit([xb,xbut],[yb,ybut],1)
        xdg=random.randrange(baller[0]+1,self.n-1)
        xg,yg=self.position_to_xy(xdg,0)
        yg=a*xg+b
        xdg,ydg=self.xy_to_position(xg, yg)
        goal=(xdg,ydg)
        
        cases.remove(goal)
        
        defenseur = random.choice(cases)
        cases.remove(defenseur)
        xdef,ydef=self.position_to_xy(defenseur[0], defenseur[1])
       
        mate = random.choice(cases)
        cases.remove(mate)
        xm,ym=self.position_to_xy(mate[0], mate[1])
        

        self.position = baller
        self.positionxy=(xb,yb)
        self.goal = goal
        self.goalxy=(xg,yg)
        self.defenseur = defenseur
        self.defenseurxy=(xdef,ydef)
        self.mate = mate
        self.matexy=(xm,ym)
        self.counter = 0
        
        if not self.alea:
            self.start = baller
        return self._get_state()
    
    def reset(self):
        if not self.alea:
            self.position = self.start
            self.counter = 0
            return self._get_state()
        else:
            return self.generate_game()

    def _get_grille(self, x, y):
        grille = [
            [0] * self.n for i in range(self.m)
        ]
        grille[x][y] = 1
        return grille

    def _get_state(self):
        # x, y = self.position
        
        return np.reshape([self._get_grille(x, y) for (x, y) in
                    [self.position, self.goal, self.defenseur, self.mate]],(1,4*self.n*self.m))
        # else:
        #     return np.reshape(self._get_grille(x, y),(1,4*self.n*self.m))
    
    def get_random_action(self):
        return random.choice(self.ACTIONS)
    
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
    
    def move(self, action):
        """
        takes an action parameter
        :param action : the id of an action
        :return ((state_id, end, hole, block), reward, is_final, actions)
        """
        
        self.counter += 1

        if action not in self.ACTIONS:
            raise Exception("Invalid action")

        # random actions sometimes (2 times over 10 default)
        # choice = random.random()
        # if choice < self.wrong_action_p:
        #     action = (action + 1) % 4
        # elif choice < 2 * self.wrong_action_p:
        #     action = (action - 1) % 4

        d_x, d_y = self.MOVEMENTS[action]
        x, y = self.position
        new_x, new_y = x + d_x, y + d_y

        if (new_x, new_y)in [self.defenseur,self.goal,self.mate]:
            return self._get_state(), -3, False, self.ACTIONS
        
        elif new_x >= self.n or new_y >= self.m or new_x < 0 or new_y < 0:
            return self._get_state(), -1, False, self.ACTIONS
        
        elif not self.openGoal(new_x,new_y):
            self.position = new_x, new_y
            self.positionxy = self.position_to_xy(new_x, new_y)
            return self._get_state(), -1, False, self.ACTIONS
        
        elif self.openGoal(new_x,new_y):
            self.position = new_x, new_y
            self.positionxy = self.position_to_xy(new_x, new_y)
            return self._get_state(), 10, True, self.ACTIONS
        
        elif self.counter > 20:
            self.position = new_x, new_y
            self.positionxy = self.position_to_xy(new_x, new_y)
            return self._get_state(), -1, True, self.ACTIONS
        else:
            self.position = new_x, new_y
            self.positionxy = self.position_to_xy(new_x, new_y)
            return self._get_state(), -1, False, self.ACTIONS

    def print(self):
        fig=plt.figure()
        ax = fig.add_subplot(111)
        ax.axis([-longueur,longueur,-largeur,largeur])
        grid_x=np.arange(-longueur,longueur,2*longueur/self.m)
        grid_y=np.arange(-largeur,largeur,2*largeur/self.n)
        ax.set_xticks(grid_x)
        ax.set_yticks(grid_y)
        ax.grid()
       
        ax.plot(self.goalxy[0],self.goalxy[1],'ro',ms=20)
        ax.text(self.goalxy[0],self.goalxy[1],'G',horizontalalignment='center',verticalalignment='center',color='w')
        
        ax.plot(self.matexy[0],self.matexy[1],'bo',ms=20)
        ax.text(self.matexy[0],self.matexy[1],'M',horizontalalignment='center',verticalalignment='center',color='w')
        
        ax.plot(self.positionxy[0],self.positionxy[1],'bo',ms=20)
        ax.text(self.positionxy[0],self.positionxy[1],'X',horizontalalignment='center',verticalalignment='center',color='w')
        
        ax.plot(self.defenseurxy[0],self.defenseurxy[1],'ro',ms=20)
        ax.text(self.defenseurxy[0],self.defenseurxy[1],'D',horizontalalignment='center',verticalalignment='center',color='w')
    
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
        
    def save(self):
        return[self.position,self.goal,self.goalxy,self.defenseur,self.mate]
        

class Trainer:
    def __init__(self, name=None, learning_rate=0.001, epsilon_decay=0.9999, batch_size=30, memory_size=3000):
        self.state_size = 4*121
        self.action_size = 4
        self.gamma = 0.9
        self.epsilon = 1.0
        self.epsilon_min = 0.01
        self.epsilon_decay = epsilon_decay
        self.learning_rate = learning_rate
        self.memory = deque(maxlen=memory_size)
        self.batch_size = batch_size
        
        self.name = name
        if name is not None and os.path.isdir("model-" + name):
            
            model = tf.keras.models.load_model("model-" + name)
            # self.epsilon=0
            
        else:
            model = tf.keras.models.Sequential()
            model.add(tf.keras.layers.Dense(100, activation='relu'))
            model.add(tf.keras.layers.Dense(60, activation='relu'))
            model.add(tf.keras.layers.Dense(60, activation='relu'))
            model.add(tf.keras.layers.Dense(self.action_size, activation='linear'))
            model.compile(loss='mse', optimizer=tf.keras.optimizers.Adam(lr=self.learning_rate))
        
        self.model = model      
    
    def decay_epsilon(self):
        self.epsilon *= self.epsilon_decay
       
    def get_best_action(self, state, rand=True):
        

        if rand and np.random.rand() <= self.epsilon:
            # The agent acts randomly
            return random.randrange(self.action_size)
        
        # Predict the reward value based on the given state
        act_values = self.model.predict_step(tf.constant(state))
        print(tf.constant(state).shape)
        # Pick the action based on the predicted reward
        action =  np.argmax(act_values[0])  
        return action

    def remember(self, state, action, reward, next_state, done):
        self.memory.append([state, action, reward, next_state, done])

    def replay(self, batch_size):
        batch_size = min(batch_size, len(self.memory))

        minibatch = random.sample(self.memory, batch_size)

        inputs = np.zeros((batch_size, self.state_size))
        outputs = np.zeros((batch_size, self.action_size))

        for i, (state, action, reward, next_state, done) in enumerate(minibatch):
            target = self.model.predict(state)[0]
            if done:
                target[action] = reward
            else:
                target[action] = reward + self.gamma * np.max(self.model.predict(next_state))

            inputs[i] = state
            outputs[i] = target

        return self.model.fit(inputs, outputs, epochs=1, verbose=0, batch_size=batch_size)

    def save(self, id=None, overwrite=False):
        name = 'model'
        if self.name:
            name += '-' + self.name
        else:
            name += '-' + str(time.time())
        if id:
            name += '-' + id
        self.model.save(name, overwrite=overwrite)

def smooth(vector, width=30):
    return np.convolve(vector, [1/width]*width, mode='valid')
    
def train(episodes, trainer, alea, collecting=False, snapshot=5000):
    batch_size = 32
    g = Game(11, 11, alea=alea)
    # counter = 1
    scores = []
    global_counter = 0
    losses = [0]
    epsilons = []

    # we start with a sequence to collect information, without learning
    if collecting:
        collecting_steps = 1000
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
                trainer.remember(state, action, reward, next_state, done)
                state = next_state

    print("Starting training")  
    global_counter = 0
    for e in range(episodes+1):
        state = g.generate_game()
        state = np.reshape(state, (1,trainer.action_size*g.n*g.m))
        score = 0
        done = False
        steps = 0
        while not done:
            steps += 1
            global_counter += 1
            action = trainer.get_best_action(state)
            trainer.decay_epsilon()
            next_state, reward, done, _ = g.move(action)
            next_state = np.reshape(next_state, (1,trainer.action_size*g.n*g.m))
            score += reward
            trainer.remember(state, action, reward, next_state, done)
            state = next_state
            if global_counter % 100 == 0:
                l = trainer.replay(batch_size)
                losses.append(l.history['loss'][0])
            if done:
                scores.append(score)
                epsilons.append(trainer.epsilon)
            if steps > 200:
                break
        if (e % 20 == 0)&(e!=0):
            print("episode: {}/{}, moves: {}, score: {}, epsilon: {}, loss: {}"
                  .format(e, episodes, steps, score, trainer.epsilon, losses[-1]))
            print(np.mean(scores[-21:-1]))
        if e > 0 and e % snapshot == 0:
            trainer.save(id='iteration-%s' % e)
    return scores, losses, epsilons


    def save(self):
        if self.name:
            self.model.save("model-" + self.name, overwrite=True)
        else:
            self.model.save("model-" + str(time.time()))





def play_game(name_trainer,game=False,disp=False):
    trainer=Trainer(name=name_trainer)
    if game==False:
        g=Game(11,11,alea=True)
        state=g.generate_game()
        state = np.reshape(state, (1,trainer.action_size*g.n*g.m))
    else :
        g=Game(11,11,terrain=game)
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

def train_ia():
    trainer =Trainer(learning_rate=0.001, epsilon_decay=0.999995)
    scores, losses, epsilons = train(500, trainer, True,True, snapshot=2500)
    trainer.save()
    sc = smooth(scores, width=500)
    
    
    fig, ax1 = plt.subplots()
    ax1.plot(scores)
    ax2 = ax1.twinx()
    ax2.plot(epsilons, color='r')
    ax1.set_ylabel('Score')
    ax2.set_ylabel('Epsilon', color='r')
    ax2.tick_params('y', colors='r')
    plt.title("Score, and Epsilon over training")
    ax1.set_xlabel("Episodes")
    plt.figure()
  