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
import tensorflow as tf
# from keras.models import Sequential, load_model
# from keras.layers.core import Dense, Activation
# from keras.optimizers import Adam
import random
import time
import os
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as lines
from collections import deque
import copy

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

    def __init__(self, n=9, m=9, alea=False,terrain=None):
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
            cases = [(x, y) for x in range(self.n) for y in range(self.m)]
       
            cases.remove((0,3))
            cases.remove((0,4))
            cases.remove((0,5))
            cases.remove((8,3))
            cases.remove((8,4))
            cases.remove((8,5))
            
            for case in (self.goal,self.defenseur,self.mate,self.start):
                if case in cases:
                    cases.remove(case)
                    
            other_cases=[(8,2),(7,2),(7,3),(7,4),(7,5),(7,6),(8,6)]
            for case in other_cases:
                if case in cases:
                    cases.remove(case)
            self.cases=cases
            

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
        cases = [(x, y) for x in range(self.n) for y in range(self.m)]
        xbut=1350
        ybut=0
        cases.remove((0,3))
        cases.remove((0,4))
        cases.remove((0,5))
        cases.remove((8,3))
        cases.remove((8,4))
        cases.remove((8,5))
        
        cases_prime=copy.deepcopy(cases)
        cases_prime.remove((8,2))
        cases_prime.remove((7,2))
        cases_prime.remove((7,3))
        cases_prime.remove((7,4))
        cases_prime.remove((7,5))
        cases_prime.remove((7,6))
        cases_prime.remove((8,6))
        
        baller = random.choice(cases_prime)
        cases.remove(baller)
        
        xb,yb=self.position_to_xy(baller[0], baller[1])
        a,b=np.polyfit([xb,xbut],[yb,ybut],1)
        if baller[0]<7:
            xdg=random.randrange(baller[0]+1,self.n-1)
            xg,yg=self.position_to_xy(xdg,0)
            yg=a*xg+b
            xdg,ydg=self.xy_to_position(xg, yg)
        else :
            if baller[1]<3:
                ydg=2
            elif baller[1]>5:
                ydg=6
            xg,yg=self.position_to_xy(0,ydg)
            xg=(yg-b)/a
            xdg,ydg=self.xy_to_position(xg, yg)
        goal=(xdg,ydg)
        
        cases.remove(goal)
        
        defenseur = random.choice(cases)
        cases.remove(defenseur)
        xdef,ydef=self.position_to_xy(defenseur[0], defenseur[1])
       
        mate = random.choice(cases)
        cases.remove(mate)
        xm,ym=self.position_to_xy(mate[0], mate[1])
        
        other_cases=[(8,2),(7,2),(7,3),(7,4),(7,5),(7,6),(8,6),(8,1)(8,7)]
        for case in other_cases:
            if case in cases:
                cases.remove(case)
        
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

       

        d_x, d_y = self.MOVEMENTS[action]
        x, y = self.position
        new_x, new_y = x + d_x, y + d_y
        new_X,new_Y=self.position_to_xy(new_x, new_y)
        

        if (new_x, new_y) not in self.cases:
            return self._get_state(), -3, False, self.ACTIONS
        
              
        
        elif (self.openGoal(new_x,new_y))&(new_X>100):
            self.position = new_x, new_y
            self.positionxy = self.position_to_xy(new_x, new_y)
            
            return self._get_state(), 20, True, self.ACTIONS
        
        # elif not self.openGoal(new_x,new_y):
        #     self.position = new_x, new_y
        #     self.positionxy = self.position_to_xy(new_x, new_y)
        #     return self._get_state(), -1, False, self.ACTIONS
        
        elif self.counter > 100:
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
                elif (i,j) in self.cases:
                    car='---'
                else :
                    car='***'
                ligne+=car
            terrain+=ligne
            terrain+='\n'
        print(terrain)
        
    def save(self):
        return[self.position,self.goal,self.goalxy,self.defenseur,self.mate]
        

class Agent:
    def __init__(self, name=None, learning_rate=0.001, epsilon_iteration=5000, batch_size=32, memory_size=10000):
        self.state_size = 4*81
        self.action_size = 4
        self.gamma = 0.9
        self.epsilon = 1.0
        self.epsilon_min = 0.1
        self.epsilon_iteration = epsilon_iteration
        self.learning_rate = learning_rate
        self.memory = deque(maxlen=memory_size)
        self.batch_size = batch_size
        
        self.name = name
        self.mse=tf.keras.losses.MeanSquaredError()

        if name is not None and os.path.isdir("model-" + name):
            
            self.q_network = tf.keras.models.load_model("model-" + name)
            self.target_network = self.creation_agent()
            # self.update_target() 
            
        else:
            
            self.q_network = self.creation_agent() 
            self.target_network = self.creation_agent()
            self.update_target()      
   
    
    def creation_agent(self):
        model = tf.keras.models.Sequential()
        # model.add(tf.keras.layers.Flatten())
        model.add(tf.keras.layers.Dense(256,input_dim=self.state_size, activation='relu'))
        model.add(tf.keras.layers.Dense(64, activation='relu'))
        model.add(tf.keras.layers.Dense(64, activation='relu'))
        model.add(tf.keras.layers.Dense(self.action_size, activation='linear'))
        
        model.compile(loss='mse', optimizer=tf.keras.optimizers.Adam(lr=self.learning_rate))
        return model
    
    def decay_epsilon(self,i):
        self.epsilon = np.exp(i/self.epsilon_iteration*np.log(self.epsilon_min))
       
    def get_best_action(self, state, rand=True):
        if rand and np.random.rand() <= self.epsilon:
            # The agent acts randomly
            return random.randrange(self.action_size)
        
        # Predict the reward value based on the given state
        act_values = self.q_network.predict_step(tf.constant(state))
        # print(tf.constant(state).shape)
        # Pick the action based on the predicted reward
        action =  np.argmax(act_values[0])  
        return action
    
    @tf.function
    def predict(self,state):
        prediction = self.q_network(state)
        return prediction
    
    
    def remember(self, state, action, reward, next_state, done):
        self.memory.append([state, action, reward, next_state, done])
    
   
    def replay(self, batch_size):
        batch_size = min(batch_size, len(self.memory))

        minibatch = random.sample(self.memory, batch_size)

        losses=np.zeros(batch_size)
        
        
        for i, (state, action, reward, next_state, done) in enumerate(minibatch):
            state=tf.convert_to_tensor([state],dtype=tf.float32)
            l=self.optimize(state, action, reward, next_state, done)
            losses[i]=l    
        return np.mean(losses)
    
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
            loss=(target-target1)**2
            # print(loss)
      
            gradients = tape.gradient(loss,self.q_network.trainable_variables)
            # print(gradients)
           
            self.q_network.optimizer.apply_gradients(zip(gradients,self.q_network.trainable_variables))
        return loss
   
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

    def save(self, id=None, overwrite=False,training=1,nb=2000):
        name = 'model'
        # if self.name is not None :
        #     name += '-' + self.name
        # else:
        name += '-' +'gamma'+ str(self.gamma) +'-'+'lr'+ str(self.learning_rate)+'-'+ 'method' + str(training) +'-' + 'episodes'+str(nb)
        if id:
            name = str(time.time())+'-' + id
        self.q_network.save(name, overwrite=overwrite)

    def update_target(self):
        self.target_network.set_weights(self.q_network.get_weights())
        
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
    
        # we start with a sequence to collect information, without learning
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
                    action = g.get_random_action()
                    next_state, reward, done, _ = g.move(action)
                    self.remember(state, action, reward, next_state, done)
                    state = next_state

        print("Starting training")  
        global_counter = 0
        fail=0
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
                    fail+=1
                    print(fail)
                    break
                
                if global_counter%4==0:
                    l = self.replay(batch_size)
                    # print(l)
                    losses.append(l)
                #print('Pas:',steps, 'Loss=',losses[-1])
                
               
                if global_counter % update == 0:
                    self.update_target()
            
            
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
    return (positions[-1],score,fail,counter)

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
    
    print('Score total ia1:',sum(score1),'Score total ia2:',sum(score2))
    print('Echec ia1:',100*echec1/nombre_de_parties,'% ','Echec ia2:',100*echec2/nombre_de_parties,'%')
    
    
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
        
