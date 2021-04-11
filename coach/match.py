# -*- coding: utf-8 -*-
"""
Created on Fri Mar 12 14:26:38 2021

@author: paulg
"""

import potentiel as pt
import robot as rbt
import coach as coach
import numpy as np
import time

import parametres as p



class Match():
    
    def __init__(self,nom,vision,grSim,com,controlledTeams='B',blueSide='L',start='B',disp=0):
        self.nom=nom
        Y0=rbt.Robot('Y',0,self,grSim,com)
        Y1=rbt.Robot('Y',1,self,grSim,com)
        B0=rbt.Robot('B',0,self,grSim,com)
        B1=rbt.Robot('B',1,self,grSim,com)
        self.n=9
        self.m=9
        self.disp=disp
        self.vision=vision
        self.joueurs=[Y0,Y1,B0,B1]
        self.controlledTeams=controlledTeams
        self.blueSide=blueSide
        if blueSide=='L':
            yellowSide='R'
        else:
            yellowSide='L'
        Yellow=coach.Coach([self.joueurs[0],self.joueurs[1]],'Y',yellowSide)
        Blue=coach.Coach([self.joueurs[2],self.joueurs[3]],'B',blueSide)
        self.balle=Balle(0,0)
        self.blue=Blue
        self.yellow=Yellow
        
        self.stop=False
        self.go=False
        self.score_jaune=0
        self.score_bleu=0
        self.team_engagement=start
        self.engagement=True
        self.start_pause=time.time()
    
    def __repr__(self):
        return f'{self.nom}'
    
    # def getVision(self):
    #     while not self.vision.isDataReady(): 
    #         continue
  
    #     return self.vision.getLocations()
    # #Récupération des données de SSL Vision et actualisation des robots et de la balle + affichage
    # def Vision(self):
    #     balls, blueBots, yellowBots = self.getVision()
  
    #     for robot in self.joueurs:
    #         data=False
    #         if robot.team=='Y':
    #             tag='gold'
    #             for bot in yellowBots:
    #                 if bot[5]==robot.id:
    #                     data=True
    #                     botInfo=bot
    #         else :
    #             tag='b'
    #             for bot in blueBots:
    #                 if bot[5]==robot.id:
    #                     data=True
    #                     botInfo=bot
            
    #         #parfois il manque une donnée pour un robot
    #         if data: 
    #             robot.position(botInfo)
    #         else :
    #             self.Vision()
            
            
        
        
    #     if len(balls)>0:
    #         self.balle.Position(balls[0])

    def getVision(self):
        # print(self.vision.isDataReady())
        if self.vision.isDataReady(): 
            
  
            return self.vision.getLocations()
        else :
            return None,None,None
    
    #Récupération des données de SSL Vision et actualisation des robots et de la balle + affichage
    def Vision(self):
        balls, blueBots, yellowBots = self.getVision()
        if balls!=None:
            for robot in self.joueurs:
                data=False
                if robot.team=='Y':
                    
                    for bot in yellowBots:
                        if bot[5]==robot.id:
                            data=True
                            botInfo=bot
                else :
                    
                    for bot in blueBots:
                        if bot[5]==robot.id:
                            data=True
                            botInfo=bot
                
                #parfois il manque une donnée pour un robot
                if data: 
                    robot.position(botInfo)
               
                
                
            
            
            if len(balls)>0:
                self.balle.Position(balls[0])
                ball=balls[0]
                ballex,balley=ball[6],ball[7]
                if (ballex>1350) and(abs(balley)<175)and self.go:
                    print('but droite')
                    self.start_pause=time.time()
                    if self.blue.side=='L':
                        self.but_bleu()
                    else:
                        self.but_jaune()
                        
                elif (ballex<-1350) and(abs(balley)<175)and self.go:
                    print('but gauche')
                    self.start_pause=time.time()
                    if self.blue.side=='R':
                        self.but_bleu()
                    else:
                        self.but_jaune()
               
            
        
        
    #Fonction pour passer d'une position x,y vers une position discrète dans une grille de 9x9 (utile pour l'ia et la création du terrain)
    def xy_to_position(self,x,y,n,m):
        return(int(int(x+p.longueur)//(2*p.longueur/m)),int(int(y+p.largeur)//(2*p.largeur/n)))
    
    #Fonction inverse de la précédente car l'ia renvoie une position sur la grille de 9x9
    def position_to_xy(self,xd,yd,n,m):
        return(-p.longueur+(xd+0.5)*(2*p.longueur)/m,-p.largeur+(yd+0.5)*(2*p.largeur)/n)
    
    def Stop(self):
        self.stop=True
        print('STOP')

    def Go(self):
        self.go=True
        self.stop=False
        self.engagement=False
        print('GO')
        
    def but_jaune(self):
        print('BUT JAUNE')
        self.score_jaune+=1
        self.team_engagement='B'
        print(f'Le score est BLEU {self.score_bleu} - {self.score_jaune} JAUNE')
        self.engagement=True
        self.go=False
        
    def but_bleu(self):
        print('BUT BLEU')
        self.score_bleu+=1
        self.team_engagement='Y'
        print(f'Le score est BLEU {self.score_bleu} - {self.score_jaune} JAUNE')
        self.engagement=True
        self.go=False
        
    def regame(self):
        self.stop=False
        print('NOUVEAU MATCH')
        self.score_bleu=0
        self.score_jaune=0
        self.engagement=True
    
    def Reset(self):
        self.Vision()
        if 'B' in self.controlledTeams:
            self.blue.reset()
        if 'Y' in self.controlledTeams:
            self.yellow.reset()
        
    def Engagement(self):
        self.Vision()
        if 'B' in self.controlledTeams:
            self.blue.engagement()
        if 'Y' in self.controlledTeams:
            self.yellow.engagement()
        
    def Play(self):
        #Actualisation des positions + detection but 
        self.Vision()
        
        #Controle des bleus
        if 'B' in self.controlledTeams:
            self.blue.changementDePoste()
            self.blue.action()
        
        #Controle des jaunes
        if 'Y' in self.controlledTeams:
            self.yellow.changementDePoste()
            self.yellow.action()
        
    
class Balle():
    
    def __init__(self,x,y):
        self.x=x
        self.y=y
        self.potentiel=pt.Potentiel(-3,[self.x,self.y],p.x_grid,p.y_grid,2500) 
        self.positionc=complex(x,y)
        self.x10=[x]
        self.y10=[y]
        self.t10=[0]
        
    def Position(self,ball): 
        self.x,self.y,t=ball[6],ball[7],ball[1]
        self.positionc=complex(self.x,self.y)
        self.potentiel=pt.Potentiel(-3,[self.x,self.y],p.x_grid,p.y_grid,2500) #potentiel négatif pour attirer les robots
        
        if len(self.x10)>9:
            self.x10.pop(0)
            self.y10.pop(0)
            self.t10.pop(0)
        self.x10.append(self.x) #on enregistre les 10 dernieres positions de la balle
        self.y10.append(self.y)
        self.t10.append(t)
        
            
        
            
    #Calcul du vecteur correspodant à la trajectoire de la balle    
    def trajectoire(self):
        a,b=np.polyfit(self.x10,self.y10,1)
        vect_balle=complex(1,a)
        if (self.x10[-1]-self.x10[0])<0:
            vect_balle=-vect_balle
        return vect_balle
        
    def vitesse(self):
        dt=self.t10[-1]-self.t10[0]
        if dt!=0:
            dx=self.x10[-1]-self.x10[0]
            dy=self.y10[-1]-self.y10[0]
            dl=np.sqrt(dx**2+dy**2)
            
            return dl/dt
    

   


