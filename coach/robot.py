# -*- coding: utf-8 -*-
"""
Created on Fri Mar 12 14:23:15 2021

@author: paulg
"""

import potentiel as pt


from psl_package import paris_saclay_league as psl
import cmath as c
import time
import numpy as np
import random


import match as match

import parametres as p



class Robot():
    
    def __init__(self,team,numero,match,grSim,com):
        self.id = numero
        self.x = numero*500
        self.y = 0
        self.orientation = 0
        self.match = match
        self.positionc = 0
        self.team = team
        self.poste = [0,'CHASER']
        self.distance_balle = 10**6
        self.goto = complex(numero*500,0)
        self.status = 'INI'
        self.origine_passe = complex(0,0)
        self.interception = complex(0,0)
        self.grSim = grSim
        self.com = com
        self.orientation_but = False
    
    def __str__(self):
        if self.team == 'Y':
            equipe = 'jaune'
        else :
            equipe = 'bleu'
        return (f"Robot n°{self.id} {equipe}")
    
    def __repr__(self):
        return f"Robot('{self.team}',{self.id},{self.match})"
    
    def __eq__(self, o):
        return (self.team==o.team)&(self.id==o.id)
   
    def match(self,match):
        self.match = match
    
    #Permet de récuperer le coéquipier du robot
    def teammate(self):
        if self.team == self.match.blue.nom:
            team = self.match.blue
        else:
            team = self.match.yellow
        for robot in team.joueurs:
            if robot != self:
                return robot
    def myTeam(self):
        if self.team == self.match.blue.nom:
            team = self.match.blue
        else:
            team = self.match.yellow
            
        return team
    #Permet de récuperer les adversaires du robot
    def opponents(self):
        if self.team != self.match.blue.nom:
            team = self.match.blue
        else:
            team = self.match.yellow
        return team.joueurs
                
    #Attribution du poste avec une pile de 2 (cela permet de detecter un changement de poste)
    def defPoste(self,pos):
        
        self.poste.append(pos)
        if len(self.poste) > 2:
            self.poste.pop(0)
    
    #Actualise les position x,y,theta du robot
    def position(self,botInfo):     
        self.x,self.y,self.orientation = botInfo[6],botInfo[7],botInfo[8]
        self.positionc = complex(self.x,self.y)
        
    ##Ancienne version
    # def hasTheBall(self):
    #     pos_ball=self.match.balle.positionc
    #     pos_rob=self.positionc
    #     pos_vitual=pos_rob+complex(r_robot*np.cos(self.orientation),r_robot*np.sin(self.orientation))
    #     distance,phi=c.polar(pos_vitual-pos_ball)
       
    #     if distance <25:
    #         return True
    #     else :
    #         return False
    
    #Permet de savoir si le robot a la balle
    # def hasTheBall(self):
    #     pos_ball=self.match.balle.positionc
    #     pos_rob=self.positionc
        
    #     distance,phi=c.polar(pos_ball-pos_rob)
    #     delta=self.orientation-phi
       
    #     if (distance <141)&(abs(delta)<0.33):
    #         return True
    #     else :
            
    #         return False
    # from matplotlib import pyplot as plt
    # fig=plt.figure()
    # ax = fig.add_subplot(1, 1, 1)
    # ax.set_xlim(-300,300)
    # ax.set_ylim(-300,300)
    def hasTheBall(self):
        
        # global ax
        # ax.clear()
        pos_ball = self.match.balle.positionc
        pos_rob = self.positionc
        
        theta = self.orientation
        pos_dribbler = pos_rob + complex(p.r_robot*np.cos(theta),p.r_robot*np.sin(theta))
        vect_ball = pos_ball-pos_dribbler
        theta = self.orientation
        #projection de vect bal dans le repere du robot
        x1 = np.cos(theta) * vect_ball.real + np.sin(theta) * vect_ball.imag
        y1 = -np.sin(theta) * vect_ball.real + np.cos(theta) * vect_ball.imag
        # print(x1,y1)
        if (x1 < 10) and (abs(y1) < 45) and (x1 > -50):
            # print(x1,y1)
            return True
        else :
            return False
        
    
   
    #Calcul de la distance entre le robot et un point du terrain
    def distanceToXY(self,pos_target):
        # pos_vitual=pos_rob+complex(r_robot*np.cos(self.orientation),r_robot*np.sin(self.orientation))
        distance,phi = c.polar(self.positionc-pos_target)
        # print(distance)
        return distance
    
    #Calcul du potentiel créé par le robot  
    def Potentiel(self):       
        self.potentiel = pt.Potentiel(10,[self.x,self.y],p.x_grid,p.y_grid,p.r_evit)
    
        
    #Calcule le champ tournant et repulsif produit par ce robot en fct de la position du robot sur la trajectoire
    def champ_perso(self,x_depart,x_arrivee,a,b):       
        self.Potentiel()
        Ex1,Ey1 = pt.Gradient(self.potentiel)
        Ex1,Ey1 = pt.Grad(Ex1,Ey1,self.x,self.y,x_depart,x_arrivee,a,b)
        
        potentiel_repulsion = pt.Potentiel(+10,[self.x,self.y],p.x_grid,p.y_grid,p.r_evit/1.5)
        Ex10,Ey10 = pt.Gradient(potentiel_repulsion)
        
        self.Ex , self.Ey = p.k * Ex10 + Ex1 , p.k * Ey10 + Ey1
    
    #Calcule le champ provoquer par les autres joueurs    
    def champ_autre(self,x_depart,x_arrivee,y_arrivee,a,b):
        Ex_autre = 0
        Ey_autre = 0
        for robot in self.match.joueurs:
            if robot != self:
                #On ne prend pas en compte le champ créé par les robots derrière nous
                if (x_arrivee-self.x)*(robot.x-self.x) + (y_arrivee-self.y)*(robot.y-self.y) > 0:
                    robot.champ_perso(x_depart, x_arrivee, a, b)
                    Ex,Ey = robot.Ex,robot.Ey
                    Ex_autre += Ex
                    Ey_autre += Ey
        self.Ex_autre,self.Ey_autre=Ex_autre,Ey_autre
    
    
    #Commande vers une position avec orientation vers un point donné (xo,yo)
    def commande_position(self,x,y,xo,yo,balle=False,spin=False):
        self.goto = complex(x,y)
        position_arrivee = match.Balle(x,y)     
        
        vecteur = position_arrivee.positionc-self.positionc
        distance,phi = c.polar(vecteur)
        
        o = c.polar(complex(xo,yo)-self.positionc)[1]
        delta = (o-self.orientation)
        delta += np.pi
        delta = delta%(2*np.pi)
        delta -= np.pi
        
        vitesse_angulaire = min(p.sat_vitesse_angulaire, abs(delta) * p.K_angulaire) * np.sign(delta)
        
        distance_autres_joueurs = []
        distance_autres_joueurs_balle = []
        for robot in self.match.joueurs:
            if robot != self:
                distance_autres_joueurs.append(self.distanceToXY(robot.positionc))
                distance_autres_joueurs_balle.append(robot.distanceToXY(position_arrivee.positionc))
        
        
        
        #si l'objectif est proche mais un autre robot aussi alors on fait abstraction des champs repulsifs
        if (min(distance_autres_joueurs)>distance) & (min(distance_autres_joueurs_balle)<300) & (distance<300) :
            if self.poste[-1] != 'GOAL':
                self.commande_robot(p.K_proche*np.cos(phi-self.orientation), p.K_proche*np.sin(phi-self.orientation), vitesse_angulaire,spinner=(balle or spin))
            else:
                self.commande_robot(0.6*p.K_max*np.cos(phi-self.orientation), 0.6*p.K_max*np.sin(phi-self.orientation), vitesse_angulaire,spinner=(balle or spin))
                
            return 'EN COURS'
        
        elif (abs(distance) > 35) or (abs(delta) > 0.05):
            #champ créé par l'objectif
            Exb,Eyb = pt.Gradient(position_arrivee.potentiel)
            
            #Calcul des positions x,y discrètes sur la grille
            xd = int((self.x + p.longueur) / (2*p.longueur) * p.nbPoints)
            yd = int((self.y + p.largeur) / (2*p.largeur) * p.nbPoints)
            # print(Ex[yd,xd],xd,self.x,self.y)
            
            
            #test si les robots sont sur le terrain
            if xd < p.nbPoints and yd < p.nbPoints:
                a,b = np.polyfit([x,self.x],[y,self.y],1)
                
                self.champ_autre(self.x,x,y,a,b)
                Ex,Ey = Exb + p.k * self.Ex_autre, Eyb + p.k * self.Ey_autre
                #champ surfaces interdites
                Ex += 2*p.surface_x
                Ey += 2*p.surface_y
                
                Ex,Ey = pt.norme(Ex,Ey)
                
                # #Affichage du champ
                # fig = plt.figure()
                # ax1 = fig.add_subplot(1, 1, 1)
                # ax1.quiver(x_grid,y_grid,Ex,Ey,width=0.0008)
                
                #Saturation
                vitesse_tangente = min(p.saturation_vitesse_tangente,p.K_max*(np.sin(self.orientation)*Ey[yd,xd]+np.cos(self.orientation)*Ex[yd,xd]))
                vitesse_normale = min(p.saturation_vitesse_normale,p.K_max*(-np.sin(self.orientation)*Ex[yd,xd]+np.cos(self.orientation)*Ey[yd,xd]))
                
                if self.poste[-1] == 'GOAL':
                    seuil = p.seuil_distance
                if distance < p.seuil_distance:
                    vitesse_tangente = vitesse_tangente * distance / p.seuil_distance
                    vitesse_normale = vitesse_normale * distance / p.seuil_distance
                    spin = (balle or spin)
                
                
                self.commande_robot(vitesse_tangente, vitesse_normale, vitesse_angulaire,spinner=spin)
                
            
                return 'EN COURS'
            
            
            
        #Objectif atteint    
        else :    
            self.commande_robot(0, 0, 0,spinner=True)
        
            return 'DONE'
    
    #Commande pour récuperer la balle    
    def commande_balle(self): 
        ballon = self.match.balle
        self.commande_position(ballon.x,ballon.y,ballon.x,ballon.y,balle=True) 
        
        
        
    #Commande pour s'orienter avec la balle comme centre instanné de rotation
    def orientation_with_ball(self,xo,yo):
        o_bot = self.orientation
        o = c.polar(complex(xo,yo)-self.positionc)[1]
        delta = (o-o_bot)
        delta += np.pi
        delta = delta%(2*np.pi)
        delta -= np.pi
        vitesse_angulaire = min(p.sat_orientation_balle,abs(delta)*p.K_orientation_balle)*np.sign(delta)
        if (abs(delta) > .05):
            
            self.commande_robot(0, -vitesse_angulaire/9.25, vitesse_angulaire,spinner=True)
            return 'EN COURS'

        else :
            self.commande_robot(0, 0, 0,spinner=True)
            return 'DONE'
        
        
    
    #Commmande pour s'orienter face au but et tirer    
    def Tir(self):
        if self.team == self.match.blue.nom:
            xbut,ybut = self.match.blue.but_adversaire
        else :
            xbut,ybut = self.match.yellow.but_adversaire
        posbut = complex(xbut,ybut)
        
        posbot = self.positionc
        vecteur = posbut-posbot
        distance,phi = c.polar(vecteur)
        # print(phi)
        self.orientation_with_ball(xbut,ybut)
        # self.commande_position(self.x,self.y,xbut,ybut)

        delta = phi-self.orientation
        
        if (abs(delta) > .5) & (not self.myTeam().openGoal(self)):
            self.defPoste('ATT')
            return 'DONE'
        
        if (abs(delta) < .05):
            self.commande_robot(0, 0, 0,tir=5)
            print('Boom')
            return 'DONE'
        else :
            return 'EN COURS'
     
        
    #Commande pour orienter les deux joueurs face à face avant une passe puis passe
    def Passe(self):
        if not self.hasTheBall():
            print('fuck')
            self.defPoste('WAIT')
            return 'EN COURS'
        else:
            pass
        #calcul vecteur directeur entre le receveur et le passeur
        passeur = self.positionc
        mate = self.teammate()
        mate.origine_passe = passeur
        receveur = complex(mate.goto.real,mate.goto.imag)+p.r_spinner*np.exp(complex(0,mate.orientation))
        direction = receveur-passeur
        distance,phi = c.polar(direction)
        
        angle = (phi+np.pi)%(2*np.pi)
        if angle > np.pi:
            angle -= 2*np.pi
        
        
         #calcul de l'angle passeur-receveur-but
        but_adv = mate.myTeam().but_adversaire
        _,angle_but = c.polar(complex(but_adv[0],but_adv[1])-receveur)
        but_adv_c = complex(but_adv[0],but_adv[1])
        x = but_adv_c-receveur
        y = passeur-receveur
        dot = x.real * y.real + x.imag * y.imag
        norm = abs(x) * abs(y)
        theta = np.arccos(dot/norm)
        # print(theta)
        
        if theta < np.pi/3:
            status = self.orientation_with_ball(mate.goto.real+p.r_spinner*np.cos(angle_but),mate.goto.imag+p.r_spinner*np.sin(angle_but))
            # mate.commande_position(mate.goto.real, mate.goto.imag,but_adv[0],but_adv[1] )
            delta1 = angle_but-mate.orientation
            mate.orientation_but = True
            if delta1 < -np.pi:
                delta1 += 2*np.pi
                # print(delta1)
            elif delta1 > np.pi:
                delta1 -= 2*np.pi
                # print(delta1)
                
            # print(delta1)
        else:    
            #orientation du receveur vers le passeur
            status = self.orientation_with_ball(mate.goto.real+p.r_spinner*np.cos(mate.orientation),mate.goto.imag+p.r_spinner*np.sin(mate.orientation))
            # mate.commande_position(mate.goto.real, mate.goto.imag,self.x,self.y )
            delta1 = angle-mate.orientation
            if delta1 < -np.pi:
                delta1 += 2*np.pi
                # print(delta1)
            elif delta1 > np.pi:
                delta1 -= 2*np.pi
                # print(delta1)
            mate.orientation_but = False
        
        
        # delta=phi-self.orientation
        
        # print(status,angle,mate.orientation, delta1)
        
        #Alignement réussi + mate pas trop loin de son ojectif IDEE arrêt lors du changement de signe
        if (status == 'DONE') & (abs(delta1) < 1.2):
            #ajustement de la puissance en fonction de la position du receveur
            if mate.distanceToXY(mate.goto) > 150:
                puissance = self.puissanceKicker(distance) + 0.3
            else :
                # puissance=distance/400
                puissance = self.puissanceKicker(distance) + 1
            #passe
            self.commande_robot(0, 0, 0,tir=puissance)
            mate.origine_passe = self.match.balle.positionc
            print('Passe')
            self.myTeam().passe = True
            self.defPoste('DEMARQUE')
            self.teammate().defPoste('RECEVEUR')
            return 'DONE'
        
        else :
            # print(delta1,mate.orientation_but)
            # print('load passe',delta,delta1)
            return 'EN COURS'
    
#Commande pour orienter les deux joueurs face à face avant une passe puis passe
    def PasseLob(self):
        if not self.hasTheBall():
            print('fuck')
            self.defPoste('WAIT')
            return 'EN COURS'
        else:
            pass
        
        #calcul vecteur directeur entre le receveur et le passeur
        passeur = self.positionc
        mate = self.teammate()
        mate.origine_passe = passeur
        receveur = complex(mate.goto.real,mate.goto.imag)+p.r_spinner*np.exp(complex(0,mate.orientation))
        direction = receveur-passeur
        distance,phi = c.polar(direction)
        
        #orientation du passeur vers le receveur
        self.orientation_with_ball(mate.goto.real+p.r_spinner*np.cos(mate.orientation),mate.goto.imag+p.r_spinner*np.sin(mate.orientation))
        
        angle = (phi+np.pi)%(2*np.pi)
        if angle > np.pi:
            angle -= 2*np.pi
        mate.orientation_but = False
        #orientation du receveur vers le passeur
        mate.commande_position(mate.goto.real, mate.goto.imag,self.x,self.y )
        
        delta = phi-self.orientation
        delta1 = angle-mate.orientation
        
        
        
        #Alignement réussi IDEE arrêt lors du changement de signe
        if (abs(delta) < .05) & (abs(delta1) < 0.5) & (mate.distanceToXY(mate.goto) < 150):
                       
            puissance = self.puissanceKicker(distance)
            
            #passe
            pr = psl.packetCommandBot(self.team=='Y', 
                         id=self.id, 
                         veltangent=0, 
                         velnormal=0, 
                         velangular=0,
                         spinner=False,
                         kickspeedx=puissance,
                         kickspeedz=2.5)
            self.grSim.send(pr)
            mate.origine_passe = self.match.balle.positionc
            print('Passe lobée')
            self.myTeam().lob = True
            self.defPoste('DEMARQUE')
            self.teammate().defPoste('RECEVEUR')
            return 'DONE'
        
        else :
            # print('load passe',delta,delta1)
            return 'EN COURS'        
            
    #Commande pour que le receveur se dirige vers le point d'arrivée de la balle
    def reception(self):  
        # print('rec')
        
        #calcul vecteur directeur entre le receveur et le passeur
        mate = self.teammate()
        receveur = self.positionc + p.r_spinner * np.exp(complex(0,self.orientation))
        passeur = self.origine_passe
        direction = receveur-passeur
        distance,phi = c.polar(direction)
        
        ballon = self.match.balle
        vitesse_balle = ballon.vitesse()
        #vecteur directeur de la balle
        vect_balle = ballon.trajectoire()
        d,o_balle = c.polar(vect_balle)
        
        #Angle entre le vecteur entre les deux robots et le vecteur de la balle
        omega = o_balle-phi
        # print(phi,o_balle)
        
        if self.orientation_but:
            #calcul de l'angle passeur-receveur-but
            but_adv = self.myTeam().but_adversaire
        
        #test si la balle est derriere le receveur
        vecteur1 = ballon.positionc-receveur
        vecteur2 = -direction
        dot = vecteur1.real * vecteur2.real + vecteur1.imag * vecteur2.imag
        # print(dot)
        
        
        #passe ratée
        if (self.teammate().poste[-1] not in ['PASSEUR','LOB']):
            passeur_no_balle = mate.distanceToXY(ballon.positionc) > 300
            #Si on observe un trop gros écart angulaire alors la passe a rebondit ou elle est ratée
            if ((abs(omega) > 1.0) & (vitesse_balle > 300) & passeur_no_balle) or ((vitesse_balle < 100) & passeur_no_balle) or (dot < 0):
                self.defPoste('WAIT')
                self.myTeam().passe = False
                self.myTeam().lob = False
                
                
            
            else : 
                #calcul de l'ajustement du receveur
                dU = complex(-direction.imag,direction.real) #rotation a 90° du vecteur reliant les robots
                dU = 1*np.sin(omega) * dU #projection de la position où la balle devrait arriver
                # print(theta,dU)
                
                #ajustement + orientation du receveur en fonction de theta
                if self.orientation_but:
                    #orientation du receveur vers le but
                    self.commande_position(self.x+dU.real, self.y+dU.imag,but_adv[0],but_adv[1],spin=True)
                    
                else:    
                    #orientation du receveur vers le passeur
                    self.commande_position(self.x+dU.real, self.y+dU.imag,mate.x,mate.y,spin=True)
        
        
        #passe pas encore effectuée       
        else :
            #asservissement + orientation du receveur en fonction de theta         
            if self.orientation_but:
                self.commande_position(self.goto.real, self.goto.imag, but_adv[0], but_adv[1])
               
            else:    
                #orientation du receveur vers le passeur
                self.commande_position(self.goto.real, self.goto.imag, mate.x, mate.y)  
            
       
    
            
    def puissanceKicker(self,distance):
        if distance > 140:
            return 0.05249286 * (distance-140) ** 0.5
        else :
            return 0
    
        
    def PasseEnProfondeur(self):
        passeur = self.positionc
        mate = self.teammate()
        receveur = mate.goto
        direction = receveur-passeur
        distance,phi = c.polar(direction)
        
        #orientation du passeur vers l'endroit de la passe
        # self.orientation_with_ball(mate.x,mate.y)
        self.commande_position(self.x,self.y,mate.x,mate.y)
        
        angle = (phi+np.pi)%(2*np.pi)
        if angle > np.pi:
            angle -= 2*np.pi
        
        
        delta = phi-self.orientation
        
        
        #Alignement réussi
        if abs(delta) < .05:
            
            self.commande_robot(0, 0, 0,tir=self.puissanceKicker(distance))
            print('Passe')
            self.defPoste('DEMARQUE')
            
            return 'DONE'
        
        else :
            return 'EN COURS'
     
        
    #Commande pour se placer en tant que gardien entre le balle et le but tout en respectant la surface    
    def goal(self,objectif=None):
        team = self.myTeam()
        alpha = 0.8   
        if team.side == 'L':
            x = -1350
            xg = -900
            y = 0
            flip = True
           
        else :
            x = 1350
            xg = 900
            y = 0
            flip = False
        balle = self.match.balle
        
        if not self.match.engagement:
            #test tir ennemi
            vect_balle = balle.trajectoire()
        else:
            vect_balle = complex(0,0)
        
        if flip:
            y_prediction = balle.y - vect_balle.imag * (x - balle.x)
            # print('flip going to the left')
        else:
            y_prediction = balle.y + vect_balle.imag * (x - balle.x)
        
        spin = False
        if abs(y_prediction) < 175 and balle.vitesse() > 800:
            # print(y_prediction,balle.vitesse())
            alpha = 1
            y = y_prediction
            spin = True
                
        if type(objectif) == complex:
            balleX = objectif.real
            balleY = objectif.imag
            balleC = objectif
        else:
            balle = self.match.balle
            balleX = balle.x
            balleY = balle.y
            balleC = balle.positionc
        
        a,b = np.polyfit([balleX,x],[balleY,y],1)
            
        #interception
        vect = balleC-self.positionc
        vect_but = balleC - complex(x,y)
        
        scalaire = (vect.real * vect_but.real + vect.imag * vect_but.imag)
        dist = vect - scalaire * vect_but / (abs(vect_but)**2)
        self.interception = dist
        # print(a,vect_but,vect_but/abs(vect_but),dist)
        
        
        yg=a * xg + b
        if abs(yg) > 440:
            yg= np.sign(yg) * 480
            xg= (yg - b) / a
        # distance,phi=c.polar(balle.positionc-complex(x,y))
        goal_pos = complex(xg,yg)
        if (abs(dist) < p.r_robot) or (scalaire < 0):
        #position but
 
            self.goto = goal_pos
            
            
        else:
            interception_pos = self.positionc+dist
            
            #mix entre position gardien et interception
            
            placement = alpha * interception_pos + (1 - alpha) * goal_pos
            
            #check surface
            if (abs(placement.real) > 1000 - p.r_robot) and (abs(placement.imag) < 350 + p.r_robot):
                self.goto = goal_pos
            else:
                self.goto = placement
                
        if type(objectif) == complex:
            pass
        else:
            self.commande_position(self.goto.real, self.goto.imag, balleX,balleY,spin=spin)
    
    def goDef(self,danger):
        defense = random.random()
        # print(defense)
        if self.match.strategie == 'NORMAL':
            if danger:
                if defense < 0.15:
                    self.defPoste('DEF1')
                elif defense < 0.5:
                    self.defPoste('DEF2')
                elif defense < 0.85:
                    self.defPoste('DEF3')
                else:
                    self.defPoste('DEF4')
                    
            else:
                if defense < 0.2:
                    self.defPoste('DEF1')
                elif defense < 0.5:
                    self.defPoste('DEF2')
                elif defense < 0.6:
                    self.defPoste('DEF3')
                else:
                    self.defPoste('DEF4')
        
        elif self.match.strategie == 'DEFENSIF':
            if danger:
                if defense < 0.02:
                    self.defPoste('DEF1')
                elif defense < 0.4:
                    self.defPoste('DEF2')
                elif defense < 0.98:
                    self.defPoste('DEF3')
                else:
                    self.defPoste('DEF4')
                    
            else:
                if defense < 0.1:
                    self.defPoste('DEF1')
                elif defense < 0.5:
                    self.defPoste('DEF2')
                elif defense < 0.9:
                    self.defPoste('DEF3')
                else:
                    self.defPoste('DEF4')
                    
        elif self.match.strategie == 'OFFENSIF':      
            if danger:
                if defense < 0.15:
                    self.defPoste('DEF1')
                elif defense < 0.5:
                    self.defPoste('DEF2')
                elif defense < 0.85:
                    self.defPoste('DEF3')
                else:
                    self.defPoste('DEF4')
                    
            else:
                if defense < 0.4:
                    self.defPoste('DEF1')
                elif defense < 0.5:
                    self.defPoste('DEF2')
                elif defense < 0.55:
                    self.defPoste('DEF3')
                else:
                    self.defPoste('DEF4')
            
    
    def def1(self,balle):
        adversaires = self.opponents()
        adv1 = adversaires[0].positionc
        adv2 = adversaires[1].positionc
        placement = (adv1 + adv2) / 2
        self.goto = placement
        distance1 = adversaires[0].distanceToXY(balle.positionc)
        distance2 = adversaires[1].distanceToXY(balle.positionc)
        if distance1 < distance2:
            orientation = adv1
        else:
            orientation = adv2
        self.commande_position(self.goto.real,self.goto.imag,orientation.real,orientation.imag)
        self.defPoste('DEF1')
        
    def def2(self,but,balle):
        
        #determination de l'adversaire qui a la balle
        adversaires = self.opponents()
        mate = self.teammate()
        distance1 = adversaires[0].distanceToXY(balle.positionc)
        distance2 = adversaires[1].distanceToXY(balle.positionc)
        #le def s'occupe de l'autre
        if distance1 < distance2:
            adv = adversaires[1]
        else:
            adv = adversaires[0]
            
        pos_adv = adv.positionc
        
        #placement entre le but et l'adversaire
        placement = (pos_adv + complex(but[0],but[1])) / 2
        
        #check surface
        if (abs(placement.real) > 1000 - p.r_robot) and (abs(placement.imag) < 350 + p.r_robot):
            self.goal(pos_adv)
            placement = self.goto
            
        
        #calcul changement de poste entre le goal et le def par exemple lors d'une passe
        a,b = np.polyfit([balle.x,but[0]],[balle.y,but[1]],1)
        d1 = self.distance_droite(a, b)
        d2 = mate.distance_droite(a, b)
        v1 = balle.positionc-complex(but[0],but[1])
        v2 = balle.positionc-self.positionc
        dot = v1.real*v2.real+v1.imag*v2.imag
        # print(dot)
        if (d1 < d2) & (dot > 0) & (not(self.match.engagement)):
            print('chgt')
            self.defPoste('GOAL')
            self.teammate().goto = placement
            self.teammate().defPoste('DEF2')
        else:
            self.goto = placement
            self.commande_position(self.goto.real,self.goto.imag,pos_adv.real,pos_adv.imag)
            self.defPoste('DEF2')
            # print('no')
     
            
    def def3(self,but,balle):
        
        #determination de l'adversaire qui a la balle
        adversaires = self.opponents()
        distance1 = adversaires[0].distanceToXY(balle.positionc)
        distance2 = adversaires[1].distanceToXY(balle.positionc)
        #le def s'occupe de l'autre
        if distance1 < distance2:
            adv = adversaires[1]
        else:
            adv = adversaires[0]
        pos_adv = adv.positionc
        
        
        self.goal(pos_adv)
        placement = self.goto
            
        
        #calcul changement de poste entre le goal et le def par exemple lors d'une passe
        a,b = np.polyfit([balle.x,but[0]],[balle.y,but[1]],1)
        d1 = self.distance_droite(a, b)
        d2 = self.teammate().distance_droite(a, b)
        v1 = balle.positionc - complex(but[0],but[1])
        v2 = balle.positionc - self.positionc
        dot = v1.real * v2.real + v1.imag * v2.imag
        if (d1 < d2) & (dot > 0) & (not(self.match.engagement)):
            print('chgt')
            self.defPoste('GOAL')
            self.teammate().goto = placement
            self.teammate().defPoste('DEF3')
        else:
            self.goto = placement
            self.commande_position(self.goto.real,self.goto.imag,pos_adv.real,pos_adv.imag)
            self.defPoste('DEF3')
    
        
    def def4(self,balle):
        adversaires = self.opponents()
        adv1 = adversaires[0].positionc
        adv2 = adversaires[1].positionc
        d1 = adversaires[0].distanceToXY(balle.positionc)
        d2 = adversaires[1].distanceToXY(balle.positionc)
        
        if d2<d1:
            adv1,adv2 = adv2,adv1
        interception = 0.75 * adv2 + 0.25 * adv1
        
        dU=250 * (adv1 - adv2) / abs((adv1 - adv2)) * np.exp(complex(0,np.pi/2))
        
        but = complex(self.myTeam().but[0],self.myTeam().but[1])
        placement1 = interception + dU
        placement2 = interception - dU
        
        if adversaires[0].hasTheBall() or adversaires[1].hasTheBall():
            spin = False
            if abs(placement1 - but) < abs(placement2 - but):
                self.goto = placement1
            else:
                self.goto = placement2
        else:
            self.goto = interception
            spin = True
        
        self.commande_position(self.goto.real,self.goto.imag,balle.x,balle.y,spin=spin)
        self.defPoste('DEF4')
        
        
    #Fonction pour créer les données du terrain (la position des 4 robots) pour alimentier l'ia
    def create_game(self):
        changement_cote = False
        if ((self.team == self.match.blue.nom) & (self.match.blue.side == 'R')) or( (self.team == self.match.yellow.nom) & (self.match.yellow.side == 'R')):
            changement_cote = True
            # print('chgt cote')
        
        if not changement_cote:
            xb,yb = self.match.xy_to_position(self.x, self.y, self.match.n, self.match.m)
            xm,ym = self.match.xy_to_position(self.teammate().x, self.teammate().y, self.match.n, self.match.m)
            for robot in self.opponents():
                a,b = np.polyfit([self.x,robot.x],[self.y,robot.y],1)
                if robot.distance_droite(a,b) < p.r_robot:
                    xdg,ydg = self.match.xy_to_position(robot.x, robot.y, self.match.n, self.match.m)
                    xg,yg = robot.x,robot.y
                    xd,yd = self.match.xy_to_position(robot.teammate().x, robot.teammate().y, self.match.n, self.match.m)
        else:
            xb,yb = self.match.xy_to_position(-self.x, self.y, self.match.n, self.match.m)
            xm,ym = self.match.xy_to_position(-self.teammate().x, self.teammate().y, self.match.n, self.match.m)
            for robot in self.opponents():
                a,b = np.polyfit([self.x,robot.x],[self.y,robot.y],1)
                if robot.distance_droite(a,b) < p.r_robot:
                    xdg,ydg = self.match.xy_to_position(-robot.x, robot.y, self.match.n, self.match.m)
                    xg,yg = -robot.x,robot.y
                    xd,yd = self.match.xy_to_position(-robot.teammate().x, robot.teammate().y, self.match.n, self.match.m)
         
        
       
        return [(xb,yb),(xdg,ydg),(xg,yg),(xd,yd),(xm,ym)]
        
    def distance_droite(self,a,b): #calcul la distance d'un robot par rapport a une droite tel que y=ax+b  
        distance = abs(a * self.x - self.y + b) / np.sqrt( a**2+1)
        return distance
    
    def commande_com(self,ID,Vtan,Vnorm,Vrot,spinner=False,tir=0):
        chaine_commande = "D"+str(ID) + ","
        liste_vitesses = [Vtan,Vnorm,Vrot]
        spin = 0
        if spinner:
            spin = 1
            
        for vitesse in liste_vitesses :
            vitesse = int(vitesse * 999) #Régler ici si la vitesse max n'est pas 1
            
            if abs(vitesse) < 20 : #Zone morte manette
                vitesse = 0
            
            longueur_vitesse = len(str(abs(vitesse)))
            if vitesse >= 0:
                signe = "0" #Symbole pour +
            else :
                signe = "1" #Symbole pour -
                
            if longueur_vitesse > 3:
                chaine_commande += signe + "999,"
            else :
                chaine_commande += signe + "0"*(3-longueur_vitesse)+str(abs(vitesse))+","
        chaine_commande += str(spin) + "," + str(tir)[0] #a changer
        
        # print (ser.portstr)       # check which port was really used
        print(chaine_commande)
        for charactere in chaine_commande:
                self.com.write(str.encode(charactere))
        # time.sleep(0.05)
        return chaine_commande
    
    def commande_robot(self,Vtang,Vnorm,Vang,spinner=True,tir=0):
        if self.grSim != None:
            
            p = psl.packetCommandBot(self.team=='Y', 
                         id=self.id, 
                         veltangent=Vtang, 
                         velnormal=Vnorm, 
                         velangular=Vang,
                         spinner=spinner,
                         kickspeedx=tir)
            self.grSim.send(p)
        if self.com:
            # start=time.time()
            self.commande_com(self.id,Vtang,Vnorm,Vang,spinner,tir)
            # time.sleep(0.05)
            # print(time.time()-start)