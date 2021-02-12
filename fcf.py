# -*- coding: utf-8 -*-
"""
Created on Thu Dec 31 10:05:17 2020

@author: mathi
"""
import time
import math as m
import cmath
from psl_package import paris_saclay_league as psl


class Robot():
    """Classe représentant un robot joueur sur le terrain"""
    def __init__(self,equipe,ID,position,angle):
        self.ID = ID
        
        self.equipe = equipe
        self.equipe.robots.append(self)
        
        self.position = position #La position est un complexe représentant ses coordonnées
        self.angle = angle
        
        self.possession = True
        
    def __str__(self):
        return("ID:{}, équipe : {}".format(self.ID,self.equipe.couleur))
    
    def __eq__(self, other):
        return(self.equipe==other.equipe and self.ID == other.ID)
        
    
    def commande(self,veltangent,velnormal,velangular,spinner,kickspeedx):
        p = psl.packetCommandBot(self.equipe.couleur == "jaune", 
                         self.ID, 
                         veltangent, 
                         velnormal, 
                         velangular,
                         spinner,
                         kickspeedx)
        grSim.send(p)

         
class Balle():
    """Classe représentant une balle sur le terrain"""    
    def __init__(self,position):
        self.position = position


class Equipe():
    """Classe représentant une équipe de robots, leur couleur et leur coté de jeu"""
    def __init__(self,robots,couleur,cote): #robots est la liste des robots de l'equipe, couleur est bleu ou joaune et coté est droite ou gauche
        self.robots = robots
        self.couleur = couleur #jaune ou bleu
        self.cote = cote #droite ou gauche
        
    def __eq__(self, other):
        return(self.couleur == other.couleur)
    
    def __str__(self):
        return("couleur:{} '=, robots:{}".format(self.couleur,self.robots))

  
class Terrain():
    def __init__(self,robots,balle):
        self.robots = robots #Contient une liste de tous les robots presents sur le terrain
        self.balle = balle
        self.vision = psl.SSLVisionClient(ip='224.5.23.2', port=10020)
        self.vision.connect()
    
    def debut_match(self):
        self.lecture_position()
    
    def lecture_position(self): #Met à jour les positions et les possessions de balle.
    
        ### Position des robots/de la balle
        while not self.vision.isDataReady():
            continue
        balls, blueBots, yellowBots = self.vision.getLocations()
        if len(blueBots) >= 2 and len(yellowBots) >= 2 and len(balls) >= 1:
            for R in self.robots : #On parcourt les robots pour assigner leur position en fonction des données reçues de la vision
                
                for data in blueBots:
                    if R.equipe.couleur == "bleu" and R.ID == data[5]:
                        R.position = complex(data[6],data[7])
                        R.angle = data[8]
                        
                for data in yellowBots :
                    if R.equipe.couleur == "jaune" and R.ID == data[5]:
                        R.position = complex(data[6],data[7])
                        R.angle = data[8]
        
        self.balle.position = complex(balls[0][6], balls[0][7]) # On choisit arbitrairement la première valeur reçue pour la position de la balle
        
        ### Possession de balle
        
        for R in self.robots :
            R.possession = (abs(R.position-self.balle.position)<120) #On regle ici la distance à laquelle la balle est considérée comme en possession
      

class Coach():
    def __init__(self,equipe_jouee,equipe_adverse,terrain):
        self.equipe_jouee = equipe_jouee
        self.equipe_adverse = equipe_adverse
        self.terrain = terrain
        for robot in self.equipe_jouee.robots :
            if robot.ID == 0 :
                self.r0 = robot
            if robot.ID == 1 :
                self.r1 = robot
                
                
    def goTo(self,robot_commande,xc,yc,xo,yo,vitesse_tir):
            saturation_vitesse_deplacement = 1
            saturation_vitesse_deplacement_si_proche = 0.2
            saturation_vitesse_angulaire = 3
            coef_prop_deplacement = 0.004
            
            vect_botp_consigne=complex(xc,yc)-robot_commande.position
            distance_botp_consigne=abs(vect_botp_consigne)
            
            #On nouhaite éviter les colisions. Pour cela, on doit comparer la position du robot traité avec celle des 3 autres
            coef_secu = 300 #A augmenter pour plus éviter les bots
                  
            for robot in self.terrain.robots :
                if robot != robot_commande :
                    vect_bota_botp = robot_commande.position - robot.position
                    distance_bota_botp = abs(vect_bota_botp)
                    if distance_bota_botp < min(450,distance_botp_consigne+70) :
                        norme_objectif = coef_secu/(distance_bota_botp*0.001)
                        modif_traj = vect_bota_botp/distance_bota_botp*norme_objectif
                        vect_botp_consigne += modif_traj
            
            
            
            #Si l'objectif est trop proche d'un robot (distance à régler ici), on diminue la vitesse max pour éviter les collisions
            dist_min = 50000
            
            for robot in self.terrain.robots :
                if robot != robot_commande :
                    vect_botb_objectif = complex(xc,yc) - robot.position
                    distance_botb_objectif = abs(vect_botb_objectif)
                    dist_min = min(dist_min, distance_botb_objectif)
            
                polarc = cmath.polar(vect_botp_consigne)
    
            if distance_botp_consigne < 250 and dist_min < 250: #Si on est proche de l'objectif et que l'objectif est proche du robot, on sature plus
                saturation_vitesse_deplacement = saturation_vitesse_deplacement_si_proche
            
            distance_botp_consigne_sat = min(polarc[0]*coef_prop_deplacement,saturation_vitesse_deplacement) #On applique une saturation
    
    
            velt = distance_botp_consigne_sat*m.cos(robot_commande.angle-polarc[1])
            veln = -distance_botp_consigne_sat*m.sin(robot_commande.angle-polarc[1])
            
            
            anglec=(cmath.polar(complex(xo,yo)-robot_commande.position))[1]
            ecart_angle=anglec - robot_commande.angle
        
            ecart_angle += m.pi
            ecart_angle=ecart_angle%(2*m.pi)
            ecart_angle -= m.pi
        
            
            #On règle ici le coefficient proportionnel pour la vitesse de rotation
            ecart_angle *= 4
            
            if abs(ecart_angle) > saturation_vitesse_angulaire:
                ecart_angle = abs(ecart_angle)/ecart_angle * saturation_vitesse_angulaire
            
            vela = ecart_angle
            
            robot_commande.commande(velt,veln,vela,1,vitesse_tir)


#Création des équipes replies de 2 robots
equipe_B, equipe_J = Equipe([],"bleu","droite"), Equipe([],"jaune","gauche")
RB0, RB1, RJ0, RJ1 = Robot(equipe_B,0,complex(0,0),0), Robot(equipe_B,1,complex(0,0),0), Robot(equipe_J,0,complex(0,0),0), Robot(equipe_J,1,complex(0,0),0)

#Creation de la balle, du terrain et du coach
balle = Balle(complex(0,0))
terrain = Terrain([RB0,RB1,RJ0,RJ1],balle)
coach = Coach(equipe_B, equipe_J, terrain)


grSim = psl.SSLgrSimClient('127.0.0.1', 20011)
grSim.connect()
time.sleep(1)
terrain.debut_match()



while True:
    time.sleep(0.01)
    terrain.lecture_position()
    coach.goTo(RB0,-1000,0,0,0,0)