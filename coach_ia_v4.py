#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar  5 10:48:18 2021

@author: psl
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Feb  7 20:19:12 2021

@author: psl
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb  2 20:16:00 2021

@author: psl
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec 21 17:14:26 2020

@author: psl
"""
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as patches
import matplotlib.lines as lines
import potentiel as pt


from psl_package import paris_saclay_league as psl
import cmath as c
import time
import numpy as np

import ia_v3 as ia




#Connexion au simulateur
vision = psl.SSLVisionClient(ip='224.5.23.2', port=10020)
vision.connect()
grSim = psl.SSLgrSimClient('127.0.0.1', 20011)
grSim.connect()


#%% Paramètres


#TERRAIN
longueur=1350
largeur=1000
xmin=-longueur
xmax=longueur
ymin=-largeur
ymax=largeur



##ROBOT
r_robot=115
K_max=0.8 #facteur prop vitesse
seuil_distance=250
sat_vitesse_angulaire=6
K_angulaire=3
sat_orientation_balle=3
K_orientation_balle=10

saturation_vitesse_tangente=0.9
saturation_vitesse_normale=0.85

##REPULSION
nbPoints=80
r_evit=120
k=4 #facteur de repulsion

R_non_evitement=360
K_proche=0.3

X = np.linspace(xmin,xmax,nbPoints)
Y = np.linspace(ymin,ymax,nbPoints)
x_grid,y_grid = np.meshgrid(X,Y)

#Création du champ répulsif des surfaces
surface_x=np.zeros((80,80))
surface_y=np.zeros((80,80))
for i in range(int(350/2700*80)):
    for j in range(int(650/2000*80),int(1350/2000*80)):
        surface_x[j,i]=1
        surface_x[j,79-i]=-1
        surface_y[j,i]=1*np.sign(j-40)*(int(350/2700*80)-i-1)
        surface_y[j,79-i]=1*np.sign(j-40)*(int(350/2700*80)-i-1)


#%%Commande

def getVision():
    global vision
    while not vision.isDataReady(): 
        continue
  
    return vision.getLocations()


class Robot():
    
    def __init__(self,team,numero,match):
        self.id=numero
        self.x=0
        self.y=0
        self.orientation=0
        self.match=match
        self.positionc=0
        self.team=team
        self.poste=[0,'CHASER']
        self.distance_balle=10**6
        self.goto=(0,0)
        self.status='INI'
    
    def __str__(self):
        if self.team=='Y':
            equipe='jaune'
        else :
            equipe='bleu'
        return (f"Robot n°{self.id} {equipe}")
    
    def __repr__(self):
        return f"Robot('{self.team}',{self.id},{self.match})"
    
    def __eq__(self, o):
        return (self.team==o.team)&(self.id==o.id)
   
    def match(self,match):
        self.match=match
    
    #Permet de récuperer le coéquipier du robot
    def teammate(self):
        if self.team==self.match.blue.nom:
            team=self.match.blue
        else:
            team=self.match.yellow
        for robot in team.joueurs:
            if robot!=self:
                return robot
    
    #Permet de récuperer les adversaires du robot
    def opponents(self):
        if self.team!=self.match.blue.nom:
            team=self.match.blue
        else:
            team=self.match.yellow
        return team.joueurs
                
    #Attribution du poste avec une pile de 2 (cela permet de detecter un changement de poste)
    def defPoste(self,pos):
        
        self.poste.append(pos)
        if len(self.poste)>2:
            self.poste.pop(0)
    
    #Actualise les position x,y,theta du robot
    def position(self,botInfo):     
        self.x,self.y,self.orientation=botInfo[6],botInfo[7],botInfo[8]
        self.positionc=complex(self.x,self.y)
        
    ##Ancienne version
    # def hasTheBall(self):
    #     pos_ball=self.match.balle.position
    #     pos_rob=self.positionc
    #     pos_vitual=pos_rob+complex(r_robot*np.cos(self.orientation),r_robot*np.sin(self.orientation))
    #     distance,phi=c.polar(pos_vitual-pos_ball)
       
    #     if distance <25:
    #         return True
    #     else :
    #         return False
    
    #Permet de savoir si le robot a la balle
    def hasTheBall(self):
        pos_ball=self.match.balle.position
        pos_rob=self.positionc
        
        distance,phi=c.polar(pos_ball-pos_rob)
        delta=self.orientation-phi
       
        if (distance <141)&(abs(delta)<0.33):
            return True
        else :
            
            return False
    
   
    #Calcul de la distance entre le robot et un point du terrain
    def distanceToXY(self,pos_target):
        # pos_vitual=pos_rob+complex(r_robot*np.cos(self.orientation),r_robot*np.sin(self.orientation))
        distance,phi=c.polar(self.positionc-pos_target)
        # print(distance)
        return distance
    
    #Calcul du potentiel créé par le robot  
    def Potentiel(self):       
        self.potentiel=pt.Potentiel(10,[self.x,self.y],x_grid,y_grid,r_evit)
    
        
    #Calcule le champ tournant et repulsif produit par ce robot en fct de la position du robot sur la trajectoire
    def champ_perso(self,x_depart,x_arrivee,a,b):       
        self.Potentiel()
        Ex1,Ey1=pt.Gradient(self.potentiel)
        Ex1,Ey1=pt.Grad(Ex1,Ey1,self.x,self.y,x_depart,x_arrivee,a,b)
        
        potentiel_repulsion=pt.Potentiel(+10,[self.x,self.y],x_grid,y_grid,r_evit/1.5)
        Ex10,Ey10=pt.Gradient(potentiel_repulsion)
        
        self.Ex,self.Ey=k*Ex10+Ex1,k*Ey10+Ey1
    
    #Calcule le champ provoquer par les autres joueurs    
    def champ_autre(self,x_depart,x_arrivee,y_arrivee,a,b):
        Ex_autre=0
        Ey_autre=0
        for robot in self.match.joueurs:
            if robot!=self:
                #On ne prend pas en compte le champ créé par les robots derrière nous
                if (x_arrivee-self.x)*(robot.x-self.x) + (y_arrivee-self.y)*(robot.y-self.y) > 0:
                    robot.champ_perso(x_depart, x_arrivee, a, b)
                    Ex,Ey=robot.Ex,robot.Ey
                    Ex_autre+=Ex
                    Ey_autre+=Ey
        self.Ex_autre,self.Ey_autre=Ex_autre,Ey_autre
    
    #Commande vers une position avec orientation vers un point donné (xo,yo)
    def commande_position(self,x,y,xo,yo,balle=False,spin=False): 
        position_arrivee=Balle(x,y)     
        
        vecteur=position_arrivee.position-self.positionc
        distance,phi=c.polar(vecteur)
        
        o=c.polar(complex(xo,yo)-self.positionc)[1]
        delta=(o-self.orientation)
        delta+=np.pi
        delta=delta%(2*np.pi)
        delta-=np.pi
        
        vitesse_angulaire=min(sat_vitesse_angulaire,abs(delta)*K_angulaire)*np.sign(delta)
        
        distance_autres_joueurs=[]
        distance_autres_joueurs_balle=[]
        for robot in self.match.joueurs:
            if robot!=self:
                distance_autres_joueurs.append(self.distanceToXY(robot.positionc))
                distance_autres_joueurs_balle.append(robot.distanceToXY(robot.match.balle.position))
        
        
        
        #si l'objectif est proche mais un autre robot aussi alors on fait abstraction des champs repulsifs
        if (min(distance_autres_joueurs)>distance) & (min(distance_autres_joueurs_balle)<300) & (distance<300) & balle:
            # print("oui")
            
            p = psl.packetCommandBot(self.team=='Y', 
                         id=self.id, 
                         veltangent=K_proche*np.cos(delta), 
                         velnormal=K_proche*np.sin(delta), 
                         velangular=vitesse_angulaire,
                         spinner=True)
            
            grSim.send(p)
        
            return 'EN COURS'
        
        elif (abs(distance)>35)|(abs(delta)>0.05):
            #champ créé par l'objectif
            Exb,Eyb=pt.Gradient(position_arrivee.potentiel)
            
            #Calcul des positions x,y discrètes sur la grille
            xd=int((self.x+longueur)/(2*longueur)*nbPoints)
            yd=int((self.y+largeur)/(2*largeur)*nbPoints)
            # print(Ex[yd,xd],xd,self.x,self.y)
            
            a,b=np.polyfit([x,self.x],[y,self.y],1)
            
            self.champ_autre(self.x,x,y,a,b)
            Ex,Ey=Exb+k*self.Ex_autre,Eyb+k*self.Ey_autre
            #champ surfaces interdites
            Ex+=2*surface_x
            Ey+=2*surface_y
            
            Ex,Ey=pt.norme(Ex,Ey)
            
            # #Affichage du champ
            # fig = plt.figure()
            # ax1 = fig.add_subplot(1, 1, 1)
            # ax1.quiver(x_grid,y_grid,Ex,Ey,width=0.0008)
            
            #Saturation
            vitesse_tangente=min(saturation_vitesse_tangente,K_max*(np.sin(self.orientation)*Ey[yd,xd]+np.cos(self.orientation)*Ex[yd,xd]))
            vitesse_normale=min(saturation_vitesse_normale,K_max*(-np.sin(self.orientation)*Ex[yd,xd]+np.cos(self.orientation)*Ey[yd,xd]))
            
            if distance<seuil_distance:
                vitesse_tangente=vitesse_tangente*distance/seuil_distance
                vitesse_normale=vitesse_normale*distance/seuil_distance
                spin=True
            
            
            p = psl.packetCommandBot(self.team=='Y', 
                              id=self.id, 
                              veltangent=vitesse_tangente, 
                              velnormal=vitesse_normale, 
                              velangular=vitesse_angulaire,
                              spinner=spin)
            grSim.send(p)
        
            return 'EN COURS'
            
            
            
        #Objectif atteint    
        else :    
            p = psl.packetCommandBot(self.team=='Y', 
                                 id=self.id, 
                                 veltangent=0., 
                                 velnormal=0, 
                                 velangular=0.,
                                 spinner=True)
            grSim.send(p)
        
            return 'DONE'
    
    #Commande pour récuperer la balle    
    def commande_balle(self): 
        ballon=self.match.balle
        self.commande_position(ballon.x,ballon.y,ballon.x,ballon.y,balle=True) 
        
        
        
    #Commande pour s'orienter avec la balle comme centre instanné de rotation
    def orientation_with_ball(self,xo,yo):
        o_bot=self.orientation
        o=c.polar(complex(xo,yo)-self.positionc)[1]
        delta=(o-o_bot)
        delta+=np.pi
        delta=delta%(2*np.pi)
        delta-=np.pi
        vitesse_angulaire=min(sat_orientation_balle,abs(delta)*K_orientation_balle)*np.sign(delta)
        if (abs(delta)>.05):
            p = psl.packetCommandBot(self.team=='Y', 
                         id=self.id, 
                         veltangent=0., 
                         velnormal=-vitesse_angulaire/9.25, 
                         spinner=True,
                         velangular=vitesse_angulaire)
        
        else :
            p = psl.packetCommandBot(self.team=='Y', 
                             id=self.id, 
                             veltangent=0., 
                             velnormal=0, 
                             velangular=0.,
                             spinner=True
                             ) 
       
        grSim.send(p)
        
    
    #Commmande pour s'orienter face au but et tirer    
    def Tir(self):
        if self.team==self.match.blue.nom:
            xbut,ybut=self.match.blue.but_adversaire
        else :
            xbut,ybut=self.match.yellow.but_adversaire
        posbut=complex(xbut,ybut)
        
        posbot=self.positionc
        vecteur=posbut-posbot
        distance,phi=c.polar(vecteur)
        # print(phi)
        self.orientation_with_ball(xbut,ybut)

        delta=phi-self.orientation
        
        if (abs(delta)<.05):
            p = psl.packetCommandBot(self.team=='Y', 
                             id=self.id, 
                             veltangent=0., 
                             velnormal=0, 
                             velangular=0.,
                             kickspeedx=10.) 
            grSim.send(p)
            print('Boom')
            return 'DONE'
        else :
            return 'EN COURS'
     
    #Commande pour orienter les deux joueurs face à face avant une passe puis passe
    def Passe(self):
        
        passeur=self.positionc
        mate=self.teammate()
        receveur=mate.positionc
        direction=receveur-passeur
        distance,phi=c.polar(direction)
        
        #orientation du passeur vers le receveur
        self.orientation_with_ball(mate.x,mate.y)
        
        angle=(phi+np.pi)%(2*np.pi)
        if angle>np.pi:
            angle-=2*np.pi
        
        #orientation du receveur vers le passeur
        mate.commande_position(mate.x, mate.y,self.x,self.y )
        
        delta=phi-self.orientation
        delta1=angle-mate.orientation
        
        
        
        #Alignement réussi IDEE arrêt lors du changement de signe
        if (abs(delta)<.05)&(abs(delta1)<0.05):
            
            p = psl.packetCommandBot(self.team=='Y', 
                         id=self.id, 
                         veltangent=0., 
                         velnormal=0, 
                         velangular=0.,
                         kickspeedx=distance/400)
            grSim.send(p)
            print('Passe')
            self.defPoste('DEMARQUE')
            self.teammate().defPoste('RECEVEUR')
            return 'DONE'
        
        else :
            return 'EN COURS'
            
            
    #Commande pour que le receveur se dirige vers le point d'arrivée de la balle
    def reception(self):  
        print('rec')
        mate=self.teammate()
        receveur=self.positionc
        passeur=mate.positionc
        direction=receveur-passeur
        distance,phi=c.polar(direction)
        
        
        # angle=(phi+np.pi)%(2*np.pi)
        # if angle>np.pi:
        #     angle-=np.pi
        
        #vecteur directeur de la balle
        vect_balle=self.match.balle.trajectoire()
        d,o_balle=c.polar(vect_balle)
        
        #Angle entre le vecteur entre les deux robots et le vecteur de la balle
        theta=phi-o_balle
        print(phi,o_balle)
        
        # print(theta,((self.match.balle.x10[-10]-self.match.balle.x10[-1])**2+(self.match.balle.y10[-10]-self.match.balle.y10[-1])**2))
        
        #Si on observe un trop gros écart angulaire alors la passe a rebondit ou elle est ratée
        if (abs(theta)>0.4)&(((self.match.balle.x10[-10]-self.match.balle.x10[-1])**2+(self.match.balle.y10[-10]-self.match.balle.y10[-1])**2)>40000):
            self.defPoste('WAIT')
        else :
            # self.defPoste('RECEVEUR')
            
            dU=complex(-direction.imag,direction.real) #rotation a 90° du vecteur reliant les robots
            dU=1*np.sin(theta)*dU #projection de la position où la balle devrait arriver
            print(theta,dU)
            if self.match.disp==2:
                
                goto,=ax.plot(self.x+dU.real, self.y+dU.imag,'mo')
                ax.draw_artist(goto)
            self.commande_position(self.x+dU.real, self.y+dU.imag,mate.x,mate.y )
        

    #Commande pour se placer en tant que gardien entre le balle et le but tout en respectant la surface    
    def goal(self):
        if self.match.blue.nom==self.team:
            team=self.match.blue
        else:
            team=self.match.yellow
            
        if team.side=='L':
            x=-1350
            xg=-900
            y=0
           
        else :
            x=1350
            xg=900
            y=0
            
        
        balle=self.match.balle
        
        a,b=np.polyfit([balle.x,x],[balle.y,y],1)
        yg=a*xg+b
        if abs(yg)>440:
            yg=np.sign(yg)*400
            xg=(yg-b)/a
        # distance,phi=c.polar(balle.position-complex(x,y))
        self.commande_position(xg, yg, balle.x,balle.y)
        
    #Fonction pour créer les données du terrain (la position des 4 robots) pour alimentier l'ia
    def create_game(self):
        changement_cote=False
        if ((self.team==self.match.blue.nom)&(self.match.blue.side=='R'))|((self.team==self.match.yellow.nom)&(self.match.yellow.side=='R')):
            changement_cote=True
            # print('chgt cote')
        
        if not changement_cote:
            xb,yb=self.match.xy_to_position(self.x, self.y, self.match.n, self.match.m)
            xm,ym=self.match.xy_to_position(self.teammate().x, self.teammate().y, self.match.n, self.match.m)
            for robot in self.opponents():
                a,b=np.polyfit([self.x,robot.x],[self.y,robot.y],1)
                if abs(a*robot.x+b-robot.y)<r_robot:
                    xdg,ydg=self.match.xy_to_position(robot.x, robot.y, self.match.n, self.match.m)
                    xg,yg=robot.x,robot.y
                    xd,yd=self.match.xy_to_position(robot.teammate().x, robot.teammate().y, self.match.n, self.match.m)
        else:
            xb,yb=self.match.xy_to_position(-self.x, self.y, self.match.n, self.match.m)
            xm,ym=self.match.xy_to_position(-self.teammate().x, self.teammate().y, self.match.n, self.match.m)
            for robot in self.opponents():
                a,b=np.polyfit([self.x,robot.x],[self.y,robot.y],1)
                if abs(a*robot.x+b-robot.y)<r_robot:
                    xdg,ydg=self.match.xy_to_position(-robot.x, robot.y, self.match.n, self.match.m)
                    xg,yg=-robot.x,robot.y
                    xd,yd=self.match.xy_to_position(-robot.teammate().x, robot.teammate().y, self.match.n, self.match.m)
         
        
       
        return [(xb,yb),(xdg,ydg),(xg,yg),(xd,yd),(xm,ym)]
        
    def distance_droite(self,a,b): #calcul la distance d'un robot par rapport a une droite tel que y=ax+b  
        distance=abs(a*self.x-self.y+b)/np.sqrt(a**2+1)
        return distance
        
class Balle():
    
    def __init__(self,x,y):
        self.x=x
        self.y=y
        self.potentiel=pt.Potentiel(-3,[self.x,self.y],x_grid,y_grid,2500) 
        self.position=complex(x,y)
        self.x10=[x]
        self.y10=[y]
        
    def Position(self,ball): 
        self.x,self.y=ball[6],ball[7]
        self.position=complex(self.x,self.y)
        self.potentiel=pt.Potentiel(-3,[self.x,self.y],x_grid,y_grid,2500) #potentiel négatif pour attirer les robots
        
        if len(self.x10)>9:
            self.x10.pop(0)
            self.y10.pop(0)
        self.x10.append(self.x) #on enregistre les 10 dernieres positions de la balle
        self.y10.append(self.y)
            
        
            
    #Calcul du vecteur correspodant à la trajectoire de la balle    
    def trajectoire(self):
        a,b=np.polyfit(self.x10,self.y10,1)
        vect_balle=complex(1,a)
        if (self.x10[-1]-self.x10[0])<0:
            vect_balle=-vect_balle
        return vect_balle
        
    

class Coach():
    
    def __init__(self,joueurs,couleur,side):
        self.side=side
        self.joueurs=joueurs
        self.nom=couleur
        self.baller=None
        self.passe=False
        self.ia=ia.Agent('ia3')
       
        if self.side=='L':
            self.but=(-1350,0)  
            self.but_adversaire=(1350,0)
       
        else :
            self.but=(1350,0)
            self.but_adversaire=(-1350,0)
          
        
    def __eq__(self, o):
        return self.joueurs==o.joueurs
    
    
    #Fonction qui permet d'attribuer les postes aux 2 joueurs en fonction de l'observation du terrain et des postes des joueurs
          
                
    def changementDePoste(self):
        baller=None
        ball=False
        distance=[]
        closer=None
        for joueur in self.joueurs[0].match.joueurs :
            if joueur.hasTheBall():
                baller=joueur
                ball=True
            balle=self.joueurs[0].match.balle
            d=joueur.distanceToXY(balle.position)
            joueur.distance_balle=d
            distance.append(d)
            
        closer=self.joueurs[0].match.joueurs[np.argmin(distance)]
        
        for joueur in self.joueurs:
            
                
            if joueur.poste[-1]=='WAIT':
                if ball:
                    if baller==joueur:
                        joueur.defPoste('ATT')
                    
                    
                        
                    elif baller.team!=self.nom:
                        if self.whoIsTheGoal()==joueur:
                            joueur.defPoste('GOAL')
                        else:    
                            if ((self.side=='L')&(balle.x<0))|((self.side=='R')&(balle.x>0)):
                                joueur.defPoste('TACKLE')
                                
                            else :
                                joueur.defPoste('DEF')
                else :
                    if closer.team!=self.nom:
                        if self.whoIsTheGoal()==joueur:
                            joueur.defPoste('GOAL')
                            
                        elif (min(distance)/joueur.distanceToXY(balle.position))>0.7:
                            joueur.defPoste('CHASER')
                        
                
                    
                
            elif joueur.poste[-1]=='ATT':
                if self.openGoal(joueur):
                    joueur.defPoste('SHOOTER')
                                        
                    
                else :
                    
                    if self.openGoal(joueur.teammate()):
                        joueur.defPoste('PASSEUR')
                    
                    else :
                        # print('here')
                        
                        field=joueur.create_game()
                        final_pos_joueur,score_joueur,fail=ia.play_game_3(agent=self.ia,game=field)
                        
                        # print(field)
                        # ia.Game(9,9,terrain=field).print()
                        
                        field=joueur.teammate().create_game()
                        final_pos_mate,score_mate,fail=ia.play_game_3(agent=self.ia,game=field)
                        
                        
                        if score_mate>score_joueur+2:
                            if self.side=='L':
                                joueur.teammate().goto=final_pos_mate
                            else:
                                joueur.teammate().goto=(-final_pos_mate[0],final_pos_mate[1])
                            joueur.teammate().goto=final_pos_mate
                           
                            joueur.teammate().status=joueur.teammate().commande_position(joueur.teammate().goto[0], joueur.teammate().goto[1], self.but_adversaire[0],self.but_adversaire[1],spin=True)
                            
                            joueur.teammate().defPoste('DEMARQUE')
                            
                            joueur.defPoste('PASSEUR')
                            
                            
                            
                            
                        else:
                            if self.side=='L':
                                joueur.goto=final_pos_joueur
                            else:
                                joueur.goto=(-final_pos_joueur[0],final_pos_joueur[1])
                            joueur.status=joueur.commande_position(joueur.goto[0], joueur.goto[1], self.but_adversaire[0],self.but_adversaire[1],spin=True)
                            # print(joueur.goto)
                            
                            joueur.defPoste('DRIBBLE')
            
            elif joueur.poste[-1]=='SHOOTER':
                if joueur.status=='DONE':
                    joueur.defPoste('WAIT')
                    if joueur.teammate().poste[-1]=='DEMARQUE':
                        joueur.teammate().defPoste('WAIT')
                
            elif joueur.poste[-1]=='DEMARQUE':
                if ball:
                    if baller in joueur.opponents():
                        joueur.defPoste('WAIT')
                elif (min(distance)/joueur.distanceToXY(balle.position))>0.7:
                            joueur.defPoste('CHASER')

            
            elif joueur.poste[-1]=='DRIBBLE': #commande vers la position calcuée
                if joueur.status=='DONE':
                    joueur.defPoste('ATT')
            
            elif (joueur.poste[-1]=='PASSEUR') : #procédure avant la passe
                #Calcul si la passe est possible
                passe=True
                a,b=np.polyfit([joueur.x,joueur.teammate().x],[joueur.y,joueur.teammate().y],1)
                for robot in joueur.opponents():
                    if robot.distance_droite(a,b)<r_robot:
                        passe=False
                        break
                    
                
                if not passe: #dans ce cas, il faut determiner ce qu'on doit faire
                    print('passe impossible')
                    joueur.defPoste('DRIBBLE')
                    joueur.teammate().defPoste('DEMARQUE')
                    
                
            
            elif (joueur.poste[-1]=='RECEVEUR'): #procédure pendant la passe pour ajuster la position du receveur
                if ball:    
                    if baller==joueur:
                            joueur.defPoste('ATT')
            
            elif joueur.poste[-1]=='CHASER':
                if ball:        
                    if baller:
                        if(baller==joueur):
                            joueur.defPoste('ATT')
                   
            elif joueur.poste[-1]=='GOAL':
                if ball:
                    if baller==joueur:
                        joueur.defPoste('ATT')
                
            elif joueur.poste[-1]=='DEF':
                if ball:
                    if baller==joueur:
                        joueur.defPoste('ATT')
                  
                    elif baller.team!=self.nom:
                        if ((self.side=='L')&(balle.x<0))|((self.side=='R')&(balle.x>0)): #Balle de notre coté
                            joueur.defPoste('TACKLE')
                else:
                    joueur.defPoste('CHASER')
                
            elif joueur.poste[-1]=='TACKLE':
                if ball:
                
                    if baller==joueur:
                        joueur.defPoste('ATT')
                    elif baller==None:
                        joueur.defPoste('CHASER')
                
            
            
               
       
        
        #Affichage changement de status
        if self.joueurs[0].match.disp==0:
            changement=False
            status=''
            for joueur in self.joueurs:
                status+=(f'{joueur} {joueur.poste} {joueur.status} \n')
                if joueur.poste[-2]!=joueur.poste[-1]:
                    changement=True
            if changement:
                print('---------------')
                print(status)
        
        
        
            
    #Réalisation des actions en fonction des postes attribués aux joueurs        
    def action(self):
        for joueur in self.joueurs:
            if joueur.poste[-1]=='SHOOTER':
                joueur.status=joueur.Tir()
                
            elif joueur.poste[-1]=='CHASER':
                joueur.status=joueur.commande_balle()
                
            elif joueur.poste[-1]=='DEMARQUE':
                if (joueur.status=='DONE')&(not self.openGoal(joueur)): #calcul d'une nouvelle position 
                    field=joueur.create_game()
                    final_pos_joueur,score_joueur,fail=ia.play_game_3(agent=self.ia,game=field)
                    if self.side=='L': #l'ia est entrainée que sur un coté donc on doit flip la position
                        joueur.goto=final_pos_joueur
                    else: 
                        joueur.goto=(-final_pos_joueur[0],final_pos_joueur[1])
                    joueur.status=joueur.commande_position(joueur.goto[0], joueur.goto[1], joueur.teammate().x,joueur.teammate().y)
                    
                    
                else : #la position est déjà calculée donc commande vers la position
                    joueur.status=joueur.commande_position(joueur.goto[0], joueur.goto[1], joueur.teammate().x,joueur.teammate().y)
                
                #affichage de la position prédicte
                if joueur.match.disp==2:
                    goto,=ax.plot(joueur.goto[0],joueur.goto[1],'go')
                    ax.draw_artist(goto)
                
                
            
            
            elif joueur.poste[-1]=='DRIBBLE': #commande vers la position calcuée
                 if (joueur.status=='DONE')&(not self.openGoal(joueur)): #calcul d'une nouvelle position 
                    field=joueur.create_game()
                    final_pos_joueur,score_joueur,fail=ia.play_game_3(agent=self.ia,game=field)
                    if self.side=='L': #l'ia est entrainée que sur un coté donc on doit flip la position
                        joueur.goto=final_pos_joueur
                    else: 
                        joueur.goto=(-final_pos_joueur[0],final_pos_joueur[1])

                 joueur.status=joueur.commande_position(joueur.goto[0], joueur.goto[1], self.but_adversaire[0],self.but_adversaire[1],spin=True)
                
                 if joueur.match.disp==2:  #affichage
                    goto,=ax.plot(joueur.goto[0],joueur.goto[1],'go')
                    ax.draw_artist(goto)
                
                
                 joueur.defPoste('DRIBBLE')
            
            elif (joueur.poste[-1]=='PASSEUR') : #procédure avant la passe
                joueur.status=joueur.Passe()
                
                
            
            elif (joueur.poste[-1]=='RECEVEUR') : #procédure pendant la passe pour ajuster la position du receveur
                joueur.reception()
                
                    
            elif joueur.poste[-1]=='GOAL': 
                joueur.defPoste('GOAL')
                joueur.goal()
                
            elif joueur.poste[-1]=='DEF':
                joueur.defPoste('DEF')
                if self.side=='L':
                    x=-300
                    
                else :
                    x=300
      
                joueur.commande_position(x, joueur.match.balle.y/2, joueur.match.balle.x,joueur.match.balle.y)     
                
            elif joueur.poste[-1]=='TACKLE':
                joueur.defPoste('TACKLE')
                joueur.commande_balle()
            
               
            
            
            
        
        
        
    #Permet de savoir si le but est ouvert depuis le robot 'baller'    
    def openGoal(self,baller):
        
        openGoal=True
        shooter=baller
    
        x=shooter.x
        y=shooter.y
        xbut,ybut=self.but_adversaire
        a,b=np.polyfit([x,xbut],[y,ybut],1)    
        for robot in self.joueurs[0].match.joueurs:
           if robot!=shooter:
               if robot.distance_droite(a, b)<r_robot:
                   robot.defPoste('GOAL')
                   openGoal=False
                   break
        return openGoal
    
    
    #Calcul du joueur le plus proche du but => le goal
    def whoIsTheGoal(self):
        if self.side=='L':
            xbut=-1350
            ybut=0
            
        else :
            xbut=+1350
            ybut=0
        goal=None
        d=[]
        for joueur in self.joueurs:
            
            d.append(np.sqrt((xbut-joueur.x)**2+(ybut-joueur.y)**2))
        
        goal=self.joueurs[d.index(min(d))]
        
        return goal
        
class Match():
    
    def __init__(self,nom,disp=0):
        self.nom=nom
        Y0=Robot('Y',0,self)
        Y1=Robot('Y',1,self)
        B0=Robot('B',0,self)
        B1=Robot('B',1,self)
        self.n=9
        self.m=9
        self.disp=disp
        
        self.joueurs=[Y0,Y1,B0,B1]
        Yellow=Coach([self.joueurs[0],self.joueurs[1]],'Y','R')
        Blue=Coach([self.joueurs[2],self.joueurs[3]],'B','L')
        self.balle=Balle(0,0)
        self.blue=Blue
        self.yellow=Yellow
    
    def __repr__(self):
        return f'{self.nom}'
    
    
    #Récupération des données de SSL Vision et actualisation des robots et de la balle + affichage
    def Vision(self):
        balls, blueBots, yellowBots = getVision()
  
        for robot in self.joueurs:
            data=False
            if robot.team=='Y':
                tag='gold'
                for bot in yellowBots:
                    if bot[5]==robot.id:
                        data=True
                        botInfo=bot
            else :
                tag='b'
                for bot in blueBots:
                    if bot[5]==robot.id:
                        data=True
                        botInfo=bot
            
            #parfois il manque une donnée pour un robot
            if data: 
                robot.position(botInfo)
            else :
                self.Vision()
            
            #affichage
            if self.disp==2:
                draw_robot,=ax.plot(robot.x,robot.y,tag,marker='o',ms=20,label=(str(robot)+' '+str(robot.poste[-1])+' '+str(robot.status)))
                text_robot=ax.text(robot.x,robot.y,robot.id,horizontalalignment='center',verticalalignment='center',color='w')
                text2_robot=ax.text(robot.x,robot.y-100,str(robot.poste[-1])+' '+str(robot.status),horizontalalignment='center',verticalalignment='center',color='black')
                ax.draw_artist(draw_robot)
                ax.draw_artist(text_robot)
                ax.draw_artist(text2_robot)
            
            elif self.disp==1:
                if robot.team=='B':
                    x=-700
                else :
                    x=700
                draw_robot=ax.text(x,robot.id*200,str(robot)+' '+str(robot.poste[-1])+' '+str(robot.status),horizontalalignment='center',verticalalignment='center')
                ax.draw_artist(draw_robot)
        
        
        if len(balls)>0:
            self.balle.Position(balls[0])

        if self.disp==2:
            draw_ball,=ax.plot(self.balle.x10,self.balle.y10,'ro')
            ax.draw_artist(draw_ball)
        
        
    #Fonction pour passer d'une position x,y vers une position discrète dans une grille de 9x9 (utile pour l'ia et la création du terrain)
    def xy_to_position(self,x,y,n,m):
        return(int(int(x+longueur)//(2*longueur/m)),int(int(y+largeur)//(2*largeur/n)))
    
    #Fonction inverse de la précédente car l'ia renvoie une position sur la grille de 9x9
    def position_to_xy(self,xd,yd,n,m):
        return(-longueur+(xd+0.5)*(2*longueur)/m,-largeur+(yd+0.5)*(2*largeur)/n)
    
    






#%%Création match


match_test=Match('test',disp=1)

if match_test.disp==2:
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.set_xlim(-longueur,longueur)
    ax.set_ylim(-largeur,largeur)
    ax.add_artist(lines.Line2D((-longueur, -longueur+350, -longueur+350,-longueur), (-350, -350,350,350), color = 'green'))
    ax.add_artist(lines.Line2D((longueur, longueur-350, longueur-350,longueur), (-350, -350,350,350), color = 'green'))
    ax.add_artist(lines.Line2D((0,0), (largeur,-largeur), color = 'green'))
    ax.add_artist(patches.Circle((0, 0), 500,facecolor='None', edgecolor = 'green'))
    ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left',
                ncol=2, mode="expand", borderaxespad=0.)
    text=ax.text(0,largeur-200,"")
    fig.canvas.draw()
    axbackground = fig.canvas.copy_from_bbox(ax.bbox)
    
    


elif match_test.disp==1:
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.set_xlim(-longueur,longueur)
    ax.set_ylim(-largeur,largeur)
    text=ax.text(0,largeur-200,"")
    axbackground = fig.canvas.copy_from_bbox(ax.bbox)
    fig.canvas.draw()
    frame =0
    
t_list = [time.time()]
def t_update(t_list):
    t_list.append(time.time())
    if len(t_list)>30:
        t_list.pop(0)
    return(t_list)
    
while True:
    
    try:
        if match_test.disp>0:
            fig.canvas.restore_region(axbackground)  

        #Match 2v2
        match_test.Vision()
        match_test.blue.changementDePoste()
        match_test.blue.action()
        match_test.yellow.changementDePoste()
        match_test.yellow.action()
        
        
        if match_test.disp>0:
            t_list=t_update(t_list)
            tx = 'Mean Frame Rate:\n {fps:.3f}FPS'.format(fps= (len(t_list) / (t_list[-1] - t_list[0]) )) 
            text.set_text(tx)
            ax.draw_artist(text)
            fig.canvas.blit(ax.bbox)
            fig.canvas.flush_events()
            
            
        
        # xb=match_test.balle.x
        # yb=match_test.balle.y
        # match_test.blue.joueurs[0].commande_position(0,0,xb,yb)
        # break
        


        # match_test.Vision()
        
        
        # if match_test.joueurs[2].hasTheBall() :
            
        #     print("balle atteinte")
        #     p = psl.packetCommandBot(False, 
        #                   id=0, 
        #                   veltangent=0., 
        #                   velnormal=0, 
        #                   velangular=0.)
        #     grSim.send(p)
        # else :
        #     match_test.joueurs[2].commande_balle()
        
        # break
            
            
    except KeyboardInterrupt :#mise à zéro de tous les robots lors de l'intérruption du programme
        print('INTERRUPTION')
        p = psl.packetCommandBot(True, 
                          id=0, 
                          veltangent=0., 
                          velnormal=0, 
                          velangular=0.)
        grSim.send(p)
        p = psl.packetCommandBot(True, 
                          id=1, 
                          veltangent=0., 
                          velnormal=0, 
                          velangular=0.)
        grSim.send(p)
        p = psl.packetCommandBot(False, 
                          id=0, 
                          veltangent=0., 
                          velnormal=0, 
                          velangular=0.)
        grSim.send(p)
        p = psl.packetCommandBot(False, 
                          id=1, 
                          veltangent=0., 
                          velnormal=0, 
                          velangular=0.)
        grSim.send(p)
        break


vision.close()
grSim.close()