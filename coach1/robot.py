# -*- coding: utf-8 -*-
"""
Created on Fri Mar 12 14:23:15 2021

@author: paulg
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

import serial
import serial.tools.list_ports

import match as match

import parametres as p



class Robot():
    
    def __init__(self,team,numero,match,grSim,com):
        self.id=numero
        self.x=0
        self.y=0
        self.orientation=0
        self.match=match
        self.positionc=0
        self.team=team
        self.poste=[0,'CHASER']
        self.distance_balle=10**6
        self.goto=complex(0,0)
        self.status='INI'
        self.origine_passe=complex(0,0)
        self.grSim=grSim
        self.com=com
    
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
    def myTeam(self):
        if self.team==self.match.blue.nom:
            team=self.match.blue
        else:
            team=self.match.yellow
            
        return team
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
        self.potentiel=pt.Potentiel(10,[self.x,self.y],p.x_grid,p.y_grid,p.r_evit)
    
        
    #Calcule le champ tournant et repulsif produit par ce robot en fct de la position du robot sur la trajectoire
    def champ_perso(self,x_depart,x_arrivee,a,b):       
        self.Potentiel()
        Ex1,Ey1=pt.Gradient(self.potentiel)
        Ex1,Ey1=pt.Grad(Ex1,Ey1,self.x,self.y,x_depart,x_arrivee,a,b)
        
        potentiel_repulsion=pt.Potentiel(+10,[self.x,self.y],p.x_grid,p.y_grid,p.r_evit/1.5)
        Ex10,Ey10=pt.Gradient(potentiel_repulsion)
        
        self.Ex,self.Ey=p.k*Ex10+Ex1,p.k*Ey10+Ey1
    
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
        self.goto=complex(x,y)
        position_arrivee=match.Balle(x,y)     
        
        vecteur=position_arrivee.position-self.positionc
        distance,phi=c.polar(vecteur)
        
        o=c.polar(complex(xo,yo)-self.positionc)[1]
        delta=(o-self.orientation)
        delta+=np.pi
        delta=delta%(2*np.pi)
        delta-=np.pi
        
        vitesse_angulaire=min(p.sat_vitesse_angulaire,abs(delta)*p.K_angulaire)*np.sign(delta)
        
        distance_autres_joueurs=[]
        distance_autres_joueurs_balle=[]
        for robot in self.match.joueurs:
            if robot!=self:
                distance_autres_joueurs.append(self.distanceToXY(robot.positionc))
                distance_autres_joueurs_balle.append(robot.distanceToXY(robot.match.balle.position))
        
        
        
        #si l'objectif est proche mais un autre robot aussi alors on fait abstraction des champs repulsifs
        if (min(distance_autres_joueurs)>distance) & (min(distance_autres_joueurs_balle)<300) & (distance<300) & balle:
            # print("oui")
            
            self.commande_robot(p.K_proche*np.cos(delta), p.K_proche*np.sin(delta), vitesse_angulaire,spinner=True)
            
        
            return 'EN COURS'
        
        elif (abs(distance)>35)|(abs(delta)>0.05):
            #champ créé par l'objectif
            Exb,Eyb=pt.Gradient(position_arrivee.potentiel)
            
            #Calcul des positions x,y discrètes sur la grille
            xd=int((self.x+p.longueur)/(2*p.longueur)*p.nbPoints)
            yd=int((self.y+p.largeur)/(2*p.largeur)*p.nbPoints)
            # print(Ex[yd,xd],xd,self.x,self.y)
            
            
            #test si les robots sont sur le terrain
            if xd<p.nbPoints and yd<p.nbPoints:
                a,b=np.polyfit([x,self.x],[y,self.y],1)
                
                self.champ_autre(self.x,x,y,a,b)
                Ex,Ey=Exb+p.k*self.Ex_autre,Eyb+p.k*self.Ey_autre
                #champ surfaces interdites
                Ex+=2*p.surface_x
                Ey+=2*p.surface_y
                
                Ex,Ey=pt.norme(Ex,Ey)
                
                # #Affichage du champ
                # fig = plt.figure()
                # ax1 = fig.add_subplot(1, 1, 1)
                # ax1.quiver(x_grid,y_grid,Ex,Ey,width=0.0008)
                
                #Saturation
                vitesse_tangente=min(p.saturation_vitesse_tangente,p.K_max*(np.sin(self.orientation)*Ey[yd,xd]+np.cos(self.orientation)*Ex[yd,xd]))
                vitesse_normale=min(p.saturation_vitesse_normale,p.K_max*(-np.sin(self.orientation)*Ex[yd,xd]+np.cos(self.orientation)*Ey[yd,xd]))
                
                if distance<p.seuil_distance:
                    vitesse_tangente=vitesse_tangente*distance/p.seuil_distance
                    vitesse_normale=vitesse_normale*distance/p.seuil_distance
                    spin=True
                
                
                self.commande_robot(vitesse_tangente, vitesse_normale, vitesse_angulaire,spinner=spin)
                
            
                return 'EN COURS'
            
            
            
        #Objectif atteint    
        else :    
            self.commande_robot(0, 0, 0,spinner=True)
        
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
        vitesse_angulaire=min(p.sat_orientation_balle,abs(delta)*p.K_orientation_balle)*np.sign(delta)
        if (abs(delta)>.05):
            
            self.commande_robot(0, -vitesse_angulaire/9.25, vitesse_angulaire,spinner=True)

        else :
            self.commande_robot(0, 0, 0,spinner=True)
       
        
        
    
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
        # self.commande_position(self.x,self.y,xbut,ybut)

        delta=phi-self.orientation
        
        if (abs(delta)>.5)&(not self.myTeam().openGoal(self)):
            self.defPoste('ATT')
            return 'DONE'
        
        if (abs(delta)<.05):
            self.commande_robot(0, 0, 0,tir=5)
            print('Boom')
            return 'DONE'
        else :
            return 'EN COURS'
     
        
    #Commande pour orienter les deux joueurs face à face avant une passe puis passe
    def Passe(self):
        
        passeur=self.positionc
        mate=self.teammate()
        mate.origine_passe=passeur
        receveur=complex(mate.goto.real,mate.goto.imag)
        direction=receveur-passeur
        distance,phi=c.polar(direction)
        
        #orientation du passeur vers le receveur
        self.orientation_with_ball(mate.goto.real,mate.goto.imag)
        # self.commande_position(self.x,self.y,mate.x,mate.y)
        
        angle=(phi+np.pi)%(2*np.pi)
        if angle>np.pi:
            angle-=2*np.pi
        
        #orientation du receveur vers le passeur
        mate.commande_position(mate.goto.real, mate.goto.imag,self.x,self.y )
        
        delta=phi-self.orientation
        delta1=angle-mate.orientation
        
        
        
        #Alignement réussi IDEE arrêt lors du changement de signe
        if (abs(delta)<.05)&(abs(delta1)<0.55):
            if mate.distanceToXY(mate.goto)>150:
                puissance=self.puissanceKicker(distance)
            else :
                puissance=distance/400
            self.commande_robot(0, 0, 0,tir=puissance)
            print('Passe')
            self.defPoste('DEMARQUE')
            self.teammate().defPoste('RECEVEUR')
            return 'DONE'
        
        else :
            return 'EN COURS'
            
            
    #Commande pour que le receveur se dirige vers le point d'arrivée de la balle
    def reception(self):  
        # print('rec')
        mate=self.teammate()
        receveur=self.positionc
        passeur=self.origine_passe
        direction=receveur-passeur
        distance,phi=c.polar(direction)
        
        
        # angle=(phi+np.pi)%(2*np.pi)
        # if angle>np.pi:
        #     angle-=np.pi
        
        #vecteur directeur de la balle
        vect_balle=self.match.balle.trajectoire()
        d,o_balle=c.polar(vect_balle)
        
        #Angle entre le vecteur entre les deux robots et le vecteur de la balle
        theta=o_balle-phi
        # print(phi,o_balle)
        
        # print(theta,((self.match.balle.x10[-10]-self.match.balle.x10[-1])**2+(self.match.balle.y10[-10]-self.match.balle.y10[-1])**2))
        
        #Si on observe un trop gros écart angulaire alors la passe a rebondit ou elle est ratée
        if (np.sqrt(((self.match.balle.x10[0]-self.match.balle.x10[-1])**2+(self.match.balle.y10[0]-self.match.balle.y10[-1])**2))>300):
            
            if (abs(theta)>0.4):
                self.defPoste('WAIT')
            else :
                # self.defPoste('RECEVEUR')
                
                dU=complex(-direction.imag,direction.real) #rotation a 90° du vecteur reliant les robots
                dU=2*np.sin(theta)*dU #projection de la position où la balle devrait arriver
                # print(theta,dU)
                # self.goto=complex(self.x+dU.real, self.y+dU.imag)
                    
                    
                self.commande_position(self.x+dU.real, self.y+dU.imag,mate.x,mate.y )
    
            
    def puissanceKicker(self,distance):
        if distance>140:
            return 0.05249286*(distance-140)**0.5
        else :
            return 0
    
        
    def PasseEnProfondeur(self):
        passeur=self.positionc
        mate=self.teammate()
        receveur=mate.goto
        direction=receveur-passeur
        distance,phi=c.polar(direction)
        
        #orientation du passeur vers l'endroit de la passe
        # self.orientation_with_ball(mate.x,mate.y)
        self.commande_position(self.x,self.y,mate.x,mate.y)
        
        angle=(phi+np.pi)%(2*np.pi)
        if angle>np.pi:
            angle-=2*np.pi
        
        
        delta=phi-self.orientation
        
        
        #Alignement réussi
        if abs(delta)<.05:
            
            self.commande_robot(0, 0, 0,tir=self.puissanceKicker(distance))
            print('Passe')
            self.defPoste('DEMARQUE')
            
            return 'DONE'
        
        else :
            return 'EN COURS'
     
        
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
                if robot.distance_droite(a,b)<p.r_robot:
                    xdg,ydg=self.match.xy_to_position(robot.x, robot.y, self.match.n, self.match.m)
                    xg,yg=robot.x,robot.y
                    xd,yd=self.match.xy_to_position(robot.teammate().x, robot.teammate().y, self.match.n, self.match.m)
        else:
            xb,yb=self.match.xy_to_position(-self.x, self.y, self.match.n, self.match.m)
            xm,ym=self.match.xy_to_position(-self.teammate().x, self.teammate().y, self.match.n, self.match.m)
            for robot in self.opponents():
                a,b=np.polyfit([self.x,robot.x],[self.y,robot.y],1)
                if robot.distance_droite(a,b)<p.r_robot:
                    xdg,ydg=self.match.xy_to_position(-robot.x, robot.y, self.match.n, self.match.m)
                    xg,yg=-robot.x,robot.y
                    xd,yd=self.match.xy_to_position(-robot.teammate().x, robot.teammate().y, self.match.n, self.match.m)
         
        
       
        return [(xb,yb),(xdg,ydg),(xg,yg),(xd,yd),(xm,ym)]
        
    def distance_droite(self,a,b): #calcul la distance d'un robot par rapport a une droite tel que y=ax+b  
        distance=abs(a*self.x-self.y+b)/np.sqrt(a**2+1)
        return distance
    
    def commande_com(self,ID,Vtan,Vnorm,Vrot,spinner=False,tir=0):
        chaine_commande = "D"+str(ID) + ","
        liste_vitesses = [Vtan,Vnorm,Vrot]
        spin=0
        if spinner:
            spin=1
            
        for vitesse in liste_vitesses :
            vitesse = int(vitesse*999) #Régler ici si la vitesse max n'est pas 1
            
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
    
        for charactere in chaine_commande:
                self.com.write(str.encode(charactere))
        return chaine_commande
    
    def commande_robot(self,Vtang,Vnorm,Vang,spinner=False,tir=0):
        if self.grSim!=None:
            p = psl.packetCommandBot(self.team=='Y', 
                         id=self.id, 
                         veltangent=Vtang, 
                         velnormal=Vnorm, 
                         velangular=Vang,
                         spinner=spinner,
                         kickspeedx=tir)
            self.grSim.send(p)
        if self.com:
            start=time.time()
            self.commande_com(self.id,Vtang,Vnorm,Vang,spinner,tir)
            print(time.time()-start)