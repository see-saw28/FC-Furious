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
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import potentiel as pt

# import asservissement as ass
from psl_package import paris_saclay_league as psl
import cmath as c
import time
import numpy as np

import ia as ia




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
K_max=0.6 #facteur prop vitesse
seuil_distance=250
sat_vitesse_angulaire=5
K_angulaire=10
sat_orientation_balle=3
K_orientation_balle=10

##REPULSION
nbPoints=80
r_evit=120
k=4 #facteur de repulsion



X = np.linspace(xmin,xmax,nbPoints)
Y = np.linspace(ymin,ymax,nbPoints)
x_grid,y_grid = np.meshgrid(X,Y)


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
        self.poste=[0,0]
        self.distance_balle=10**6
        self.goto=None
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
    
    def teammate(self):
        if self.team==self.match.blue.nom:
            team=self.match.blue
        else:
            team=self.match.yellow
        for robot in team.joueurs:
            if robot!=self:
                return robot
    
    def opponents(self):
        if self.team!=self.match.blue.nom:
            team=self.match.blue
        else:
            team=self.match.yellow
        return team.joueurs
                
    def defPoste(self,pos):
        self.poste.append(pos)
        if len(self.poste)>2:
            self.poste.pop(0)
    
    def position(self,botInfo):     #permet d'obtenir la position x,y,theta du robot
        
        self.x,self.y,self.orientation=botInfo[6],botInfo[7],botInfo[8]
        self.positionc=complex(self.x,self.y)
        
    
    # def hasTheBall(self):
    #     pos_ball=self.match.balle.position
    #     pos_rob=self.positionc
    #     pos_vitual=pos_rob+complex(r_robot*np.cos(self.orientation),r_robot*np.sin(self.orientation))
    #     distance,phi=c.polar(pos_vitual-pos_ball)
       
    #     if distance <25:
    #         return True
    #     else :
    #         return False
    
    def hasTheBall(self):
        pos_ball=self.match.balle.position
        pos_rob=self.positionc
        
        distance,phi=c.polar(pos_ball-pos_rob)
        delta=self.orientation-phi
        # print(delta,distance)
        if (distance <116)&(abs(delta)<0.31):
            return True
        else :
            return False
    
    def distanceToTheBall(self):
        r_robot=115
        
        pos_ball=self.match.balle.position
        pos_rob=self.positionc
        pos_vitual=pos_rob+complex(r_robot*np.cos(self.orientation),r_robot*np.sin(self.orientation))
        distance,phi=c.polar(pos_vitual-pos_ball)
        # print(distance)
        return distance
        
    def Potentiel(self):        #calcul le potentiel créé par ce robot
        
        self.potentiel=pt.Potentiel(10,[self.x,self.y],x_grid,y_grid,r_evit)
        
    def champ_perso(self,x_depart,x_arrivee,a,b):       #permet de calculer le champ produit par ce robot
        self.Potentiel()
        Ex1,Ey1=pt.Gradient(self.potentiel)
        Ex1,Ey1=pt.Grad(Ex1,Ey1,self.x,self.y,x_depart,x_arrivee,a,b)
        
        potentiel_repulsion=pt.Potentiel(+10,[self.x,self.y],x_grid,y_grid,r_evit/1.5)
        Ex10,Ey10=pt.Gradient(potentiel_repulsion)
        
        self.Ex,self.Ey=k*Ex10+Ex1,k*Ey10+Ey1
        
    def champ_autre(self,x_depart,x_arrivee,a,b):  #permet de calculer le champ provoquer par les autres joueurs
        Ex_autre=0
        Ey_autre=0
        for robot in self.match.joueurs:
            if robot!=self:
                robot.champ_perso(x_depart, x_arrivee, a, b)
                Ex,Ey=robot.Ex,robot.Ey
                Ex_autre+=Ex
                Ey_autre+=Ey
        self.Ex_autre,self.Ey_autre=Ex_autre,Ey_autre
        
    def commande_position(self,x,y,xo,yo,spin=False): #commande vers une position avec orientation
        position_arrivee=Balle(x,y)     
        Exb,Eyb=pt.Gradient(position_arrivee.potentiel)
        
        xd=int((self.x+longueur)/(2*longueur)*nbPoints)
        yd=int((self.y+largeur)/(2*largeur)*nbPoints)
        # print(Ex[yd,xd],xd,self.x,self.y)
        
        a,b=np.polyfit([x,self.x],[y,self.y],1)
        self.champ_autre(self.x,x,a,b)
        Ex,Ey=Exb+k*self.Ex_autre,Eyb+k*self.Ey_autre
        Ex,Ey=pt.norme(Ex,Ey)
        vecteur=position_arrivee.position-self.positionc
        distance,phi=c.polar(vecteur)
        
        o=c.polar(complex(xo,yo)-self.positionc)[1]
        delta=(o-self.orientation)
        delta+=np.pi
        delta=delta%(2*np.pi)
        delta-=np.pi
        
        vitesse_angulaire=min(sat_vitesse_angulaire,abs(delta)*K_angulaire)*np.sign(delta)
        
        vitesse_tangente=K_max*(np.sin(self.orientation)*Ey[yd,xd]+np.cos(self.orientation)*Ex[yd,xd])
        vitesse_normale=K_max*(-np.sin(self.orientation)*Ex[yd,xd]+np.cos(self.orientation)*Ey[yd,xd])
       
        if (abs(distance)>35)|(abs(delta)>0.05):
            
            if distance<seuil_distance:
                vitesse_tangente=vitesse_tangente*distance/seuil_distance
                vitesse_normale=vitesse_normale*distance/seuil_distance
            p = psl.packetCommandBot(self.team=='Y', 
                              id=self.id, 
                              veltangent=vitesse_tangente, 
                              velnormal=vitesse_normale, 
                              velangular=vitesse_angulaire,
                              spinner=spin)
            grSim.send(p)
        
            return 'EN COURS'
        
            
            # time.sleep(0.05)
            
            
            
            
        else :    
            p = psl.packetCommandBot(self.team=='Y', 
                                 id=self.id, 
                                 veltangent=0., 
                                 velnormal=0, 
                                 velangular=0.,
                                 spinner=True)
            grSim.send(p)
        
            return 'DONE'
        
    def commande_balle(self): #permet de recuperer la balle
        ballon=self.match.balle
        self.commande_position(ballon.x,ballon.y,ballon.x,ballon.y) 
        
        
        
    
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
        
        
    def Tir(self):
        xbut=1350
        ybut=100
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
     

    def Passe(self):
        
        passeur=self.positionc
        mate=self.teammate()
        receveur=mate.positionc
        direction=receveur-passeur
        distance,phi=c.polar(direction)
        
        self.orientation_with_ball(mate.x,mate.y)
        angle=(phi+np.pi)%(2*np.pi)
        if angle>np.pi:
            angle-=2*np.pi
        
        
        mate.commande_position(mate.x, mate.y,self.x,self.y )
        
        delta=phi-self.orientation
        delta1=angle-mate.orientation
    
        if (abs(delta)<.05)&(abs(delta1)<0.05):
            
            p = psl.packetCommandBot(self.team=='Y', 
                         id=self.id, 
                         veltangent=0., 
                         velnormal=0, 
                         velangular=0.,
                         kickspeedx=distance/400)
            grSim.send(p)
            print('Passe')
            
            
    
    def reception(self):  
        mate=self.teammate()
        receveur=self.positionc
        passeur=mate.positionc
        direction=receveur-passeur
        distance,phi=c.polar(direction)
        
        
        angle=(phi+np.pi)%(2*np.pi)
        if angle>np.pi:
            angle-=2*np.pi
        vect_balle=self.match.balle.trajectoire()
        d,o_balle=c.polar(vect_balle)
        theta=angle-o_balle
        if abs(theta)>0.4:
            self.defPoste('AT2')
        dU=complex(-direction.imag,direction.real)
        dU=10*np.sin(theta)*dU
        # print(dU)
        self.commande_position(self.x+dU.real, self.y+dU.imag,mate.x,mate.y )
        

        
    def goal(self):
        if self.match.blue.nom==self.team:
            team=self.match.blue
        else:
            team=self.match.yellow
            
        if team.side=='L':
            x=-1350
            xg=-1000
            y=0
           
        else :
            x=1350
            xg=1000
            y=0
            
        
        balle=self.match.balle
        
        a,b=np.polyfit([balle.x,x],[balle.y,y],1)
        yg=a*xg+b
        if abs(yg)>190:
            yg=np.sign(yg)*400
            xg=(yg-b)/a
        # distance,phi=c.polar(balle.position-complex(x,y))
        self.commande_position(xg, yg, balle.x,balle.y)
        
        
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
        self.potentiel=pt.Potentiel(-3,[self.x,self.y],x_grid,y_grid,2500)
        
        if len(self.x10)>9:
            self.x10.pop(0)
            self.y10.pop(0)
        self.x10.append(self.x)
        self.y10.append(self.y)
            
        
            
        
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
        
    def __eq__(self, o):
        return self.joueurs==o.joueurs
    
    def whereIstheBall(self):
        ball=False
        baller=None
        distance=[]
        passe=False
        for joueur in self.joueurs[0].match.joueurs :
            if joueur.hasTheBall():
                ball=True
                baller=joueur
            
            d=joueur.distanceToTheBall()
            joueur.distance_balle=d
            distance.append(d)
            
            if joueur.poste[-1]=='RECEVEUR':
                passe=True
            
            if (joueur.poste[-1]=='DRIBBLE')&(joueur.status=='DONE'):
                joueur.defPoste('RECEVEUR')
                
                
            
               
        
        if ball: 
            if (baller.team==self.nom) & (not passe):
                # baller.defPoste('BALLER')
                self.baller=baller
                if self.openGoal(baller):
                    baller.defPoste('SHOOTER')
                    baller.teammate().defPoste('AT2')
                    
                
                elif (baller.poste[-1]=='DRIBBLE')&(baller.status=='EN COURS') :
                    baller.defPoste('DRIBBLE')
                    baller.teammate().defPoste('MATE' ) 
                    
                    
                elif (baller.poste[-1]=='DRIBBLE')&(baller.status=='DONE'):
                    baller.defPoste('BALLER')
                    baller.teammate().defPoste('MATE' ) 
                    
                else :
                    
                    if self.openGoal(baller.teammate()):
                        passe=True
                        baller.defPoste('PASSEUR')
                        baller.teammate().defPoste('RECEVEUR')
                    
                    elif baller.status!='EN COURS':
                        print('here')
                        baller.defPoste('BALLER')
                        field=self.joueurs[0].match.create_game()
                        final_pos_baller,score_baller=ia.play_game('ia2',game=field)
                        baller.defPoste('MATE')
                        baller.teammate().defPoste('BALLER')
                        field=self.joueurs[0].match.create_game()
                        final_pos_mate,score_mate=ia.play_game('ia2',game=field)
                        
                        print(score_mate>score_baller)
                        if score_mate>score_baller+1000:
                            baller.teammate().goto=final_pos_mate
                            baller.teammate().status=baller.teammate().commande_position(baller.teammate().goto[0], baller.teammate().goto[1], 0,spin=True)
                            
                            baller.teammate().defPoste('DRIBBLE')
                            
                            baller.defPoste('PASSEUR')
                            baller.status='EN COURS'
                            
                            
                            
                        else:
                        
                            baller.goto=final_pos_baller
                            baller.status=baller.commande_position(baller.goto[0], baller.goto[1], 0,50,spin=True)
                            
                            baller.defPoste('DRIBBLE')
                            baller.status='EN COURS'
                            
                            
                        
                    
                
                    # passe=True
                    
                # print('attaque PASSE',baller)
            
            if (baller.team==self.nom) & (passe):
                if baller.teammate()==self.baller:
                    passe=False
                    baller.defPoste('AT2')
                    
                
            elif baller.team!=self.nom :
                # print('defense1',baller)
                goal=self.whoIsTheGoal()
                goal.defPoste('GOAL')
                
                if ((baller.x<0)&(self.side=='L'))|((baller.x>0)&(self.side=='R')):
                    goal.teammate().defPoste('TACKLE')
                    
                else :
                    goal.teammate().defPoste('DEF')
        elif (not ball) & (not passe):
            closer=self.joueurs[0].match.joueurs[distance.index(min(distance))]
            if closer.team==self.nom:
                # print('attaque GO TO THE BALL',closer)
                closer.defPoste('CHASER')
                closer.teammate().defPoste('AT2')
            else :
                # print('defense', closer)
                goal=self.whoIsTheGoal()
                goal.defPoste('GOAL')
                goal.teammate().defPoste('DEF')
        
        elif (not ball) & passe:
            for joueur in self.joueurs:
                if joueur.poste[-1]=='PASSEUR':
                    joueur.defPoste('AT2')
        
        
        
        
        changement=False
        status=''
        for joueur in self.joueurs:
            status+=(f'{joueur} {joueur.poste} {joueur.status} \n')
            if joueur.poste[-2]!=joueur.poste[-1]:
                changement=True
        if changement:
           print('---------------')
           print(status)
        
        
        
            
            
    def action(self):
        for joueur in self.joueurs:
            if joueur.poste[-1]=='SHOOTER':
                joueur.Tir()
            elif joueur.poste[-1]=='CHASER':
                joueur.status=joueur.commande_balle()
                
            elif joueur.poste[-1]=='AT2':
                p = psl.packetCommandBot(joueur.team=='Y', 
                          id=joueur.id, 
                          veltangent=0., 
                          velnormal=0, 
                          velangular=0.)
                grSim.send(p)
                
            elif joueur.poste[-1]=='BALLER':
                # field=self.joueurs[0].match.create_game()
                # final_pos,score=ia.play_game('ia2',game=field)
                
                # joueur.goto=final_pos
                # joueur.status=joueur.commande_position(joueur.goto[0], joueur.goto[1], 0,spin=True)
                
                joueur.defPoste('DRIBBLE')
                joueur.defPoste('DRIBBLE')
                # print(final_pos)
                # time.pause(5)
            
            elif joueur.poste[-1]=='DRIBBLE':
                joueur.status=joueur.commande_position(joueur.goto[0], joueur.goto[1], 0,50,spin=True)
                joueur.defPoste('DRIBBLE')
            
            elif (joueur.poste[-1]=='PASSEUR') & (joueur.teammate().poste[-1]=='RECEVEUR'):
                joueur.Passe()
                joueur.defPoste('PASSEUR')
                
            
            elif (joueur.poste[-1]=='RECEVEUR') & (joueur.teammate().poste[-1]=='AT2'):
                joueur.reception()
                joueur.defPoste('RECEVEUR')
                
                if joueur.distance_balle<100:
                    joueur.poste[-1]=='CHASER'
                    
            elif joueur.poste[-1]=='GOAL':
                joueur.goal()
                
            elif joueur.poste[-1]=='DEF':
                
                if self.side=='L':
                    x=-600
                    o=0
                else :
                    x=600
                    o=np.pi
                    
                
                joueur.commande_position(x, 0, 50,0)     
                
            elif joueur.poste[-1]=='TACKLE':
                joueur.commande_balle()
            
               
            
            
            
        
        
        
        
    def openGoal(self,baller):
        
        openGoal=True
        shooter=baller
    
        x=shooter.x
        y=shooter.y
        if self.side=='L':
            xbut=1350
            ybut=0
            
        else :
            xbut=-1350
            ybut=0
        a,b=np.polyfit([x,xbut],[y,ybut],1)    
        for robot in self.joueurs[0].match.joueurs:
           if robot!=shooter:
               if abs(robot.y-(robot.x*a+b))<r_robot:
                   robot.defPoste('GOAL')
                   openGoal=False
                   break
        return openGoal
    
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
    
    def __init__(self,nom):
        self.nom=nom
        Y0=Robot('Y',0,self)
        Y1=Robot('Y',1,self)
        B0=Robot('B',0,self)
        B1=Robot('B',1,self)
        self.n=11
        self.m=11
        
        
        self.joueurs=[Y0,Y1,B0,B1]
        Yellow=Coach([self.joueurs[0],self.joueurs[1]],'Y','R')
        Blue=Coach([self.joueurs[2],self.joueurs[3]],'B','L')
        self.balle=Balle(0,0)
        self.blue=Blue
        self.yellow=Yellow
    
    def __repr__(self):
        return f'{self.nom}'
    
    def Vision(self):
        balls, blueBots, yellowBots = getVision()
        # ax.clear()
        # ax.set_xlim(-longueur,longueur)
        # ax.set_ylim(-largeur,largeur)
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
            if data:
                robot.position(botInfo)
            else :
                self.Vision()
                
           
        #     ax.plot(robot.x,robot.y,tag,marker='o',ms=20,label=(str(robot)+' '+str(robot.poste[-1])))
        #     ax.text(robot.x,robot.y,robot.id,horizontalalignment='center',verticalalignment='center',color='w')
        if len(balls)>0:
            self.balle.Position(balls[0])
            # print(balls[0][2])
        
        # ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left',
        #    ncol=2, mode="expand", borderaxespad=0.)
        # ax.plot(self.balle.x10,self.balle.y10,'ro')
    
    
    def xy_to_position(self,x,y,n,m):
        return(int(int(x+longueur)//(2*longueur/m)),int(int(y+largeur)//(2*largeur/n)))
    
    def position_to_xy(self,xd,yd,n,m):
        return(-longueur+(xd+0.5)*(2*longueur)/m,-largeur+(yd+0.5)*(2*largeur)/n)
    
    def create_game(self):
        for robot in self.joueurs:
            if robot.poste[-1]=='BALLER':
                xb,yb=self.xy_to_position(robot.x, robot.y, self.n, self.m)
                xm,ym=self.xy_to_position(robot.teammate().x, robot.teammate().y, self.n, self.m)
            elif robot.poste[-1]=='GOAL':
                xdg,ydg=self.xy_to_position(robot.x, robot.y, self.n, self.m)
                xg,yg=robot.x,robot.y
                xd,yd=self.xy_to_position(robot.teammate().x, robot.teammate().y, self.n, self.m)
        return [(xb,yb),(xdg,ydg),(xg,yg),(xd,yd),(xm,ym)]




match_test=Match('test')

# fig = plt.figure()
# ax = fig.add_subplot(1, 1, 1)
# ax.set_xlim(-longueur,longueur)
# ax.set_ylim(-largeur,largeur)



    
while True:
    match_test.Vision()
    # print(match_test.joueurs[3].hasTheBall())
    match_test.blue.whereIstheBall()
    match_test.blue.action()
    # xb=match_test.balle.x
    # yb=match_test.balle.y
    # match_test.blue.joueurs[0].commande_position(0,0,xb,yb)
    # break
    
    # plt.pause(0.3)
    # match_test.yellow.whereIstheBall()
    # match_test.yellow.action()



vision.close()
grSim.close()