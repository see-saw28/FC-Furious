# -*- coding: utf-8 -*-
"""
Created on Fri Mar 12 14:26:07 2021

@author: paulg
"""


import numpy as np
import random

import ia_v3 as ia
import ia_demarquage1 as dmq

import parametres as p


class Coach():
    
    def __init__(self,joueurs,couleur,side):
        self.side=side
        self.joueurs=joueurs
        self.nom=couleur
        self.baller=None
        self.passe=False
        self.ia=ia.Agent('ia3')
        self.dmq=dmq.Agent('dmq6')
       
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
                        joueur.teammate().defPoste('DEMARQUE')
                    
                    
                        
                    elif baller.team!=self.nom:
                        if (self.whoIsTheGoal()==joueur)&(joueur.teammate().poste[-1]!='GOAL'):
                            joueur.defPoste('GOAL')
                            
                        else:    
                            if ((self.side=='L')&(balle.x<0))|((self.side=='R')&(balle.x>0)):
                                # joueur.defPoste('TACKLE')
                                # joueur.defPoste('DEF')
                                defense=random.random()
                                # print(defense)
                                if defense<0.5:
                                    joueur.defPoste('DEF')
                                elif defense < 0.75:
                                    joueur.defPoste('DEF1')
                                else:
                                    joueur.defPoste('DEF2')
                                
                            else :
                                defense=random.random()
                                # print(defense)
                                if defense<0:
                                    joueur.defPoste('DEF')
                                elif defense < 0.5:
                                    joueur.defPoste('DEF1')
                                else:
                                    joueur.defPoste('DEF2')
                                    
                else :
                    if closer.team!=self.nom:
                        if (self.whoIsTheGoal()==joueur)&(joueur.teammate().poste[-1]!='GOAL'):
                            joueur.defPoste('GOAL')
                    
                            
                        elif ((min(distance)/joueur.distanceToXY(balle.position))>0.7)&(joueur.teammate().poste[-1]!='RECEVEUR'):
                            joueur.defPoste('CHASER')
                            joueur.teammate().defPoste('GOAL')
                        
                        else :
                            joueur.defPoste('DEF')
                            joueur.teammate().defPoste('GOAL')
                            
                    # elif (closer==joueur)&(joueur.teammate().poste[-1]!='RECEVEUR'):
                    #     joueur.defPoste('CHASER')
                    
                    else :
                        joueur.defPoste('DEMARQUE')
                        
                
                    
                
            elif joueur.poste[-1]=='ATT':
                if self.openGoal(joueur):
                    joueur.defPoste('SHOOTER')
                                        
                    
                else :
                    
                    if (self.openGoal(joueur.teammate()))&(self.openPasse()):
                        alea=random.random()
                        if alea<0.650:
                            joueur.defPoste('PASSEUR')
                            joueur.teammate().defPoste('RECEVEUR')
                        else:
                            joueur.defPoste('DRIBBLE')
                    
                    elif not self.openGoal(joueur.teammate()) :
                        # print('here')
                        alea=random.random()
                        if alea<0.750:
                            field=joueur.create_game()
                            final_pos_joueur,score_joueur,fail,_=ia.play_game_3(agent=self.ia,game=field)
                            
                            # print(field)
                            # ia.Game(9,9,terrain=field).print()
                            
                            field=joueur.teammate().create_game()
                            final_pos_mate,score_mate,fail,_=ia.play_game_3(agent=self.ia,game=field)
                            
                            
                            if score_mate>score_joueur+2:
                                if self.side=='L':
                                    joueur.teammate().goto=complex(final_pos_mate[0],final_pos_mate[1])
                                else:
                                    joueur.teammate().goto=complex(-final_pos_mate[0],final_pos_mate[1])
                                # joueur.teammate().goto=final_pos_mate
                               
                                joueur.teammate().status=joueur.teammate().commande_position(joueur.teammate().goto.real, joueur.teammate().goto.imag, self.but_adversaire[0],self.but_adversaire[1],spin=True)
                                
                                joueur.teammate().defPoste('DEMARQUE')
                                
                                joueur.defPoste('PASSEUR')
                                
                                
                                
                                
                            else:
                                if self.side=='L':
                                    joueur.goto=complex(final_pos_mate[0],final_pos_mate[1])
                                else:
                                    joueur.goto=complex(-final_pos_joueur[0],final_pos_joueur[1])
                                joueur.status=joueur.commande_position(joueur.goto.real, joueur.goto.imag, self.but_adversaire[0],self.but_adversaire[1],spin=True)
                                # print(joueur.goto)
                                
                                joueur.defPoste('DRIBBLE')
                                
                        else:
                            joueur.defPoste('PASSEUR')
                            joueur.teammate().defPoste('RECEVEUR')
                            
                    elif not self.openPasse():
                        print('here')
                        joueur.defPoste('DRIBBLE')
            
            elif joueur.poste[-1]=='SHOOTER':
                if not ball:
                    joueur.defPoste('ATT')
                if joueur.status=='DONE':
                    joueur.defPoste('WAIT')
                    if joueur.teammate().poste[-1]=='DEMARQUE':
                        joueur.teammate().defPoste('WAIT')
                
            elif joueur.poste[-1]=='DEMARQUE':
                if ball:
                    if baller in joueur.opponents():
                        joueur.defPoste('WAIT')
                        
                    elif baller==joueur:
                        joueur.defPoste('ATT')
                    
                        
                elif (min(distance)/joueur.distanceToXY(balle.position))>0.7:
                            joueur.defPoste('CHASER')
                            
                elif (closer.team!=self.nom):
                    joueur.defPoste('WAIT')
                    

            
            elif joueur.poste[-1]=='DRIBBLE': #commande vers la position calcuée
                # if not ball:
                if joueur.distanceToXY(balle.position)>150:
                    joueur.defPoste('ATT')
            
                if joueur.status=='DONE':
                    joueur.defPoste('ATT')
                    joueur.teammate().defPoste('DEMARQUE')
                if not ball:#perte de balle
                    joueur.defPoste('WAIT')
                    joueur.status='DONE'
                    
                if self.openGoal(joueur):#on tire si le but est ouvert
                    joueur.defPoste('SHOOTER')
                    
            
            elif (joueur.poste[-1]=='PASSEUR') : #procédure avant la passe
                if not ball:
                    joueur.defPoste('WAIT')
                
                    
                
                if not self.openPasse(): #dans ce cas, il faut determiner ce qu'on doit faire
                    print('passe impossible')
                    joueur.status='DONE' 
                    joueur.defPoste('DRIBBLE')
                    joueur.teammate().defPoste('DEMARQUE')
                    
                
            
            elif (joueur.poste[-1]=='RECEVEUR'): #procédure pendant la passe pour ajuster la position du receveur
                if ball:    
                    if baller.team!=self.nom:
                            joueur.defPoste('WAIT')
                            
                if joueur.distanceToXY(balle.position)<200:
                    joueur.defPoste('CHASER')
                    joueur.teammate().defPoste('DEMARQUE')
            
            elif joueur.poste[-1]=='CHASER':
                if ball:        
                    
                    if(baller==joueur):
                        joueur.defPoste('ATT')
                        joueur.teammate().defPoste('DEMARQUE')
                    else :
                        joueur.defPoste('DEF')
                #on évite d'avoir les deux robots qui chassent la balle
                if joueur.teammate().poste[-1]=='CHASER':
            
                    if joueur.distanceToXY(balle.position)>joueur.teammate().distanceToXY(balle.position):
                        joueur.defPoste('DEMARQUE')
                    else:
                        joueur.teammate().defPoste('DEMARQUE')
                   
            elif joueur.poste[-1]=='GOAL':
                if ball:
                    if baller==joueur:
                        joueur.defPoste('ATT')
                        joueur.teammate().defPoste('DEMARQUE')
                #Si la balle est pas loin, on va la récupérer
                if (joueur==closer) and (not ball):
                    joueur.defPoste('CHASER')
                
            elif joueur.poste[-1]=='DEF':
                if ball:
                    if baller==joueur:
                        joueur.defPoste('ATT')
                        joueur.teammate().defPoste('DEMARQUE')
                  
                    elif baller.team!=self.nom:
                        if ((self.side=='L')&(balle.x<0))|((self.side=='R')&(balle.x>0)): #Balle de notre coté
                            # joueur.defPoste('TACKLE')
                            joueur.defPoste('DEF2')
                elif closer==joueur:
                    joueur.defPoste('CHASER')
                    
            elif joueur.poste[-1]=='DEF1':
                if ball:
                    if baller==joueur:
                        joueur.defPoste('ATT')
                        joueur.teammate().defPoste('DEMARQUE')
                  
                    elif baller.team!=self.nom:
                        joueur.defPoste('DEF1')
                elif closer==joueur:
                    joueur.defPoste('CHASER')
                    
            elif joueur.poste[-1]=='DEF2':
                if ball:
                    if baller==joueur:
                        joueur.defPoste('ATT')
                        joueur.teammate().defPoste('DEMARQUE')
                  
                    elif baller.team!=self.nom:
                        joueur.defPoste('DEF2')
                elif closer==joueur:
                    joueur.defPoste('CHASER')
                    
                
            elif joueur.poste[-1]=='TACKLE':
                if joueur.teammate().poste[-1]=='TACKLE':
                    if joueur.distanceToXY(balle.position)>joueur.teammate().distanceToXY(balle.position):
                        joueur.defPoste('GOAL')
                    else:
                        joueur.teammate().defPoste('GOAL')
                    
                if ball:
                
                    if baller==joueur:
                        joueur.defPoste('ATT')
                        joueur.teammate().defPoste('DEMARQUE')
                else:
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
                joueur.defPoste('SHOOTER')
                
            elif joueur.poste[-1]=='CHASER':
                joueur.status=joueur.commande_balle()
                joueur.defPoste('CHASER')
                
            elif joueur.poste[-1]=='DEMARQUE':
                if (joueur.status=='DONE')&(not self.openPasse()) or (abs(joueur.goto-joueur.teammate().goto)<500): #calcul d'une nouvelle position 
                    field=joueur.create_game()
                    final_pos_joueur,score_joueur,fail,_=dmq.play_game_3(agent=self.dmq,game=field)
                    if self.side=='L': #l'ia est entrainée que sur un coté donc on doit flip la position
                        joueur.goto=complex(final_pos_joueur[0],final_pos_joueur[1])
                    else: 
                        joueur.goto=complex(-final_pos_joueur[0],final_pos_joueur[1])
                    joueur.status=joueur.commande_position(joueur.goto.real, joueur.goto.imag, joueur.teammate().x,joueur.teammate().y,spin=True)
                 
                elif (joueur.status=='DONE')&(not self.openGoal(joueur)): #calcul d'une nouvelle position 
                    field=joueur.create_game()
                    final_pos_joueur,score_joueur,fail,_=ia.play_game_3(agent=self.ia,game=field)
                    if self.side=='L': #l'ia est entrainée que sur un coté donc on doit flip la position
                        joueur.goto=complex(final_pos_joueur[0],final_pos_joueur[1])
                    else: 
                        joueur.goto=complex(-final_pos_joueur[0],final_pos_joueur[1])
                    joueur.status=joueur.commande_position(joueur.goto.real, joueur.goto.imag, joueur.teammate().x,joueur.teammate().y,spin=True)
                    
                   
                else : #la position est déjà calculée donc commande vers la position
                    joueur.status=joueur.commande_position(joueur.goto.real, joueur.goto.imag, joueur.teammate().x,joueur.teammate().y,spin=True)
                
                
                joueur.defPoste('DEMARQUE')
                
            
            
            elif joueur.poste[-1]=='DRIBBLE': #commande vers la position calcuée
                 if (joueur.status=='DONE')&(not self.openGoal(joueur)): #calcul d'une nouvelle position 
                    field=joueur.create_game()
                    final_pos_joueur,score_joueur,fail,_=ia.play_game_3(agent=self.ia,game=field)
                    if self.side=='L': #l'ia est entrainée que sur un coté donc on doit flip la position
                        joueur.goto=complex(final_pos_joueur[0],final_pos_joueur[1])
                    else: 
                        joueur.goto=complex(-final_pos_joueur[0],final_pos_joueur[1])

                 joueur.status=joueur.commande_position(joueur.goto.real, joueur.goto.imag, self.but_adversaire[0],self.but_adversaire[1],spin=True)
                
                 
                
                
                 joueur.defPoste('DRIBBLE')
            
            elif (joueur.poste[-1]=='PASSEUR') : #procédure avant la passe
                joueur.status=joueur.Passe()
                joueur.defPoste('PASSEUR')
                
            
            elif (joueur.poste[-1]=='RECEVEUR') : #procédure pendant la passe pour ajuster la position du receveur
                joueur.defPoste('RECEVEUR')    
                joueur.reception()
                
                
                    
            elif joueur.poste[-1]=='GOAL':    
                joueur.goal()
                joueur.defPoste('GOAL')
                
            elif joueur.poste[-1]=='DEF':#placement dans notre camp, mode 'passif'
                if self.side=='L':
                    x=-300
                    
                else :
                    x=300
      
                joueur.commande_position(x, joueur.match.balle.y/2, joueur.match.balle.x,joueur.match.balle.y)     
                joueur.defPoste('DEF')   
                
            elif joueur.poste[-1]=='DEF1':#placement entre les 2 attaquants, orienté vers le plus proche de la balle
                adversaires=joueur.opponents()
                adv1=adversaires[0].positionc
                adv2=adversaires[1].positionc
                placement=(adv1+adv2)/2
                joueur.goto=placement
                distance1=adversaires[0].distanceToXY(joueur.match.balle.position)
                distance2=adversaires[1].distanceToXY(joueur.match.balle.position)
                if distance1<distance2:
                    orientation=adv1
                else:
                    orientation=adv2
                joueur.commande_position(joueur.goto.real,joueur.goto.imag,orientation.real,orientation.imag)
                joueur.defPoste('DEF1')
                
            elif joueur.poste[-1]=='DEF2':#placement entre le 2è attaquant et le but, orienté vers lui
                adversaires=joueur.opponents()
                distance1=adversaires[0].distanceToXY(joueur.match.balle.position)
                distance2=adversaires[1].distanceToXY(joueur.match.balle.position)
                if distance1<distance2:
                    adv=adversaires[1]
                else:
                    adv=adversaires[0]
                pos_adv=adv.positionc
                placement=(pos_adv+complex(self.but[0],self.but[1]))/2
                joueur.goto=placement
                joueur.commande_position(joueur.goto.real,joueur.goto.imag,pos_adv.real,pos_adv.imag)
                joueur.defPoste('DEF2')
            
            elif joueur.poste[-1]=='TACKLE':
                joueur.commande_balle()
                joueur.defPoste('TACKLE')
                
            elif joueur.poste[-1]=='WAIT':
                joueur.commande_position(joueur.x,joueur.y,0,0,spin=True)
                joueur.defPoste('WAIT')
               
            
            
            
        
        
        
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
               if robot.distance_droite(a, b)<p.r_robot:
                   # robot.defPoste('GOAL')
                   openGoal=False
                   break
        return openGoal
    
    def openPasse(self):
        passe=True
        a,b=np.polyfit([self.joueurs[0].x,self.joueurs[1].x],[self.joueurs[0].y,self.joueurs[1].y],1)
        for robot in self.joueurs[0].opponents():
            if robot.distance_droite(a,b)<p.r_robot:
                passe=False
                break
        return passe
    
    
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
    
    def reset(self):
        if self.side=='R':
            x=500
        else :
            x=-500
        self.joueurs[0].commande_position(x, 500, -x, 500)
        self.joueurs[1].commande_position(x, -500, -x, -500)
        self.joueurs[0].defPoste('CHASER')
        self.joueurs[1].defPoste('GOAL')
        
        
    def engagement(self):
        if self.joueurs[0].match.team_engagement==self.nom:
            if self.side=='R':
                x=150
            else :
                x=-150
            
            if self.joueurs[0].hasTheBall():
                self.joueurs[0].commande_position(x, 0, -x, 0,spin=True)
            else :
                self.joueurs[0].commande_balle()
            self.joueurs[1].commande_position(x, -800, 0, 0)
        
            self.joueurs[0].defPoste('CHASER')
            self.joueurs[1].defPoste('DEMARQUE')
        
        else :
            self.joueurs[0].defPoste('DEF2')
            self.joueurs[1].defPoste('GOAL')
            self.action()
            