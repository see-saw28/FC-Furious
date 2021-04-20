#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 27 13:39:26 2021

@author: psl
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as lines
import time
import parametres as p

#initialisation du plot
def init(disp):
    
    #affichage complet
    if disp==2:
        fig = plt.figure(figsize=[6,6])
        ax = fig.add_subplot(1, 1, 1)
        ax.set_xlim(-p.longueur-500,p.longueur+500)
        ax.set_ylim(-p.largeur-500,p.largeur+500)
        #tracage des lignes du terrain
        ax.add_artist(lines.Line2D((-p.longueur, +p.longueur), (p.largeur, p.largeur), color = 'green'))
        ax.add_artist(lines.Line2D((-p.longueur, -p.longueur+350, -p.longueur+350,-p.longueur), (-350, -350,350,350), color = 'green'))
        ax.add_artist(lines.Line2D((p.longueur, p.longueur-350, p.longueur-350,p.longueur), (-350, -350,350,350), color = 'green'))
        ax.add_artist(lines.Line2D((0,0), (p.largeur,-p.largeur), color = 'green'))
        ax.add_artist(patches.Circle((0, 0), 500,facecolor='None', edgecolor = 'green'))
        
        score=ax.text(-400,p.largeur+200,"")
        strat=ax.text(-1400,p.largeur+200,"")
        text=ax.text(600,p.largeur+200,"")
        fig.canvas.draw()
        
        #on enregistre le terrain pour ne pas avoir le retracer à chaque fois
        #gain en performance d'affichage
        axbackground = fig.canvas.copy_from_bbox(ax.bbox)
        
        
    
    #affichage des status
    elif disp==1:
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.set_xlim(-p.longueur,p.longueur)
        ax.set_ylim(-p.largeur,p.largeur+200)
        text=ax.text(600,p.largeur-400,"")
        score=ax.text(-400,p.largeur-400,"")
        strat=ax.text(-1400,p.largeur+200,"")
        axbackground = fig.canvas.copy_from_bbox(ax.bbox)
        fig.canvas.draw()
        
    else :
        fig,ax,axbackground,text,score,start=None,None,None,None,None
        
    return fig,ax,axbackground,text,score,strat
        
        
        
    
#mis à jour de l'affichage        
def refresh(match,ax,score,strat):
    #affichage complet
    if match.disp==2:
        for joueur in match.joueurs:
            
            
            
            #affichage du robot avec son status
            if joueur.team=='Y':
                tag='gold'
                
            else :
                tag='b'
            
            #position joueur
            draw_joueur,=ax.plot(joueur.x,joueur.y,tag,marker='o',ms=20,label=(str(joueur)+' '+str(joueur.poste[-1])+' '+str(joueur.status)))
            text_joueur=ax.text(joueur.x,joueur.y,joueur.id,horizontalalignment='center',verticalalignment='center',color='w')
            #status joueur
            text2_joueur=ax.text(joueur.x,joueur.y-100,str(joueur.poste[-1])+' '+str(joueur.status),horizontalalignment='center',verticalalignment='center',color='black')
            
           
            ax.draw_artist(draw_joueur)
            ax.draw_artist(text_joueur)
            ax.draw_artist(text2_joueur)
            #affichage de son point de destination
            if joueur.poste[-1]=='RECEVEUR':
                tag='mo'
            else :
                tag='go'
            goto,=ax.plot(joueur.goto.real,joueur.goto.imag,tag)
            ax.draw_artist(goto)
        
        #affichage score
        if match.blueSide=='L':
            tx = f'BLEU : {match.score_bleu} - {match.score_jaune} : JAUNE'
        else:
            tx = f'JAUNE : {match.score_jaune} - {match.score_bleu} : BLEU'
        score.set_text(tx)
        ax.draw_artist(score)
        
        #affichage strat
        strat.set_text(match.strategie)
        ax.draw_artist(strat)
            
  
        
        #affichage de la balle
        draw_ball,=ax.plot(match.balle.x10,match.balle.y10,'ro')
        ax.draw_artist(draw_ball)
    
    #affichage des status
    elif match.disp==1:
        if match.blueSide=='L':
            tx = f'BLEU : {match.score_bleu} - {match.score_jaune} : JAUNE'
        else:
            tx = f'JAUNE : {match.score_jaune} - {match.score_bleu} : BLEU'
        score.set_text(tx)
        ax.draw_artist(score)
        for joueur in match.joueurs:
            if joueur.team=='B':
                x=-700
            else :
                x=700
            draw_robot=ax.text(x,joueur.id*200,str(joueur)+' '+str(joueur.poste[-1])+' '+str(joueur.status),horizontalalignment='center',verticalalignment='center')
            ax.draw_artist(draw_robot)
        
        #affichage strat
        strat.set_text(match.strategie)
        ax.draw_artist(strat)
            
#fonction pour calculer les FPS
def t_update(t_list):
    t_list.append(time.time())
    if len(t_list)>30:
        t_list.pop(0)
    return(t_list)