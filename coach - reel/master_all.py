#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr  2 16:55:40 2021

@author: psl
"""



#%% Librairies

import matplotlib.pyplot as plt
import serial
import serial.tools.list_ports
import time
import os

from psl_package import paris_saclay_league as psl


import match as match
import affichage

if os.name=='nt':
    import manette_win as m
    print('WINDOWS')
    
elif os.name=='posix':
    import manette_linux as m
    print('LINUX')





if __name__ == "__main__":

    
    #%% Test manette
    try:
        m.init()
        manette = True
        print('Manette détectée')
    except:
        manette = False
        print('Pas de manette détectée')
        
        
    # Main loop, one can press the PS button to break
    quit = False
    
    #%% Test communication
    try :
        #Liste des ports
        # print(list(serial.tools.list_ports.comports()))
        
        #LINUX
        if os.name=='posix':
            ser = serial.Serial("/dev/ttyACM0",baudrate=115200,bytesize = 8, parity='N', stopbits=1, timeout=None,  write_timeout=None, xonxoff=False, rtscts=False, dsrdtr=False )  # open first serial port
        
        #WINDOWS
        elif os.name=='nt':
            ser = serial.Serial("COM6",baudrate=115200,bytesize = 8,parity='N', stopbits=1, timeout=None,  write_timeout=None, xonxoff=False, rtscts=False, dsrdtr=False )
        
        
        communication=ser
        print('Carte communication détectée')
        
    except:
        communication=None
        print('Pas de carte communication détectée')
        
    #%% Connexion à la vision
    
    vision = psl.SSLVisionClient(ip='224.5.23.2', port=10020)
    vision.connect3()
    
    #%% Connexion simulateur
    
    simulateur = True
    
    
    if simulateur:
        if os.name=='nt':
            grSim = psl.SSLgrSimClient('192.168.1.11', 20011)
            # grSim = psl.SSLgrSimClient('192.168.56.1', 20011)
            
            
        elif os.name=='posix':
            grSim = psl.SSLgrSimClient('127.0.0.1', 20011)
        grSim.connect()
        sim = grSim
        print('Connexion au simulateur OK')
    else:
        print('Sans grSim')
        sim = None
     
        
     
   
        
   #%%Création match

    '''parametres :
       -disp :
           -1 : aucun affichage
            0 : affichage dans la console des changements de postes
            1 : affichage dans un plot des status des robots
            2 : affichage dans un plot du terrain avec les robots et leur status
         
        -controlledTeams:
            'B' to control blue
            'Y' to control yellow
            'BY' to control both 
        
        -blueSide:
            'L' if blue plays on the left
            'R' if blue plays on the right
            
        -start:
            'B' if blue engages first
            'Y' if yellow engages first
            
            '''
         
    
        
    match_test = match.Match('test', vision, sim, communication, disp=0, controlledTeams='BY', blueSide='L', start='B')
    match_test.engagement=manette
    
    
    if match_test.disp>0:
        fig,ax,axbackground,text,score = affichage.init(match_test.disp)
        t_list = [time.time()]

    #%%Boucle
    while not quit:
   
        if match_test.disp>0:
            fig.canvas.restore_region(axbackground)  
        
        #Lecture des commandes depuis la manette
        if manette :
            quit = m.refresh(match_test)
            
        
        if match_test.stop:
            match_test.Reset()
            
            
        elif match_test.engagement:
            match_test.Engagement()
            

        else: 
            #MATCH 2V2
            match_test.Play()
            
            
            # #Actualisation des positions + detection but 
            # match_test.Vision()
            
            # #Controle des bleus
            # match_test.blue.changementDePoste()
            # match_test.blue.action()
            
            # #Controle des jaunes
            # match_test.yellow.changementDePoste()
            # match_test.yellow.action()
            
            # # #Controle des jaunes
            # match_test.yellow.joueurs[0].defPoste('DEF1')
            # match_test.yellow.joueurs[1].defPoste('GOAL')
            # match_test.yellow.action()
            
          
        
        if match_test.disp>0:
            affichage.refresh(match_test,ax,score) 
            
            t_list = affichage.t_update(t_list)
            tx = 'Mean Frame Rate:\n {fps:.3f}FPS'.format(fps= (len(t_list) / (t_list[-1] - t_list[0]) )) 
            # print(tx)     
            text.set_text(tx)
            ax.draw_artist(text)
            
            #actualisation du plot
            fig.canvas.blit(ax.bbox)
            fig.canvas.flush_events()
        

        
        
    #%%Arrêt du programme
    
    for joueur in match_test.joueurs:
        joueur.commande_robot(0,0,0)
    
    #ferme la fenetre pyplot
    plt.close("all")
    
    #deconnexion de la vision
    vision.close()
    
    #deconnexion du simulateur
    if simulateur:
        grSim.close()
    
    #deconnexion de la communication
    if communication!=None:
        ser.close()
    