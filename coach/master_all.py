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
            
        elif os.name=='posix':
            grSim = psl.SSLgrSimClient('127.0.0.1', 20011)
        grSim.connect()
        sim = grSim
        print('Connexion au simulateur OK')
    else:
        print('Sans grSim')
        sim = None
     
        
     
   
        
   #%%Création match

    '''parametre disp :
        -1 : aucun affichage
         0 : affichage dans la console des changements de postes
         1 : affichage dans un plot des status des robots
         2 : affichage dans un plot du terrain avec les robots et leur status'''
        
    match_test = match.Match('test',vision,sim,communication,blueSide='R',start='B',disp=2)
    
    
    
    
    fig,ax,axbackground,text = affichage.init(match_test.disp)
    t_list = [time.time()]

    #%%Boucle
    while not quit:
        
        #test interruption crtl+C pour arrêter tous les robots
        try:
            
            if match_test.disp>0:
                fig.canvas.restore_region(axbackground)  
            
            #Lecture des commandes deouis la manette
            if manette :
                quit = m.refresh(match_test)
                
            
            if match_test.stop:
                match_test.Vision()
                match_test.blue.reset()
                match_test.yellow.reset()
                
            elif match_test.engagement:
                match_test.Vision()
                match_test.blue.engagement()
                match_test.yellow.engagement()
                
                
                
            else: 
                #MATCH 2V2
                
                #Actualisation des positions
                
                match_test.Vision()
                
                #Controle des bleus
                match_test.blue.changementDePoste()
                match_test.blue.action()
                
                #Controle des jaunes
                match_test.yellow.changementDePoste()
                match_test.yellow.action()
                
                
            
                
            affichage.refresh(match_test,ax)   
            
            if match_test.disp>0:
                #affichage des FPS
                t_list = affichage.t_update(t_list)
                tx = 'Mean Frame Rate:\n {fps:.3f}FPS'.format(fps= (len(t_list) / (t_list[-1] - t_list[0]) )) 
                text.set_text(tx)
                ax.draw_artist(text)
                
                #actualisation du plot
                fig.canvas.blit(ax.bbox)
                fig.canvas.flush_events()
            
            
                
                
        except KeyboardInterrupt :#mise à zéro de tous les robots lors de l'interruption du programme
            print('INTERRUPTION')
            # for joueur in match_test.joueurs:
            #     joueur.commande_robot(0,0,0)
            break
        
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
    