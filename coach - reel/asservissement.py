#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  7 11:46:00 2021

@author: psl
"""

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
    
    # vision = psl.SSLVisionClient(ip='224.5.23.2', port=10020)
    vision = psl.SSLVisionClient(ip='224.5.23.2', port=10006)
    vision.connect3()
    
    #%% Connexion simulateur
    
    simulateur = False
    
    
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

    '''parametre disp :
        -1 : aucun affichage
         0 : affichage dans la console des changements de postes
         1 : affichage dans un plot des status des robots
         2 : affichage dans un plot du terrain avec les robots et leur status'''
        
    match_test = match.Match('test', vision, sim, communication, disp=2, blueSide='R', start='B')
    match_test.engagement=False
    
    
    if match_test.disp>0:
        fig,ax,axbackground,text,score = affichage.init(match_test.disp)
        t_list = [time.time()]

    #%%Boucle
    while not quit:
        
        start=time.time()
        #test interruption crtl+C pour arrêter tous les robots
        try:
            
            
            if match_test.disp>0:
                fig.canvas.restore_region(axbackground)  
            
            #Lecture des commandes depuis la manette
            if manette :
                quit = m.refresh(match_test)
                
            #controle du robot à la manette
            if match_test.stop:
                match_test.Vision()
                Vnorm,Vtang,Vang=m.controle()
                print(Vtang)
                match_test.yellow.joueurs[0].commande_robot(Vtang, Vnorm, Vang)
                
            
                
                
                
            else: 
                
                
                #Actualisation des positions + detection but 
                match_test.Vision()
                
                
                #asservissement du robot en position ou chaser
                # match_test.blue.joueurs[0].commande_balle() 
                # match_test.yellow.joueurs[0].commande_position(500,500,0,0)
                # match_test.blue.joueurs[0].commande_robot(0, -0.1, 0)
                balle=match_test.balle
                # match_test.yellow.joueurs[0].commande_position(match_test.yellow.joueurs[0].x,match_test.yellow.joueurs[0].y,balle.x,balle.y)
                match_test.yellow.joueurs[0].commande_position(balle.x,balle.y,balle.x,balle.y)
                
                # print(match_test.yellow.joueurs[0].orientation)
                
            
            
           
              
            
            if match_test.disp>0:
                affichage.refresh(match_test,ax,score) 
                
                #affichage des FPS
                t_list = affichage.t_update(t_list)
                tx = 'Mean Frame Rate:\n {fps:.3f}FPS'.format(fps= (len(t_list) / (t_list[-1] - t_list[0]) )) 
                # print(tx)     
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
        
        # print(1/(time.time()-start))
    #%%Arrêt du programme
    
    for joueur in match_test.joueurs:
        joueur.commande_robot(0,0,0)
        if joueur.com:
            # start=time.time()
            joueur.commande_com(joueur.id,0,0,0,0,0)
        time.sleep(0.05)
        # joueur.commande_robot(0,0,0)
        # joueur.commande_robot(0,0,0)
        # joueur.commande_robot(0,0,0)
    
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
    