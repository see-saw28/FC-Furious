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
#%% Librairies

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as lines
import pygame
import serial
import serial.tools.list_ports
import time

from psl_package import paris_saclay_league as psl


import parametres as p
import match as match







if __name__ == "__main__":

    
    #%% Test manette
    try:
        pygame.init()
        pygame.joystick.init()
        
        controller = pygame.joystick.Joystick(0)
        controller.init()
        
        # Three types of controls: axis, button, and hat
        axis = {}
        button = {}
        hat = {}
        
        # Assign initial data values
        # Axes are initialized to 0.0
        for i in range(controller.get_numaxes()):
        	axis[i] = 0.0
        # Buttons are initialized to False
        for i in range(controller.get_numbuttons()):
        	button[i] = False
        # Hats are initialized to 0
        for i in range(controller.get_numhats()):
        	hat[i] = (0, 0)
        
        # Labels for DS4 controller axes
        AXIS_LEFT_STICK_X = 0
        AXIS_LEFT_STICK_Y = 1
        AXIS_RIGHT_STICK_X = 4
        AXIS_RIGHT_STICK_Y = 3
        AXIS_R2 = 5
        AXIS_L2 = 4
        
        # Labels for DS4 controller buttons
        # Note that there are 14 buttons (0 to 13 for pygame, 1 to 14 for Windows setup)
        BUTTON_SQUARE = 3
        BUTTON_CROSS = 0
        BUTTON_CIRCLE = 1
        BUTTON_TRIANGLE = 2
        
        BUTTON_L1 = 4
        BUTTON_R1 = 5
        BUTTON_L2 = 6
        BUTTON_R2 = 7
        
        BUTTON_SHARE = 8
        BUTTON_OPTIONS = 9
        
        BUTTON_LEFT_STICK = 10
        BUTTON_RIGHT_STICK = 11
        
        BUTTON_PS = 12
        BUTTON_PAD = 5
        
        # Labels for DS4 controller hats (Only one hat control)
        HAT_1 = 0
        
        
        manette=True
        print('Manette détectée')
        
    except:
        manette=False
        print('Pas de manette détectée')
        
        
    # Main loop, one can press the PS button to break
    quit = False
    
    #%% Test communication
    try :
        #Liste des ports
        # print(list(serial.tools.list_ports.comports()))
        
        #LINUX
        ser = serial.Serial("/dev/ttyACM0",baudrate=115200,bytesize = 8, parity='N', stopbits=1, timeout=None,  write_timeout=None, xonxoff=False, rtscts=False, dsrdtr=False )  # open first serial port
        
        #WINDOWS
        # ser = serial.Serial("COM6",baudrate=115200,bytesize = 8,parity='N', stopbits=1, timeout=None,  write_timeout=None, xonxoff=False, rtscts=False, dsrdtr=False )
        
        
        communication=ser
        print('Carte communication détectée')
        
    except:
        communication=None
        print('Pas de carte communication détectée')
        
    #%% Connexion à la vision
    
    vision = psl.SSLVisionClient(ip='224.5.23.2', port=10020)
    vision.connect3()
    
    #%% Connexion simulateur
    simulateur=True
    if simulateur:
        grSim = psl.SSLgrSimClient('127.0.0.1', 20011)
        grSim.connect()
        sim=grSim
        print('Connexion au simulateur OK')
    else:
        print('Sans grSim')
        sim=None
     
        
     
   
        
#%%Création match


    match_test=match.Match('test',vision,sim,communication,disp=-1)
    
    if match_test.disp==2:
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.set_xlim(-p.longueur,p.longueur)
        ax.set_ylim(-p.largeur,p.largeur)
        ax.add_artist(lines.Line2D((-p.longueur, -p.longueur+350, -p.longueur+350,-p.longueur), (-350, -350,350,350), color = 'green'))
        ax.add_artist(lines.Line2D((p.longueur, p.longueur-350, p.longueur-350,p.longueur), (-350, -350,350,350), color = 'green'))
        ax.add_artist(lines.Line2D((0,0), (p.largeur,-p.largeur), color = 'green'))
        ax.add_artist(patches.Circle((0, 0), 500,facecolor='None', edgecolor = 'green'))
        ax.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left',
                    ncol=2, mode="expand", borderaxespad=0.)
        text=ax.text(0,p.largeur-200,"")
        fig.canvas.draw()
        axbackground = fig.canvas.copy_from_bbox(ax.bbox)
        
        
    
    
    elif match_test.disp==1:
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)
        ax.set_xlim(-p.longueur,p.longueur)
        ax.set_ylim(-p.largeur,p.largeur)
        text=ax.text(0,p.largeur-200,"")
        axbackground = fig.canvas.copy_from_bbox(ax.bbox)
        fig.canvas.draw()
        frame =0
        axButton1=plt.axes([0.25,0.45,0.1,0.1])
        btn1=plt.Button(axButton1,label='Quebec',color='lightgrey',hovercolor='greenyellow')#definiton du bouton(position par rapport à la figure "ax",label,couleur, couleur lorsque la souris passe au dessus)
        btn1.label.set_fontsize(20)     #modification de la taille du label
                 
        
    t_list = [time.time()]
    def t_update(t_list):
        t_list.append(time.time())
        if len(t_list)>30:
            t_list.pop(0)
        return(t_list)
    
    r1=False
    l1=False
    opt=False
    stop=False
    go=False
    quit=False
    freq=[0 for i in range (100)]

    while not quit:
        
        try:
            if match_test.disp>0:
                
                fig.canvas.restore_region(axbackground)  
                
                
            if manette :
                
                for event in pygame.event.get():
        
                    if event.type == pygame.JOYAXISMOTION:
                        axis[event.axis] = round(event.value,3)
                    elif event.type == pygame.JOYBUTTONDOWN:
                        button[event.button] = True
                    elif event.type == pygame.JOYBUTTONUP:
                        button[event.button] = False
                    elif event.type == pygame.JOYHATMOTION:
                    	hat[event.hat] = event.value
                
                #fleche du bas pour arreter le programme
                quit = hat[HAT_1][1]==-1
                
                #carré pour stoper le match
                if (button[BUTTON_SQUARE]) & (not(stop)):
                    match_test.Stop()
                stop=  button[BUTTON_SQUARE] 
                
                #croix pour reprendre le match
                if (button[BUTTON_CROSS]) & (not go):
                    match_test.Go()
                go=button[BUTTON_CROSS]   
                
                #R1 pour but jaune
                if (not (r1 ))& (button[BUTTON_R1]):
                    match_test.but_jaune()
                r1=button[BUTTON_R1]
                
                #L1 pour but bleu
                if( not (l1) )& (button[BUTTON_L1]):
                    match_test.but_bleu()
                l1=button[BUTTON_L1]
                
                #option pour rejouer un match
                if( not (opt) )& (button[BUTTON_OPTIONS]):
                    match_test.regame()
                opt=button[BUTTON_OPTIONS]
                
            
            if match_test.stop:
                match_test.Vision()
                match_test.blue.reset()
                match_test.yellow.reset()
                
            else: 
                
                #Match 2v2
                match_test.Vision()
                match_test.blue.changementDePoste()
                match_test.blue.action()
                match_test.yellow.changementDePoste()
                match_test.yellow.action()
                
                
            
                
               
            if match_test.disp==2:
                for joueur in match_test.joueurs:
                    goto,=ax.plot(joueur.goto.real,joueur.goto.imag,'go')
                    ax.draw_artist(goto)
                    if joueur.team=='Y':
                        tag='gold'
                        
                    else :
                        tag='b'
           
                    draw_joueur,=ax.plot(joueur.x,joueur.y,tag,marker='o',ms=20,label=(str(joueur)+' '+str(joueur.poste[-1])+' '+str(joueur.status)))
                    text_joueur=ax.text(joueur.x,joueur.y,joueur.id,horizontalalignment='center',verticalalignment='center',color='w')
                    text2_joueur=ax.text(joueur.x,joueur.y-100,str(joueur.poste[-1])+' '+str(joueur.status),horizontalalignment='center',verticalalignment='center',color='black')
                    ax.draw_artist(draw_joueur)
                    ax.draw_artist(text_joueur)
                    ax.draw_artist(text2_joueur)
                    
                    draw_ball,=ax.plot(match_test.balle.x10,match_test.balle.y10,'ro')
                    ax.draw_artist(draw_ball)
                    
                    if joueur.poste[-1]=='RECEVEUR':
                        goto,=ax.plot(joueur.goto.real, joueur.goto.imag,'mo')
                        ax.draw_artist(goto)
            
            elif match_test.disp==1:
                for joueur in match_test.joueurs:
                    if joueur.team=='B':
                        x=-700
                    else :
                        x=700
                    draw_robot=ax.text(x,joueur.id*200,str(joueur)+' '+str(joueur.poste[-1])+' '+str(joueur.status),horizontalalignment='center',verticalalignment='center')
                    ax.draw_artist(draw_robot)
                
            if match_test.disp>0:
                t_list=t_update(t_list)
                tx = 'Mean Frame Rate:\n {fps:.3f}FPS'.format(fps= (len(t_list) / (t_list[-1] - t_list[0]) )) 
                text.set_text(tx)
                ax.draw_artist(text)
                
                fig.canvas.blit(ax.bbox)
                fig.canvas.flush_events()
            
            
                
                
        except KeyboardInterrupt :#mise à zéro de tous les robots lors de l'intérruption du programme
            print('INTERRUPTION')
            for joueur in match_test.joueurs:
                joueur.commande_robot(0,0,0)
            break
    
    
    for joueur in match_test.joueurs:
        joueur.commande_robot(0,0,0)
    
    #ferme la fenetre pyplot
    plt.close("all")
    
    
    vision.close()
    
    if simulateur!=None:
     grSim.close()
     
    if communication!=None:
        ser.close()
    