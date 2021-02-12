#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Feb  5 18:11:20 2021

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
import pygame
import os

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

# Main loop, one can press the PS button to break
quit = False



vision = psl.SSLVisionClient(ip='224.5.23.2', port=10020)
vision.connect()
grSim = psl.SSLgrSimClient('127.0.0.1', 20011)
grSim.connect()

idr=0
spin=False
cross=False
R1=False

while not quit:

    # Get events
    for event in pygame.event.get():

        if event.type == pygame.JOYAXISMOTION:
            axis[event.axis] = round(event.value,3)
        elif event.type == pygame.JOYBUTTONDOWN:
            button[event.button] = True
        elif event.type == pygame.JOYBUTTONUP:
            button[event.button] = False
        elif event.type == pygame.JOYHATMOTION:
        	hat[event.hat] = event.value

    quit = button[BUTTON_PS]

    # Print out results
    # os.system('cls')
    # Axes
    vn=-axis[AXIS_LEFT_STICK_X]
    vt=-axis[AXIS_LEFT_STICK_Y]
    va=-axis[AXIS_RIGHT_STICK_Y]
  
    print(vt,vn,va,idr)
    tir=0
    
    
    if button[BUTTON_SQUARE]:
        tir=10
    
    if (cross!=button[BUTTON_CROSS])&(cross==False):
        spin=not spin
    cross=button[BUTTON_CROSS]
    
    if (R1!=button[BUTTON_R1])&(R1==False):
        print('r1')
        if idr==0:
            idr=1
        else:
            idr=0
            
    R1=button[BUTTON_R1]
    
    p = psl.packetCommandBot(False, 
                          idr, 
                          veltangent=0.5*vt, 
                          velnormal=0.5*vn, 
                          velangular=3*va,
                          spinner=spin,
                          kickspeedx=tir)
    grSim.send(p)
    
   
    time.sleep(0.05)

# ani=animation.FuncAnimation(fig,animate,interval=30)
# plt.show()
# match_test.joueurs[2].commande_balle()
# match_test.joueurs[2].Passe()
# time.sleep(0.3)
# match_test.joueurs[3].Tir()
# match_test.joueurs[2].Tir()
# match_test.balle.Position()
# print(match_test.balle.position)

# Y0=match_test.joueurs[0]
# position_arrivee=Balle()
# position_arrivee.x=-500
# position_arrivee.y=500
# Exb,Eyb=pt.Gradient(position_arrivee.potentiel)
# a,b=np.polyfit([-100,Y0.x],[10,Y0.y],1)
# Y0.champ_autre(Y0.x,0,a,b)
# Ex,Ey=Exb+Y0.Ex_autre,Eyb+Y0.Ey_autre
# Ex,Ey=pt.norme(Ex,Ey)
# fig = plt.figure()
# ax = fig.add_subplot(1, 1, 1)
# ax.quiver(x,y,Ex,Ey,width=0.0008)
# Y0.commande_position(500, 500, 0)
# #Affichage
# Exb,Eyb=pt.Gradient(potentielBalle)
# # Exb,Eyb=norme(Exb,Eyb)
# Exe=Ex1+Ex2+Ex3
# Eye=Ey1+Ey2+Ey3
# # Exe,Eye=norme(Exe,Eye)
# Ex,Ey=Exb+Exe,Eyb+Eye
# Ex,Ey=norme(Ex,Ey)

# for bot in blueBots:
#     if bot[5]==0:
#             botInfo=bot
# xbot,ybot,obot=botInfo[6],botInfo[7],botInfo[8]
# ax.plot(xbot,ybot,'ro')
# ax.plot(xbot1,ybot1,'ro',color='blue')
# ax.plot([xbot2,xbot3],[ybot2,ybot3],'ro',color='yellow')
# ax.quiver(x,y,Ex,Ey,width=0.0008)

vision.close()
grSim.close()