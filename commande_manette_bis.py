# -*- coding: utf-8 -*-
"""
Created on Thu May 27 16:15:06 2021

@author: paulg
"""


# -*- coding: utf-8 -*-
"""
Created on Tue Mar  9 16:44:40 2021

@author: paulg
"""





import time
import pygame
import serial

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
AXIS_RIGHT_STICK_X = 2
AXIS_RIGHT_STICK_Y = 3
AXIS_R2 = 5
AXIS_L2 = 4

# Labels for DS4 controller buttons
# Note that there are 14 buttons (0 to 13 for pygame, 1 to 14 for Windows setup)
BUTTON_SQUARE = 2
BUTTON_CROSS = 0
BUTTON_CIRCLE = 1
BUTTON_TRIANGLE = 3

BUTTON_L1 = 9
BUTTON_R1 = 10


BUTTON_SHARE = 4
BUTTON_OPTIONS = 6

BUTTON_LEFT_STICK = 7
BUTTON_RIGHT_STICK = 8

BUTTON_PS = 5
BUTTON_PAD = 13

BUTTON_UP = 11
BUTTON_DOWN = 12
BUTTON_LEFT = 13
BUTTON_RIGHT = 14

# Main loop, one can press the PS button to break
quit = False




def commande(ID,Vtan,Vnorm,Vrot,spinner,tir):
    chaine_commande = "D"+str(ID) + ","
    liste_vitesses = [Vtan,Vnorm,Vrot]
    spin=0
    if spinner:
        spin=1
        
    for vitesse in liste_vitesses :
        vitesse = int(vitesse*999) #Régler ici si la vitesse max n'est pas 1
        
        if abs(vitesse) < 27 : #Zone morte manette
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

    # for charactere in chaine_commande:
    #         ser.write(str.encode(charactere))
    return chaine_commande

idr=1
tir=0
spin=False
cross=False
R1=False
VN=0
VT=0
up=False
down=False
left=False
right=False
# ser = serial.Serial("COM6",baudrate=115200,bytesize = 8,parity='N', stopbits=1, timeout=None,  write_timeout=None, xonxoff=False, rtscts=False, dsrdtr=False )  # open first serial port

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


    
    #VITESSE NORMALE
    vn=-axis[AXIS_LEFT_STICK_X]
    
    #VITESSE TANGENTE
    vt=-axis[AXIS_LEFT_STICK_Y]
    
    #VITESSE ANGULAIRE
    va=axis[AXIS_RIGHT_STICK_X]
  
    
    if (up!=button[BUTTON_UP])&(up==False):
        if VT<=0.8:
            VT+=0.2
    up=button[BUTTON_UP]
    
    if (down!=button[BUTTON_DOWN])&(down==False):
        if VT>=-0.8:
            VT-=0.2
    down=button[BUTTON_DOWN]
    
    if (left!=button[BUTTON_LEFT])&(left==False):
        if VN>=-0.8:
            VN-=0.2
    left=button[BUTTON_LEFT]
    
    if (right!=button[BUTTON_RIGHT])&(right==False):
        if VN<=0.8:
            VN+=0.2
    right=button[BUTTON_RIGHT]
    
    #TIR
    if button[BUTTON_SQUARE]:
        tir+=10
        
    else:
        tir=0
    
    
    #ACTIVATION SPINNER
    if (cross!=button[BUTTON_CROSS])&(cross==False):
        spin=not spin
    cross=button[BUTTON_CROSS]
    
    #GHANGEMENT ID ROBOT
    if (R1!=button[BUTTON_R1])&(R1==False):
        print('r1')
        if idr==0:
            idr=1
        else:
            idr=0
            
    R1=button[BUTTON_R1]
    
    vn=VN
    vt=VT
    va=0
    # vn=0.2
    # print('vt:',vt,'vn:',vn,'va:',va,'id',idr,'spin:',spin,'tir',tir)
    chaine=commande(idr,vt,vn,va,spin,tir)
    print(chaine)
    
   
    time.sleep(0.05)

commande(idr,0,0,0,0,0)

# ser.close()       