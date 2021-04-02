# -*- coding: utf-8 -*-
"""
Created on Fri Apr  2 10:29:46 2021

@author: paulg
"""
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 27 13:59:23 2021

@author: psl
"""
import pygame

# Labels for DS4 controller axes

#%%WINDOWS
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
BUTTON_L2 = 5
BUTTON_R2 = 14

BUTTON_SHARE = 4
BUTTON_OPTIONS = 6

BUTTON_LEFT_STICK = 7
BUTTON_RIGHT_STICK = 8

BUTTON_PS = 12
BUTTON_PAD = 13

# Labels for DS4 controller hats (Only one hat control)
HAT_1 = 0


#initialisation de pygame et de la manette
def init():
    global axis
    global button
    global hat
    global r1
    global l1
    global opt
    global stop
    global go
    
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
        
    r1=False
    l1=False
    opt=False
    stop=False
    go=False
    
    
    
    
    
    
    
#lecture des commandes de la manette
def refresh(match):
    global axis
    global button
    global hat
    
    global r1
    global l1
    global opt
    global stop
    global go
    
    #lecture des entrées de la manette
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
    quit = button[BUTTON_CIRCLE]
    
    #carré pour stoper le match
    if (button[BUTTON_SQUARE]) & (not(stop)):
        match.Stop()
    stop=  button[BUTTON_SQUARE] 
    
    #croix pour reprendre le match
    if (button[BUTTON_CROSS]) & (not go):
        match.Go()
    go=button[BUTTON_CROSS]   
    
    #R1 pour but jaune
    if (not (r1 ))& (button[BUTTON_R1]):
        match.but_jaune()
    r1=button[BUTTON_R1]
    
    #L1 pour but bleu
    if( not (l1) )& (button[BUTTON_L1]):
        match.but_bleu()
    l1=button[BUTTON_L1]
    
    #option pour rejouer un match
    if( not (opt) )& (button[BUTTON_OPTIONS]):
        match.regame()
    opt=button[BUTTON_OPTIONS]
    
    return quit