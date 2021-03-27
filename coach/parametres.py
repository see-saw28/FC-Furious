#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar 18 17:45:59 2021

@author: psl
"""

import numpy as np

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
sat_vitesse_angulaire=6
K_angulaire=3
sat_orientation_balle=3
K_orientation_balle=10

saturation_vitesse_tangente=0.9
saturation_vitesse_normale=0.85

##REPULSION
nbPoints=150
r_evit=120
k=4 #facteur de repulsion

R_non_evitement=360
K_proche=0.3

X = np.linspace(xmin,xmax,nbPoints)
Y = np.linspace(ymin,ymax,nbPoints)
x_grid,y_grid = np.meshgrid(X,Y)

#Création du champ répulsif des surfaces
surface_x=np.zeros((nbPoints,nbPoints))
surface_y=np.zeros((nbPoints,nbPoints))
for i in range(int(350/2700*nbPoints)):
    for j in range(int(650/2000*nbPoints),int(1350/2000*nbPoints)):
        surface_x[j,i]=1
        surface_x[j,nbPoints-1-i]=-1
        surface_y[j,i]=1*np.sign(j-int(nbPoints/2))*(int(350/2700*nbPoints)-i-1)
        surface_y[j,nbPoints-1-i]=1*np.sign(j-int(nbPoints/2))*(int(350/2700*nbPoints)-i-1)