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
K_max=0.8 #facteur prop vitesse
seuil_distance=250
sat_vitesse_angulaire=6
K_angulaire=5
sat_orientation_balle=3
K_orientation_balle=10

saturation_vitesse_tangente=0.9
saturation_vitesse_normale=0.85

##REPULSION
nbPoints=80
r_evit=120
k=4 #facteur de repulsion

R_non_evitement=360
K_proche=0.3

X = np.linspace(xmin,xmax,nbPoints)
Y = np.linspace(ymin,ymax,nbPoints)
x_grid,y_grid = np.meshgrid(X,Y)

#Création du champ répulsif des surfaces
surface_x=np.zeros((80,80))
surface_y=np.zeros((80,80))
for i in range(int(350/2700*80)):
    for j in range(int(650/2000*80),int(1350/2000*80)):
        surface_x[j,i]=1
        surface_x[j,79-i]=-1
        surface_y[j,i]=1*np.sign(j-40)*(int(350/2700*80)-i-1)
        surface_y[j,79-i]=1*np.sign(j-40)*(int(350/2700*80)-i-1)