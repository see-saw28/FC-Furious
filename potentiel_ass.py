#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec 11 10:40:12 2020

@author: psl
"""
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import potentiel as pt
from scipy import *
# import asservissement as ass
from psl_package import paris_saclay_league as psl
import cmath as c
import time
import numpy as np

vision = psl.SSLVisionClient(ip='224.5.23.2', port=10020)
vision.connect()
grSim = psl.SSLgrSimClient('127.0.0.1', 20011)
grSim.connect()

def getVision():
    while not vision.isDataReady(): 
        continue
  
    return vision.getLocations()

longueur=1350
largeur=1000
xmin=-longueur
xmax=longueur
ymin=-largeur
ymax=largeur

nbPoints=60
K=0.8
X = linspace(xmin,xmax,nbPoints)
Y = linspace(ymin,ymax,nbPoints)
x,y = meshgrid(X,Y)

hx=2*longueur/nbPoints
hy=2*largeur/nbPoints
balls, blueBots, yellowBots = getVision()
ball=balls[0]
xball,yball=ball[6],ball[7]



def norme(Vx,Vy):
    return(Vx/sqrt(Vx**2+Vy**2),Vy/sqrt(Vx**2+Vy**2))

def Rotationnel(x,y,Fx,Fy):
    dFx = (diff(Fx,axis=0)/hy)[:,:-1]
    dFy = (diff(Fy,axis=1)/hx)[:-1,:]
    return dFy - dFx

potentielBalle=pt.Potentiel(-3,[xball,yball],x,y,2500)

for bot in blueBots:
    if bot[5]==0:
        botInfo=bot
xbot,ybot,obot=botInfo[6],botInfo[7],botInfo[8]

a,b=np.polyfit([xball,xbot],[yball,ybot],1)

def grad(Ex,Ey,x,y):
    if (xball-xbot)<0:
        Ex,Ey=-Ex,-Ey
    if (a*x+b)>y:
        Ex,Ey=Ey,-Ex
    else :
        Ex,Ey=-Ey,Ex
    
    return Ex,Ey

r_evit=120
k=2 #facteur de repulsion

for bot in blueBots:
    if bot[5]==1:
            botInfo=bot
xbot1,ybot1,obot=botInfo[6],botInfo[7],botInfo[8]
potentielBlue1=pt.Potentiel(+5,[xbot1,ybot1],x,y,r_evit)
Ex1,Ey1=pt.Gradient(potentielBlue1)
Ex10,Ey10=grad(Ex1,Ey1,xbot1,ybot1)

potentielBlue1=pt.Potentiel(+5,[xbot1,ybot1],x,y,r_evit/1.5)
Ex1,Ey1=pt.Gradient(potentielBlue1)
Ex1,Ey1=k*Ex1+Ex10,k*Ey1+Ey10

for bot in yellowBots:
    if bot[5]==1:
            botInfo=bot
xbot2,ybot2,obot=botInfo[6],botInfo[7],botInfo[8]
potentielYel1=pt.Potentiel(+5,[xbot2,ybot2],x,y,r_evit)
Ex2,Ey2=pt.Gradient(potentielYel1)
Ex20,Ey20=grad(Ex2,Ey2,xbot2,ybot2)
potentielYel1=pt.Potentiel(+5,[xbot2,ybot2],x,y,r_evit/1.5)
Ex2,Ey2=pt.Gradient(potentielYel1)
Ex2,Ey2=k*Ex2+Ex20,k*Ey2+Ey20

for bot in yellowBots:
    if bot[5]==0:
            botInfo=bot
xbot3,ybot3,obot=botInfo[6],botInfo[7],botInfo[8]
potentielYel0=pt.Potentiel(+5,[xbot3,ybot3],x,y,r_evit)
Ex3,Ey3=pt.Gradient(potentielYel0)
Ex30,Ey30=grad(Ex3,Ey3,xbot3,ybot3)
potentielYel2=pt.Potentiel(+5,[xbot3,ybot3],x,y,r_evit/1.5)
Ex3,Ey3=pt.Gradient(potentielYel2)
Ex3,Ey3=k*Ex3+Ex30,k*Ey3+Ey30

potentielEnnemi=potentielBlue1+potentielYel0+potentielYel1
potentiel=potentielBalle+potentielEnnemi
fig = plt.figure()


ax = fig.add_subplot(1, 2, 1, projection='3d')

ax.plot_wireframe(x, y, potentiel, color='black')




ax1 = fig.add_subplot(1, 2, 2)

Exb,Eyb=pt.Gradient(potentielBalle)
# Exb,Eyb=norme(Exb,Eyb)
Exe=Ex1+Ex2+Ex3
Eye=Ey1+Ey2+Ey3
# Exe,Eye=norme(Exe,Eye)
Ex,Ey=Exb+Exe,Eyb+Eye
Ex,Ey=norme(Ex,Ey)

for bot in blueBots:
    if bot[5]==0:
            botInfo=bot
xbot,ybot,obot=botInfo[6],botInfo[7],botInfo[8]
ax1.plot(xbot,ybot,'ro')
ax1.plot(xbot1,ybot1,'ro',color='blue')
ax1.plot([xbot2,xbot3],[ybot2,ybot3],'ro',color='yellow')
ax1.plot(xball,yball,'ro',color='green')
ax1.quiver(x,y,Ex,Ey,width=0.0008)
# Ex,Ey=(Ex/sqrt(Ex**2 + Ey**2),Ey/sqrt(Ex**2 + Ey**2))



def commande(x,y,o,Id):
    pos=complex(x,y)
    balls, blueBots, yellowBots = getVision()
    for bot in blueBots:
        if bot[5]==Id:
            botInfo=bot
    xbot,ybot,obot=botInfo[6],botInfo[7],botInfo[8]
    posbot=complex(xbot,ybot)
    vecteur=pos-posbot
    distance,phi=c.polar(vecteur)
    delta=o-obot
    print((abs(distance)>5)&(abs(delta)>0.01))
    while not(abs(distance)<35):
        xd=int((xbot+longueur)/(2*longueur)*nbPoints)
        yd=int((ybot+largeur)/(2*largeur)*nbPoints)
        # print(Ex[yd,xd],xd,xbot)
        p = psl.packetCommandBot(isYellow=False, 
                         id=Id, 
                         veltangent=K*(sin(obot)*Ey[yd,xd]+cos(obot)*Ex[yd,xd]), 
                         velnormal=K*(-sin(obot)*Ex[yd,xd]+cos(obot)*Ey[yd,xd]), 
                         velangular=delta)
        grSim.send(p)
        time.sleep(0.1)
        pos=complex(x,y)
        balls, blueBots, yellowBots = getVision()
        for bot in blueBots:
            if bot[5]==Id:
                botInfo=bot
        xbot,ybot,obot=botInfo[6],botInfo[7],botInfo[8]
        posbot=complex(xbot,ybot)
        vecteur=pos-posbot
        distance,phi=c.polar(vecteur)
        delta=o-obot
        
    p = psl.packetCommandBot(isYellow=False, 
                         id=Id, 
                         veltangent=0., 
                         velnormal=0, 
                         velangular=0.,
                         spinner=True)
    grSim.send(p)


# posball=complex(xball,yball)
# posbot=complex(xbot,ybot)
# vecteur=posball-posbot
# distance,angle=c.polar(vecteur)


# commande(xball,yball,angle,0)

vision.close()
grSim.close()
