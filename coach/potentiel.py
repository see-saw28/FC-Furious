# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy import linspace, meshgrid, sqrt, gradient,exp
# from matplotlib.pyplot import quiver


# DÃ©finition du potentiel Ã©lectrostatique crÃ©Ã© par une charge q situÃ©e en A
def Potentiel(q,A,x,y,sigma):    
    delta=(A[0]-x)**2+(A[1]-y)**2
    V =q* exp(-delta/sigma**2)
    return V
    
# DÃ©finition de la grille de calcul



def Gradient(potentiel):

    Ey,Ex = gradient(potentiel)
    Ex = -Ex
    Ey = -Ey
    return Ex,Ey

def norme(Vx,Vy):
    return(Vx/sqrt(Vx**2+Vy**2),Vy/sqrt(Vx**2+Vy**2))

def Grad(Ex,Ey,x,y,xbot,xball,a,b):
    if (xball-xbot)<0:
        Ex,Ey=-Ex,-Ey
    if (a*x+b)>y:
        Ex,Ey=Ey,-Ex
    else :
        Ex,Ey=-Ey,Ex
    
    return Ex,Ey

# fig = plt.figure()

# ax = fig.add_subplot(1, 2, 2)
# X = linspace(-1,1,30)
# Y = linspace(-1,1,30)
# x,y = meshgrid(X,Y)
# Ey,Ex=Gradient(Potentiel(2,[0,0],x,y,0.1))
# plt.quiver(x,y,Ex,-Ey)
