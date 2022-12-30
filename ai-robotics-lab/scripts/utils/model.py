# -*- coding: utf-8 -*-
"""
Created on Wed Dec 11 18:14:16 2019

@author: C. Chauffaut
"""

import math
dx = 8
dy = 6


def xy2ind(x, y, dx, dy):
    index = (y)*dx + x;
    return index

def ind2xy(ind, dx, dy):
    x = ind%dx
    if x==0:
        x = 0; 
        y = math.floor(ind/dx);
    else:    
        y = math.floor(ind/dx);
    return x,y


def xyroom2index(x, y):

    """
    In the MDP model:
        - the (0,0) is in the left bottom corner
        - positive x is toward the right
        - positive y is toward up
    In the room/simulation:
        - the (0,0) is in the middle of the room
        - positive x is toward up
        - positive y is toward left
    """
    
    xmin = - dy/2
    xmax =   dy/2
    ymin = - dx/2
    ymax =   dx/2
    
    if x > xmax:
        x = xmax - 1e-6;
    elif x < xmin:
        x = xmin + 1e-6;
    
    if y > ymax:
        y = ymax - 1e-6;
    elif y < ymin:
        y = ymin + 1e-6;
    
    x_mdp = math.floor(-y) + dx/2;
    y_mdp = math.floor(x) + dy/2;
    
    index = xy2ind(x_mdp, y_mdp, dx, dy)
    
#    print('room2ind: '+ str(index)+', x,y: '+str(x)+', '+str(y)
#    +', mdp x,y: '+str(x_mdp)+', '+str(y_mdp))

    return index

def index2xyroom(ind):
    # bounds [xmin xmax ymin ymax]

    xmin = - dy/2
    # xmax =   model.dy/2
    ymin = - dx/2
#    ymax =   model.dx/2
    
    x_mdp, y_mdp = ind2xy(ind, dx, dy)    
    
    x = xmin + y_mdp + 0.5
    y = - (ymin + x_mdp + 0.5)
    
#    print('id2room: '+ str(ind)+', x,y: '+str(x)+', '+str(y)
#    +', mdp x,y: '+str(x_mdp)+', '+str(y_mdp))

    return [x, y]
