# -*- coding: utf-8 -*-
"""
Created on Wed Dec 11 18:14:16 2019

@author: C. Chauffaut
"""

#import run_planning
import mdptoolbox
import run_planning_model_generation as model

import math

## Solving MDP problem
# instanciate solver class with model and solving parameters
vi = mdptoolbox.mdp.ValueIteration(model.transFunction, model.rewFunction, 0.95, 0.01, 100)
# run solver
vi.run()
# print policy vector
#print(vi.policy)

policy = vi.policy


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
    
    xmin = - model.dy/2
    xmax =   model.dy/2
    ymin = - model.dx/2
    ymax =   model.dx/2
    
    if x > xmax:
        x = xmax - 1e-6;
    elif x < xmin:
        x = xmin + 1e-6;
    
    if y > ymax:
        y = ymax - 1e-6;
    elif y < ymin:
        y = ymin + 1e-6;
    
    x_mdp = math.floor(-y) + model.dx/2;
    y_mdp = math.floor(x) + model.dy/2;
    
    index = model.xy2ind(x_mdp, y_mdp, model.dx, model.dy)
    
#    print('room2ind: '+ str(index)+', x,y: '+str(x)+', '+str(y)
#    +', mdp x,y: '+str(x_mdp)+', '+str(y_mdp))

    return index

def index2xyroom(ind):
    # bounds [xmin xmax ymin ymax]

    xmin = - model.dy/2
    # xmax =   model.dy/2
    ymin = - model.dx/2
#    ymax =   model.dx/2
    
    x_mdp, y_mdp = model.ind2xy(ind, model.dx, model.dy)    
    
    x = xmin + y_mdp + 0.5
    y = - (ymin + x_mdp + 0.5)
    
#    print('id2room: '+ str(ind)+', x,y: '+str(x)+', '+str(y)
#    +', mdp x,y: '+str(x_mdp)+', '+str(y_mdp))

    return [x, y]

def mapForTargetStates(index):
        if len(model.mapForStates[index]) == 3 :
            return model.mapForStates[index][2];
        else:
            return -1;


def compute_mdp_state(x_robot, y_robot, x_tgt, y_tgt):
    # computing grid indexes for drone and target
    index_target = xyroom2index(x_tgt, y_tgt);
    index_robot = xyroom2index(x_robot, y_robot);

    # if target not in a obstacle, computing the symbolic MDP state
    if mapForTargetStates(index_target)!= -1 and mapForTargetStates(index_robot) != -1 :
        state = (mapForTargetStates(index_robot))*model.nts + mapForTargetStates(index_target)
        return state
    else:
        print('problem with index_target : target in an obstacle zone')

def computing_next_waypoint(x_robot, y_robot, action):
    index_robot = xyroom2index(x_robot, y_robot);
    [X_robot, Y_robot] = index2xyroom(index_robot);
    
    # bounds [xmin xmax ymin ymax]
    xmin = - model.dy/2
    xmax =   model.dy/2
    ymin = - model.dx/2
    ymax =   model.dx/2    
    
    X_d = X_robot;
    Y_d = Y_robot;
    
    if action == 0:
        # going north
        Y_d = Y_robot;
        if (X_robot < (xmax-0.5)):
            X_d = X_robot + 1; 
        
    elif action == 1:
        # going south
        Y_d = Y_robot;
        if (X_robot > (xmin+0.5)):
            X_d = X_robot - 1; 
        
    elif action == 2:
        # going east
        X_d = X_robot;
        if (Y_robot > (ymin+0.5)):
            Y_d = Y_robot - 1;  
        
    elif action == 3:
        # going west
        X_d = X_robot;
        if (Y_robot < (ymax-0.5)):
            Y_d = Y_robot + 1;
        
    elif action == 4:
        # goind northeast
        if (X_robot < (xmax-0.5)):
            X_d = X_robot + 1; 
        if (Y_robot > (ymin-0.5)):
            Y_d = Y_robot - 1;  
        
    elif action == 5:
        # going southeast
        if (X_robot > (xmin+0.5)):
            X_d = X_robot - 1; 
        if (Y_robot > (ymin+0.5)):
            Y_d = Y_robot - 1; 
        
        
    elif action == 6:
        # going northwest
        if (X_robot < (xmax-0.5)):
            X_d = X_robot + 1; 
        if (Y_robot < (ymax-0.5)):
            Y_d = Y_robot + 1;
        
    elif action == 7:
        # going southwest
        if (X_robot > (xmin+0.5)):
            X_d = X_robot - 1; 
        if (Y_robot < (ymax-0.5)):
            Y_d = Y_robot + 1; 
        
    elif action == 8:
        
        X_d = X_robot;
        Y_d = Y_robot;
        
    else:
        print('ERROR: action not corresponds to the planning model')

    next_waypoint = [X_d, Y_d];
    return next_waypoint
