#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 10 16:07:16 2019
Script that creates the MDP planning model
by C. CHANEL (ISAE-SUPAERO)
@author: cchanel
"""
import math
import numpy as np


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

def checkneighbors(x,y,dx,dy,dir1,dir2,mapForStates):
    flag = 1;
    if dir1=='n' and dir2=='e':
        nextn = xy2ind(x,y+1, dx, dy)
        nexte = xy2ind(x+1,y, dx, dy)
        if not len(mapForStates[nextn]) or not len(mapForStates[nexte]):
            flag = 0
    
    if dir1=='n'and dir2=='w':
        nextn = xy2ind(x,y+1, dx, dy)
        nextw = xy2ind(x-1,y, dx, dy)
        if not len(mapForStates[nextn]) or not len(mapForStates[nextw]):
            flag = 0

    if dir1=='s' and dir2=='e':
        nexts = xy2ind(x,y-1, dx, dy)
        nexte = xy2ind(x+1,y, dx, dy)
        if not len(mapForStates[nexts]) or not len(mapForStates[nexte]):
            flag = 0

    if dir1=='s'and dir2=='w':
        nexts = xy2ind(x,y-1, dx, dy)
        nextw = xy2ind(x-1,y, dx, dy)
        if not len(mapForStates[nexts]) or not len(mapForStates[nextw]):
            flag = 0
    
    return flag


####################################
######## main
    
dx = 8
dy = 6
obstacles = [[2,0], [4,0], [5,0], [1,2], [2,2], [7,2], [2,3], [7,3], [2,4], [4,4], [7,4]]
nts=6*8 - len(obstacles)
# mapping of possible target states
mapForStates = {};
count = 0;
for y in range(0,dy):
    for x in range(0,dx):
        try :
            index = obstacles.index([x,y])
            ind = xy2ind(x, y, dx, dy);
            mapForStates[ind] = []
        except ValueError:
            #print("is not obstacle ", x,y)
            ind = xy2ind(x, y, dx, dy);
            mapForStates[ind] = [x, y, count]
            count=count+1
            pass
             
#print(mapForStates)

# Creating robot's transition matrix
north = np.zeros((nts,nts));
south = np.zeros((nts,nts));
east = np.zeros((nts,nts));
west = np.zeros((nts,nts));
northeast = np.zeros((nts,nts));
southeast = np.zeros((nts,nts));
northwest = np.zeros((nts,nts));
southwest = np.zeros((nts,nts));

probr=0.9;
probr2=0.8;
for indg in range(0,dx*dy):
    if len(mapForStates[indg]):  
        xr=mapForStates[indg][0];
        yr=mapForStates[indg][1];
        ind=mapForStates[indg][2];
        # north
        if yr<dy-1:
            next_ind = xy2ind(xr, yr+1, dx, dy);
            if len(mapForStates[next_ind]):
                indp=mapForStates[next_ind][2];
                north[ind][indp]=probr;
                north[ind][ind]=1-probr;
            else:
                north[ind][ind]=1.0;
        else:
            north[ind][ind]=1.0;
            
        # south
        if yr-1>=0:
            next_ind = xy2ind(xr, yr-1, dx, dy);
            if len(mapForStates[next_ind]):
                indp=mapForStates[next_ind][2];
                south[ind][indp]=probr;
                south[ind][ind]=1-probr;
            else:
                south[ind][ind]=1.0;
        else:
            south[ind][ind]=1.0;
            
        # east
        if xr<dx-1:
            next_ind = xy2ind(xr+1, yr, dx, dy);
            if len(mapForStates[next_ind]):
                indp=mapForStates[next_ind][2];
                east[ind][indp]=probr;
                east[ind][ind]=1-probr;
            else:
                east[ind][ind]=1.0;
        else:
            east[ind][ind]=1.0;    
            
        # west
        if xr-1>=0:
            next_ind = xy2ind(xr-1, yr, dx, dy);
            if len(mapForStates[next_ind]):
                indp=mapForStates[next_ind][2];
                west[ind][indp]=probr;
                west[ind][ind]=1-probr;
            else:
                west[ind][ind]=1.0;
        else:
            west[ind][ind]=1.0;    
            
        # northeast
        if yr<dy-1 and xr<dx-1:
            next_ind = xy2ind(xr+1, yr+1, dx, dy);
            if len(mapForStates[next_ind]) and checkneighbors(xr,yr,dx,dy,'n','e',mapForStates):
                indp=mapForStates[next_ind][2];
                northeast[ind][indp]=probr2;
                northeast[ind][ind]=1-probr2;
            else:
                northeast[ind][ind]=1.0;
        else:
            northeast[ind][ind]=1.0;    
            
        # southeast
        if yr-1>=0 and xr<dx-1:
            next_ind = xy2ind(xr+1, yr-1, dx, dy);
            if len(mapForStates[next_ind]) and checkneighbors(xr,yr,dx,dy,'s','e',mapForStates):
                indp=mapForStates[next_ind][2];
                southeast[ind][indp]=probr2;
                southeast[ind][ind]=1-probr2;
            else:
                southeast[ind][ind]=1.0;
        else:
            southeast[ind][ind]=1.0;      
            
        # northwest
        if yr<dy-1 and xr-1>=0:
            next_ind = xy2ind(xr-1, yr+1, dx, dy);
            if len(mapForStates[next_ind]) and checkneighbors(xr,yr,dx,dy,'n','w',mapForStates):
                indp=mapForStates[next_ind][2];
                northwest[ind][indp]=probr2;
                northwest[ind][ind]=1-probr2;
            else:
                northwest[ind][ind]=1.0;
        else:
            northwest[ind][ind]=1.0;      
            
        # southwest
        if yr-1>=0 and xr-1>=0:
            next_ind = xy2ind(xr-1, yr-1, dx, dy);
            if len(mapForStates[next_ind]) and checkneighbors(xr,yr,dx,dy,'s','w',mapForStates):
                indp=mapForStates[next_ind][2];
                southwest[ind][indp]=probr2;
                southwest[ind][ind]=1-probr2;
            else:
                southwest[ind][ind]=1.0;
        else:
            southwest[ind][ind]=1.0;          
            
            
# creating target robot transitions
# it holds for any robot action
target=np.zeros((nts,nts))         
p=0.2;
n_dest=8;
for indg in range(0,dx*dy):    
    indp=[];
    ndest=0;
    #print("indg :" + str(indg))
    if len(mapForStates[indg]):
        xt=mapForStates[indg][0];
        yt=mapForStates[indg][1];
        ind=mapForStates[indg][2];
        # east
        if (xt<dx-1):
            next_ind = xy2ind(xt+1,yt, dx, dy)
            #print("est : " + str(next_ind))
            if(len(mapForStates[next_ind])):
                indp.append(mapForStates[next_ind][2])
                #indp= [indp, mapForStates[next_ind][2]]
                ndest = ndest+1;

        # west
        if(xt-1>=0):
            next_ind = xy2ind(xt-1,yt, dx, dy)
            #print("west : " + str(next_ind))
            if(len(mapForStates[next_ind])):
                indp.append(mapForStates[next_ind][2])
                #indp= [indp, mapForStates[next_ind][2]]
                ndest=ndest+1;

        # north
        if(yt<dy-1):
            next_ind = xy2ind(xt,yt+1, dx, dy)
            #print("north : " + str(next_ind))
            if(len(mapForStates[next_ind])):
                indp.append(mapForStates[next_ind][2])
                #indp= [indp, mapForStates[next_ind][2]]
                ndest=ndest+1;

        # south
        if(yt-1>=0):
            next_ind = xy2ind(xt,yt-1, dx, dy)
            #print("south : " + str(next_ind))
            if(len(mapForStates[next_ind])):
                indp.append(mapForStates[next_ind][2])
                #indp= [indp, mapForStates[next_ind][2]]
                ndest=ndest+1;
        
        # northeast
        if yt<dy-1 and xt<dx-1 : 
            next_ind = xy2ind(xt+1,yt+1, dx, dy)
            #print("northeast : " + str(next_ind))
            if(len(mapForStates[next_ind])):
                indp.append(mapForStates[next_ind][2])
                #indp= [indp, mapForStates[next_ind][2]]
                ndest=ndest+1;

        # southeast
        if yt-1>=0 and xt<dx-1:
            next_ind = xy2ind(xt+1,yt-1, dx, dy)
            #print("southest : " + str(next_ind))
            if(len(mapForStates[next_ind])):
                indp.append(mapForStates[next_ind][2])
                #indp= [indp, mapForStates[next_ind][2]]
                ndest=ndest+1;
                
        # northwest
        if yt<dy-1 and xt-1>=0 :
            next_ind = xy2ind(xt-1,yt+1, dx, dy)
            #print("northwest : " + str(next_ind))
            if(len(mapForStates[next_ind])):
                indp.append(mapForStates[next_ind][2])
                #indp= [indp, mapForStates[next_ind][2]]
                ndest=ndest+1;
            
        # southwest
        if yt-1>=0 and xt-1>=0:
            next_ind = xy2ind(xt-1,yt-1, dx, dy)
            #print(xt,yt)
            #print("southwest : " + str(next_ind))
            if(len(mapForStates[next_ind])):
                indp.append(mapForStates[next_ind][2])
                #indp= [indp, mapForStates[next_ind][2]]
                ndest=ndest+1;
                
        target[ind,ind] = p;
        #print(indp)
        for i in indp:
            target[ind,i] = (1.0-p)/ndest;

            
for i in range(0,len(target)):
    if sum(target[i,:])<0.999:
        print(i,'problem TARGET: not sum 1 over s prime')

### main transition functions
transFunction=np.ndarray((9,nts*nts,nts*nts));
transFunction[0,:,:] = np.kron(north,target);
transFunction[1,:,:] = np.kron(south,target);
transFunction[2,:,:] = np.kron(east,target);
transFunction[3,:,:] = np.kron(west,target);
transFunction[4,:,:] = np.kron(northeast,target);
transFunction[5,:,:] = np.kron(southeast,target);
transFunction[6,:,:] = np.kron(northwest,target);
transFunction[7,:,:] = np.kron(southwest,target);
transFunction[8,:,:] = np.identity(nts*nts);

for a in range(0,9):
    for i in range(0,len(transFunction[a,:,:])):
        if sum(transFunction[a,i,:])<0.999:
            print(i, 'problem: not sum 1 over s prime')


# creating reward function
rewFunction = np.zeros((nts*nts,9))-1.0;
rewFunction[:,4:7] = -np.sqrt(2);
for i in range(0, nts):
    for j in range(0,nts):
        if (i==j):
            ind = i*nts+j;
            rewFunction[ind,8]=0;

## Adding obstacle avoidance

for i in range(0, nts):
    #north
    if i>=29 or i==6 or i==7 or i==12 or i==21:
        tstates = np.array([x for x in range(0,nts)])
        ind = i*nts + tstates
        rewFunction[ind,0] = -10;
    # south  
    if i<=4 or i==7 or i==9 or i==10 or i==19 or i==31 or i==33 or i==36:
        tstates = np.array([x for x in range(0,nts)])
        ind = i*nts + tstates
        rewFunction[ind,1] = -10;
    # east
    if i==1 or i==2 or i==4 or i==12 or i==13 or i==17 or i==19 or i==23 or i==25 or i==26 or i==28 or i==36 :
        tstates = np.array([x for x in range(0,nts)])
        ind = i*nts + tstates
        rewFunction[ind,2] = -10;
        
    # west
    if i==0 or i==2 or i==3 or i==5 or i==13 or i==14 or i==18 or i==20 or i==24 or i==26 or i==27 or i==29 :
        tstates = np.array([x for x in range(0,nts)])
        ind = i*nts + tstates
        rewFunction[ind,3] = -10;
        
    # northeast
    if i==1 or i==2 or i==4 or i==5 or i==6 or i==7 or i==11 or i==12 or i==13 or i==17 or i==19 or i==20 or i==21 or i== 23 or i==25 or i==26 or i>=28 :
        tstates = np.array([x for x in range(0,nts)])
        ind = i*nts + tstates
        rewFunction[ind,4] = -10;
        
    # southeast
    if i<=4 or i==6 or i==7 or i==8 or i==9 or i==10 or i==12 or i==13 or i==17 or i==18 or i==19 or i==23 or i==25 or i==26 or i== 28 or i==30 or i==31 or i==32 or i==33 or i==35 or i==36 :
        tstates = np.array([x for x in range(0,nts)])
        ind = i*nts + tstates
        rewFunction[ind,5] = -10;         
    
    #northwest
    if i==0 or i==2 or i==3 or i==5 or i==6 or i==7 or i==8 or i==12 or i==13 or i==14 or i==18 or i==20 or i==21 or i==22 or i==24 or i==26 or i==27 or i>=29:
        tstates = np.array([x for x in range(0,nts)])
        ind = i*nts + tstates
        rewFunction[ind,6] = -10;   
        
    #southwest
    if i<=5 or i==7 or i==8 or i==9 or i==10 or i==11 or i==13 or i==14 or i==18 or i==19 or i==20 or i==24 or i==26 or i==27 or i==29 or i==31 or i==32 or i==33 or i==34 or i==36:
        tstates = np.array([x for x in range(0,nts)])
        ind = i*nts + tstates
        rewFunction[ind,7] = -10;   
        