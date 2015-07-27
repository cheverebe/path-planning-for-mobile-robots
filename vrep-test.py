# -*- coding: utf-8 -*-
"""
@author: Thomas Fischer
Created on Jul 22 2015
"""

import vrep
import sys
import time
import numpy as np
import math
from classes import *

# just in case, close all opened connections
vrep.simxFinish(-1)

PORT = 19997
clientID = vrep.simxStart("127.0.0.1", PORT, True, True, 5000, 5)
random.seed()


def run(path, handles):
    for state in path:
        for i in range(len(handles)):
            handle = handles[i]
            ret, pos = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_oneshot_wait)

            # wait until it's almost in the position
            # ------

            if (not ret == vrep.simx_return_ok):
                print('Failed to retrieve position for Quadricopter_target')
                sys.exit(1)

            #print pos
            new_pos = pos
            new_pos[0] = state.positions[i].as_tuple()[0]
            new_pos[1] = state.positions[i].as_tuple()[1]

            ret = vrep.simxSetObjectPosition(clientID, handle, -1, new_pos, vrep.simx_opmode_oneshot_wait)
            if (not ret == vrep.simx_return_ok):
                print('Failed to set position for Quadricopter_target')
                sys.exit(1)


# check if client connection successful
if clientID == -1:
    print('Could not connect to remote API server')
    sys.exit(1)

print('Connected to remote API server')

#----------------FINDING QUADS
ret, handle = vrep.simxGetObjectHandle(clientID, 'Quadricopter_target', vrep.simx_opmode_oneshot_wait)
if (not ret == vrep.simx_return_ok):
    print('Failed to retrieve handle for Quadricopter_target')
    sys.exit(1)
handles = []
drones = -1
initial_positions = []
while ret == vrep.simx_return_ok:
    handles.append(handle)
    ret, pos = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_oneshot_wait)
    init_pos = Position([pos[0], pos[1]])
    initial_positions.append(init_pos)

    drones += 1
    qid = "Quadricopter_target#%d" % drones
    ret, handle = vrep.simxGetObjectHandle(clientID, qid, vrep.simx_opmode_oneshot_wait)
print(handles)
initial_state = State(initial_positions)

#-----------------FINDING TARGET
ret, handle = vrep.simxGetObjectHandle(clientID, 'ResizableFloor_1_5', vrep.simx_opmode_oneshot_wait)
if (not ret == vrep.simx_return_ok):
    print('Failed to retrieve handle for ResizableFloor_1_5')
    sys.exit(1)
ret, pos = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_oneshot_wait)
target = Target(Position([pos[0], pos[1]]), .5)

#-----------------FINDING OBSTACLES
ret, handle = vrep.simxGetObjectHandle(clientID, 'Tree', vrep.simx_opmode_oneshot_wait)
if (not ret == vrep.simx_return_ok):
    print('Failed to retrieve handle for Tree')
    sys.exit(1)
trees = -1
obstacles = []
while ret == vrep.simx_return_ok:
    ret, pos = vrep.simxGetObjectPosition(clientID, handle, -1, vrep.simx_opmode_oneshot_wait)
    obstacle_pos = Position([pos[0], pos[1]])
    obstacle = Obstacle(obstacle_pos)
    obstacles.append(obstacle)

    trees += 1
    qid = "Tree#%d" % trees
    ret, handle = vrep.simxGetObjectHandle(clientID, qid, vrep.simx_opmode_oneshot_wait)

map = Map(obstacles)

#---------------CREATING TREE
rrt = RRT(initial_state, target, primitives(), map)
rrt.grow(1200)
rrt.plot()
path = rrt.get_solution()
run(path, handles)
