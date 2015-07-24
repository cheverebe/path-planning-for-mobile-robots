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
vrep.simxFinish( -1 )

PORT=19997
clientID = vrep.simxStart("127.0.0.1", PORT, True, True, 5000, 5)
random.seed()

# check if client connection successful
if clientID == -1:
  print('Could not connect to remote API server')
  sys.exit( 1 )

print('Connected to remote API server')

ret, handle = vrep.simxGetObjectHandle( clientID, 'Quadricopter_target', vrep.simx_opmode_oneshot_wait )
handles = [handle]
if ( not ret == vrep.simx_return_ok ):
  print('Failed to retrieve handle for Quadricopter_target')
  sys.exit( 1 )
ret, pos = vrep.simxGetObjectPosition( clientID, handle, -1, vrep.simx_opmode_oneshot_wait )
init_pos = Position([pos[0],pos[1]])
initial_state = State([init_pos])
target_checker = lambda pos: (0.5<=pos.as_tuple()[0] and pos.as_tuple()[0]<=0.9 and -0.5>=pos.as_tuple()[1] and pos.as_tuple()[1]>=-0.9)
rrt = RRT(initial_state, target_checker, primitives())
rrt.grow(1000)
rrt.plot()


def run(path, handles):

	for step in path:
		for i in range(len(handles)):
			handle = handles[i]
			ret, pos = vrep.simxGetObjectPosition( clientID, handle, -1, vrep.simx_opmode_oneshot_wait )

			# wait until it's almost in the position
			#------

			if ( not ret == vrep.simx_return_ok ):
				print('Failed to retrieve position for Quadricopter_target')
				sys.exit( 1 )

			print pos
			new_pos = step[i]

			ret = vrep.simxSetObjectPosition( clientID, handle, -1, new_pos, vrep.simx_opmode_oneshot_wait )
			if ( not ret == vrep.simx_return_ok ):
				print('Failed to set position for Quadricopter_target')
				sys.exit( 1 )

