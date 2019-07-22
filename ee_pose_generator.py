# This file tries to generate a txt file include 2000 valid ee_poses to mimic
# in the simulator
from __future__ import division
import pybullet as p
import time
import numpy as np
import pybullet_data
import utils
import IPython
import math
import sys
import random

from scipy import spatial
import cPickle as pickle

p.setAdditionalSearchPath(pybullet_data.getDataPath())
kuka_ee_idx = 8

# Introduce Kuka arm and reset the base
######################################################################
kukaID = p.loadURDF("kuka.urdf",[0,0,0],useFixedBase=True)
p.resetBasePositionAndOrientation(kukaID, [0, 0, 0], [0, 0, 0, 1])

#lower limits for null space
ll = [-2.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
#upper limits for null space
ul = [2.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
#joint ranges for null space
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
#restposes for null space
rp = [0,0,0,-np.pi/2,0,np.pi/2,0]

# for i in range(p.getNumJoints(kukaID)):
# 	print(p.getJointInfo(kukaID,i))

home_configuration = [0,0,0,-np.pi/2,0,np.pi/2,0]
for i in range(1,8):
	result = p.resetJointState(kukaID,i,home_configuration[i-1])
#######################################################################

nPoses = 2000
counter = 0
while (counter < 2000):
	# sample a ee pose
	x = random.uniform()