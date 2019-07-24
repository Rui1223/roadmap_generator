from __future__ import division
import pybullet as p
import time
import numpy as np
import pybullet_data
import utils
import IPython
import math
import random
import sys

from scipy import spatial
import cPickle as pickle

cam_info_filename = \
"/home/rui/Documents/research/roadmap_generator/ee_poses.txt"

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
kuka_ee_idx = 8
static_geometries = []

# Introduce Kuka arm and reset the base
######################################################################
kukaID = p.loadURDF("kuka.urdf",[0,0,0],useFixedBase=True)
static_geometries.append(kukaID)
p.resetBasePositionAndOrientation(kukaID, [-0.6, 0, 0.505], [0, 0, 0, 1])

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
######################################################################

# generate the static geometries
#####################################################################
print "---------Enter to shelf scene!---------"
# create the static geometries - shelf

# shelfbase
shelfbase_c = p.createCollisionShape(shapeType=p.GEOM_BOX,
						halfExtents=np.array([0.7, 1.3, 0.3])/2)
shelfbase_v = p.createVisualShape(shapeType=p.GEOM_BOX,
	halfExtents=np.array([0.7, 1.3, 0.3])/2, rgbaColor=[0.41, 0.41, 0.41, 1])
shelfbaseM = p.createMultiBody(baseCollisionShapeIndex=shelfbase_c,
				baseVisualShapeIndex=shelfbase_v,basePosition=[0, 0, 0.15])
static_geometries.append(shelfbaseM)

# the shape and visual of the flank
flank_c = p.createCollisionShape(shapeType=p.GEOM_BOX,
						halfExtents=np.array([0.7, 0.06, 1.2])/2)
flank_v = p.createVisualShape(shapeType=p.GEOM_BOX,
	halfExtents=np.array([0.7, 0.06, 1.2])/2, rgbaColor=[0.63, 0.32, 0.18, 1])
#leftflank
leftflankM = p.createMultiBody(baseCollisionShapeIndex=flank_c,
				baseVisualShapeIndex=flank_v,basePosition=[0, 0.62, 0.75])
static_geometries.append(leftflankM)
#rightflank
rightflankM = p.createMultiBody(baseCollisionShapeIndex=flank_c,
				baseVisualShapeIndex=flank_v,basePosition=[0, -0.62, 0.75])
static_geometries.append(rightflankM)
#middleflank
# middleflankM = p.createMultiBody(baseCollisionShapeIndex=flank_c,
# 				baseVisualShapeIndex=flank_v,basePosition=[0, 0, 0.75])
# static_geometries.append(middleflankM)

#the shape and visual of the flat
flat_c = p.createCollisionShape(shapeType=p.GEOM_BOX,
						halfExtents=np.array([0.7, 1.3, 0.06])/2)
flat_v = p.createVisualShape(shapeType=p.GEOM_BOX,
	halfExtents=np.array([0.7, 1.3, 0.06])/2, rgbaColor=[0.41, 0.41, 0.41, 1])
#topflat
topflatM = p.createMultiBody(baseCollisionShapeIndex=flat_c,
				baseVisualShapeIndex=flat_v,basePosition=[0, 0, 1.38])
static_geometries.append(topflatM)
#middleflat
middleflatM = p.createMultiBody(baseCollisionShapeIndex=flat_c,
				baseVisualShapeIndex=flat_v,basePosition=[0, 0, 0.8])
static_geometries.append(middleflatM)

#the shape and visual of the back
back_c = p.createCollisionShape(shapeType=p.GEOM_BOX,
						halfExtents=np.array([0.02, 1.3, 1.2])/2)
back_v = p.createVisualShape(shapeType=p.GEOM_BOX,
	halfExtents=np.array([0.02, 1.3, 1.2])/2, rgbaColor=[0.63, 0.32, 0.18, 1])
backM = p.createMultiBody(baseCollisionShapeIndex=back_c,
				baseVisualShapeIndex=back_v,basePosition=[0.34, 0, 0.8])
static_geometries.append(backM)

#the shape and visual of the standingBase
standingBase_c = p.createCollisionShape(shapeType=p.GEOM_BOX,
						halfExtents=np.array([0.3, 0.3, 0.5])/2)
standingBase_v = p.createVisualShape(shapeType=p.GEOM_BOX,
	halfExtents=np.array([0.3, 0.3, 0.5])/2)
standingBaseM = p.createMultiBody(baseCollisionShapeIndex=standingBase_c,
				baseVisualShapeIndex=standingBase_v,basePosition=[-0.6, 0, 0.25])
static_geometries.append(standingBaseM)
######################################################################



# generate hypothesis (including the true pose) for each objects
# 8 objects includes baseball as the target object
##########################################################################
Objects = dict()
# meshfile, meshType, objectRole, scale, true_pos, true_angles, uncertainty, nHypo
Objects[0] = [0, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
	"target", 2, [0.21, 0.2, 0.9], [0.0, 0.0, 0.0], [0.05, 0.05], 3]
# Objects[1] = [1, "/mesh/soft_white_lightbulb/soft_white_lightbulb.obj", 
# "lightbulb", "normal", 1.5, [-0.12, 0.25, 0.86], [0.0, 0.0, math.pi/10.0], 
# 														[0.032, 0.035, 0.06], 3]
# Objects[2] = [2, "/mesh/elmers_washable_no_run_school_glue/elmers_washable_no_run_school_glue.obj", 
# "glue", "normal", 1.5, [0.02, 0.44, 0.92], [math.pi / 2, 0.0, 25*math.pi/180], 
# 															[0.07, 0.07, 0.5], 3]
# Objects[3] = [3, "/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", 
# "tissueBox", "normal", 1.5, [-0.2, -0.31, 0.91], [0.0, 0.0, -math.pi/1.2], 
# 														[0.03, 0.03, 0.24], 3]
# Objects[4] = [4, "/mesh/dove_beauty_bar/dove_beauty_bar.obj", "doveBar", 
# 	"normal", 2, [0.1, 0.24, 0.32], [math.pi/2, 0.0, math.pi/4.3], [0.03, 0.06, 0.01], 3]
# Objects[5] = [5, "/mesh/dasani_water_bottle/dasani_water_bottle.obj", "waterBottle", 
# 	"normal", 2, [-0.2, 0.36, 0.37], [0.0, 0.0, -math.pi/5.2], [0.05, 0.04, 0.3], 3]
# Objects[6] = [6, "/mesh/up_glucose_bottle/up_glucose_bottle.obj", 
# "glucoseBottle", "normal", 1.5, [-0.23, -0.43, 0.4], [math.pi / 2, 0.0, math.pi], 
# 														[0.04, 0.04, 0.12], 3]
# Objects[7] = [7, "/mesh/dr_browns_bottle_brush/dr_browns_bottle_brush.obj", 
# "brush", "normal", 1, [0.07, -0.32, 0.325], [0.0, 0.0, -math.pi / 3.1], 
# 														[0.028, 0.02, 0.2], 3]
### Collect meshes for all the objects ###
meshDict = dict()
for i in xrange(len(Objects)):
	meshDict[i] = utils.createMesh(Objects[i][0], Objects[i][1], Objects[i][2], 
		Objects[i][3], Objects[i][4], Objects[i][5], Objects[i][6], 
												Objects[i][7], Objects[i][8])
##########################################################################



##############start sampling##################
f = open("kuka_shelf_roadmap.txt", "w")

nodes = []

nsamples = 5000
f.write(str(nsamples)+"\n")
temp_counter = 0

while temp_counter < nsamples:
	# sample a configuration based on the joint limit
	j1 = random.uniform(ll[0], ul[0])
	j2 = random.uniform(ll[1], ul[1])
	j3 = random.uniform(ll[2], ul[2])
	j4 = random.uniform(ll[3], ul[3])
	j5 = random.uniform(ll[4], ul[4])
	j6 = random.uniform(ll[5], ul[5])
	j7 = random.uniform(ll[6], ul[6])
	ikSolution = [j1, j2, j3, j4, j5, j6, j7]
	for j in range(1,8):
		result = p.resetJointState(kukaID,j,ikSolution[j-1])
	p.stepSimulation()
	#time.sleep(0.05)
	isCollision = utils.collisionCheck_staticG(kukaID, static_geometries)
	#print "Collision Status: " + str(isCollision)
	if isCollision == False:
		nodes.append(ikSolution)
		## write it into a roadmap file
		f.write(str(temp_counter) + " " + str(j1) + " " + str(j2) + " " \
			+ str(j3) + " " + str(j4) + " " + str(j5) + " " + str(j6) + " " \
			+ str(j7) + "\n")
		temp_counter += 1
f.close()

tree = spatial.KDTree(nodes)
num_neighbors = math.log(nsamples)
nsteps = 100

# for each node
for i in xrange(len(nodes)):
	queryNode = nodes[i]
	knn = tree.query(queryNode, k=num_neighbors, p=2)
	# for each neighbor
	for j in xrange(len(knn[0])):
		if knn[1][j] == i: ## the neighbor is the query node itself
			continue 
		# Otherwise, check the edge validity
		# between the query node and the the current neighbor
		neighbor = node[knn[1][j]]
		isEdgeValid = checkEdgeValidity(queryNode, neighbor, kukaID, 
															static_geometries)




# let's look at these joint configurations (if they are valid or feasible)
# n_line = 0
# f = open("kuka_shelf_roadmap.txt", "r")
# for line in f:
# 	line = line.split()
# 	n_line += 1

# 	if (n_line >=2 and n_line <= 101):
# 		## These are the lines to read all the samples (joint configurations)
# 		idx = int(line[0])
# 		temp_j1 = float(line[1])
# 		temp_j2 = float(line[2])
# 		temp_j3 = float(line[3])
# 		temp_j4 = float(line[4])
# 		temp_j5 = float(line[5])
# 		temp_j6 = float(line[6])
# 		temp_j7 = float(line[7])
# 		temp_ikSolution = [temp_j1, temp_j2, temp_j3, 
# 							temp_j4, temp_j5, temp_j6, temp_j7]
# 		for j in range(1, 8):
# 			result = p.resetJointState(kukaID,j,temp_ikSolution[j-1])
# 		p.stepSimulation()
# 		time.sleep(0.1)

		
#print (len(nodes))
time.sleep(10000)