from __future__ import division
import pybullet as p
import pybullet_data
import utils_motoman

import math
import random
import time
import numpy as np

import sys
import os
import subprocess

from scipy import spatial
import cPickle as pickle

import IPython

### create two servers ###
### One for planning, the other executing (ground truth) ###
planningServer = p.connect(p.DIRECT)
executingServer = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
### set the real-time physics simulation ###
# p.setGravity(0.0, 0.0, -9.8, executingServer)
# p.setRealTimeSimulation(1, executingServer)
static_geometries_planning = []
static_geometries_executing = []

### Introduce Motoman arm ###
motomanID_p = p.loadURDF("motoman.urdf", useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION 
								or p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT, physicsClientId=planningServer)
motomanID_e = p.loadURDF("motoman.urdf", useFixedBase=True, physicsClientId=executingServer)
static_geometries_planning.append(motomanID_p)
static_geometries_executing.append(motomanID_e)
# print "Motoman Robot: " + str(motomanID_e)
# num_joints = p.getNumJoints(motomanID_e, executingServer)
# print "Num of joints: " + str(num_joints)
# for i in range(num_joints):
# 	print(p.getJointInfo(motomanID_e, i, executingServer))

########## information related to Motoman ###########
motoman_ee_ix = 10
### There is a torso joint which connect the lower and upper body (-2.957 ~ 2.957)
### For each arm, there are 10 joints and 7 of them are revolute joints
### There are total 15 revolute joints for each arm
### lower limits for null space
ll = [-2.957, -3.13, -1.90, -2.95, -2.36, -3.13, -1.90, -3.13, -3.13, -1.90, -2.95, -2.36, -3.13, -1.90, -3.13]
### upper limits for null space
ul = [2.957, 3.13, 1.90, 2.95, 2.36, 3.13, 1.90, 3.13, 3.13, 1.90, -2.95, 2.36, 3.13, 1.90, 3.13]
### joint ranges for null space
jr = [5.914, 6.26, 3.80, 5.90, 4.72, 6.26, 3.80, 6.26, 6.26, 3.80, 5.90, 4.72, 6.26, 3.80, 6.26]
### restposes for null space
rp = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]



### set up several parameter before generating the benchmark ###
bt = sys.argv[1] # choose "table" or "shelf"
# scene = sys.argv[2] # choose "1" or "2"
# nHypos = int(sys.argv[3]) # choose from (1-7)
# noiseLevel = int(sys.argv[4]) # choose from (1-7)
# transErrors = [0.005, 0.01, 0.015, 0.02, 0.025, 0.03, 0.035]
# orientErrors = [5, 10, 15, 20, 25, 30, 35]
# orientErrors = [float(format(ii * math.pi/180, '.3f')) for ii in orientErrors]
# nsamples = int(sys.argv[5]) # choose from (1000-5000)
# mode = sys.argv[6] # choose "g" or "e"
# if mode == "e":
# 	idx1 = sys.argv[7]
# 	idx2 = sys.argv[8]

known_geometries_planning = []
known_geometries_executing = []


### generate the static geometries ###
########################################################################################################################
if bt == "table":
	table_path = "newChapter/table"
	try:
		os.mkdir(table_path)
	except OSError:
		print "Creation of the directory %s falied\n" % table_path
	else:
		pass
	print "---------Enter to table scene!----------"
	### create the known geometries - table ###
	table_dim = np.array([0.8, 1.3, 0.35])
	tablePosition=[0, 0, table_dim[2]/2]
	table_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX,
							halfExtents=table_dim/2, physicsClientId=planningServer)
	table_v_p = p.createVisualShape(shapeType=p.GEOM_BOX,
							halfExtents=table_dim/2, physicsClientId=planningServer)
	tableM_p = p.createMultiBody(baseCollisionShapeIndex=table_c_p, baseVisualShapeIndex=table_v_p,
										basePosition=tablePosition, physicsClientId=planningServer)
	table_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX,
							halfExtents=table_dim/2, physicsClientId=executingServer)
	table_v_e = p.createVisualShape(shapeType=p.GEOM_BOX,
							halfExtents=table_dim/2, physicsClientId=executingServer)
	tableM_e = p.createMultiBody(baseCollisionShapeIndex=table_c_e, baseVisualShapeIndex=table_v_e,
										basePosition=tablePosition, physicsClientId=executingServer)
	known_geometries_planning.append(tableM_p)
	known_geometries_executing.append(tableM_e)
	print "table: " + str(tableM_e)
	### reset the base of motoman
	motomanBasePosition = [tablePosition[0]-table_dim[0]/2-0.4, tablePosition[1], 0.0]
	motomanBaseOrientation = [0, 0, 0, 1]
	p.resetBasePositionAndOrientation(motomanID_p, motomanBasePosition, 
									motomanBaseOrientation, physicsClientId=planningServer)
	p.resetBasePositionAndOrientation(motomanID_e, motomanBasePosition, 
									motomanBaseOrientation, physicsClientId=executingServer)
	### set motoman home configuration
	home_configuration = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
	# home_configuration = [0, np.pi/2, np.pi/2, np.pi, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, 
	# 														-np.pi/2, np.pi/2, np.pi, np.pi/2, np.pi, np.pi/2, np.pi/2]
	### left arm with the suction cup
	for i in range(0, 8):
		result_p = p.resetJointState(motomanID_p, i, home_configuration[i], physicsClientId=planningServer)
		result_e = p.resetJointState(motomanID_e, i, home_configuration[i], physicsClientId=executingServer)
	### right arm with the fingered gripper
	for i in range(11, 18):
		result_p = p.resetJointState(motomanID_p, i, home_configuration[i-3], physicsClientId=planningServer)
		result_e = p.resetJointState(motomanID_e, i, home_configuration[i-3], physicsClientId=executingServer)
else:
	shelf_path = "newChapter/shelf"
	try:
		os.mkdir(shelf_path)
	except OSError:
		print "Creation of the directory %s falied\n" % shelf_path
	else:
		pass
	print "---------Enter to shelf scene!----------"
	# ### standingBase
	# standingBase_dim = np.array([0.3, 0.3, 0.65])
	# standingBasePosition=[-0.6, 0.0, standingBase_dim[2]/2]
	# standingBase_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX, 
	# 				halfExtents=standingBase_dim/2, physicsClientId=planningServer)
	# standingBase_v_p = p.createVisualShape(shapeType=p.GEOM_BOX, 
	# 				halfExtents=standingBase_dim/2, physicsClientId=planningServer)
	# standingBaseM_p = p.createMultiBody(baseCollisionShapeIndex=standingBase_c_p,
	# 						baseVisualShapeIndex=standingBase_v_p,
	# 							basePosition=standingBasePosition, physicsClientId=planningServer)
	# standingBase_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX, 
	# 				halfExtents=standingBase_dim/2, physicsClientId=executingServer)
	# standingBase_v_e = p.createVisualShape(shapeType=p.GEOM_BOX, 
	# 				halfExtents=standingBase_dim/2, physicsClientId=executingServer)
	# standingBaseM_e = p.createMultiBody(baseCollisionShapeIndex=standingBase_c_e,
	# 						baseVisualShapeIndex=standingBase_v_e,
	# 							basePosition=standingBasePosition, physicsClientId=executingServer)
	# known_geometries_planning.append(standingBaseM_p)
	# known_geometries_executing.append(standingBaseM_e)
	# print "standing base: " + str(standingBaseM_e)
	### create the known geometries - shelf ###
	### shelfbase
	shelfbase_dim = np.array([0.7, 1.3, 0.3])
	shelfbasePosition = [0.0, 0.0, shelfbase_dim[2]/2]
	shelfbase_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX, 
							halfExtents=shelfbase_dim/2, physicsClientId=planningServer)
	shelfbase_v_p = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=shelfbase_dim/2, 
								rgbaColor=[0.41, 0.41, 0.41, 1], physicsClientId=planningServer)
	shelfbaseM_p = p.createMultiBody(baseCollisionShapeIndex=shelfbase_c_p,
							baseVisualShapeIndex=shelfbase_v_p,
								basePosition=shelfbasePosition, physicsClientId=planningServer)
	shelfbase_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX, 
							halfExtents=shelfbase_dim/2, physicsClientId=executingServer)
	shelfbase_v_e = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=shelfbase_dim/2, 
								rgbaColor=[0.41, 0.41, 0.41, 1], physicsClientId=executingServer)
	shelfbaseM_e = p.createMultiBody(baseCollisionShapeIndex=shelfbase_c_e,
							baseVisualShapeIndex=shelfbase_v_e,
								basePosition=shelfbasePosition, physicsClientId=executingServer)
	known_geometries_planning.append(shelfbaseM_p)
	known_geometries_executing.append(shelfbaseM_e)
	print "shelf base: " + str(shelfbaseM_e)
	### the shape and visual of the flank
	flank_dim = np.array([shelfbase_dim[0], 0.06, 1.2])
	flank_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX, 
								halfExtents=flank_dim/2, physicsClientId=planningServer)
	flank_v_p = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=flank_dim/2, 
							rgbaColor=[0.63, 0.32, 0.18, 1], physicsClientId=planningServer)
	flank_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX, 
								halfExtents=flank_dim/2, physicsClientId=executingServer)
	flank_v_e = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=flank_dim/2, 
							rgbaColor=[0.63, 0.32, 0.18, 1], physicsClientId=executingServer)
	### left flank
	leftflankPosition = [0.0, shelfbase_dim[1]/2-flank_dim[1]/2, shelfbase_dim[2]+flank_dim[2]/2]
	leftflankM_p = p.createMultiBody(baseCollisionShapeIndex=flank_c_p, 
								baseVisualShapeIndex=flank_v_p, 
									basePosition=leftflankPosition, physicsClientId=planningServer)
	leftflankM_e = p.createMultiBody(baseCollisionShapeIndex=flank_c_e, 
								baseVisualShapeIndex=flank_v_e, 
									basePosition=leftflankPosition, physicsClientId=executingServer)
	known_geometries_planning.append(leftflankM_p)
	known_geometries_executing.append(leftflankM_e)
	print "left flank: " + str(leftflankM_e)
	### right flank
	rightflankPosition = [0.0, -shelfbase_dim[1]/2+flank_dim[1]/2, shelfbase_dim[2]+flank_dim[2]/2]
	rightflankM_p = p.createMultiBody(baseCollisionShapeIndex=flank_c_p,
								baseVisualShapeIndex=flank_v_p, 
									basePosition=rightflankPosition, physicsClientId=planningServer)
	rightflankM_e = p.createMultiBody(baseCollisionShapeIndex=flank_c_e,
								baseVisualShapeIndex=flank_v_e, 
									basePosition=rightflankPosition, physicsClientId=executingServer)
	known_geometries_planning.append(rightflankM_p)
	known_geometries_executing.append(rightflankM_e)
	print "right flank: " + str(rightflankM_e)
	### middle flank
	middleflankPosition = [0.0, 0.0, shelfbase_dim[2]+flank_dim[2]/2]
	middleflankM_p = p.createMultiBody(baseCollisionShapeIndex=flank_c_p,
							baseVisualShapeIndex=flank_v_p, 
								basePosition=middleflankPosition, physicsClientId=planningServer)
	middleflankM_e = p.createMultiBody(baseCollisionShapeIndex=flank_c_e,
							baseVisualShapeIndex=flank_v_e, 
								basePosition=middleflankPosition, physicsClientId=executingServer)
	known_geometries_planning.append(middleflankM_p)
	known_geometries_executing.append(middleflankM_e)
	print "middle flank: " + str(middleflankM_e)
	### the shape and visual of the flat
	flat_dim = np.array([shelfbase_dim[0], shelfbase_dim[1], 0.06])
	flat_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX, 
									halfExtents=flat_dim/2, physicsClientId=planningServer)
	flat_v_p = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=flat_dim/2, 
							rgbaColor=[0.41, 0.41, 0.41, 1], physicsClientId=planningServer)
	flat_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX, 
									halfExtents=flat_dim/2, physicsClientId=executingServer)
	flat_v_e = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=flat_dim/2, 
							rgbaColor=[0.41, 0.41, 0.41, 1], physicsClientId=executingServer)
	### middle flat
	middleflatPosition = [0.0, 0.0, shelfbase_dim[2]+flank_dim[2]/2]
	middleflatM_p = p.createMultiBody(baseCollisionShapeIndex=flat_c_p,
						baseVisualShapeIndex=flat_v_p, 
							basePosition=middleflatPosition, physicsClientId=planningServer)
	middleflatM_e = p.createMultiBody(baseCollisionShapeIndex=flat_c_e,
						baseVisualShapeIndex=flat_v_e, 
							basePosition=middleflatPosition, physicsClientId=executingServer)
	known_geometries_planning.append(middleflatM_p)
	known_geometries_executing.append(middleflatM_e)
	print "middle flat: " + str(middleflatM_e)
	### top flat
	topflatPosition = [0.0, 0.0, shelfbase_dim[2]+flank_dim[2]+flat_dim[2]/2]
	topflatM_p = p.createMultiBody(baseCollisionShapeIndex=flat_c_p,
						baseVisualShapeIndex=flat_v_p, 
							basePosition=topflatPosition, physicsClientId=planningServer)
	topflatM_e = p.createMultiBody(baseCollisionShapeIndex=flat_c_e,
						baseVisualShapeIndex=flat_v_e, 
							basePosition=topflatPosition, physicsClientId=executingServer)
	known_geometries_planning.append(topflatM_p)
	known_geometries_executing.append(topflatM_e)
	print "top flat: " + str(topflatM_e)
	### back
	shelfback_dim = np.array([0.02, shelfbase_dim[1], flank_dim[2]+flat_dim[2]])
	shelfbackPosition = [shelfbase_dim[0]/2-shelfback_dim[0]/2, 0.0, 
														shelfbase_dim[2]+shelfback_dim[2]/2]
	shelfback_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=shelfback_dim/2, 
																physicsClientId=planningServer)
	shelfback_v_p = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=shelfback_dim/2, 
								rgbaColor=[0.63, 0.32, 0.18, 1], physicsClientId=planningServer)
	shelfback_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=shelfback_dim/2, 
																physicsClientId=executingServer)
	shelfback_v_e = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=shelfback_dim/2, 
								rgbaColor=[0.63, 0.32, 0.18, 1], physicsClientId=executingServer)
	shelfbackM_p = p.createMultiBody(baseCollisionShapeIndex=shelfback_c_p,
							baseVisualShapeIndex=shelfback_v_p, 
								basePosition=shelfbackPosition, physicsClientId=planningServer)
	shelfbackM_e = p.createMultiBody(baseCollisionShapeIndex=shelfback_c_e,
							baseVisualShapeIndex=shelfback_v_e, 
								basePosition=shelfbackPosition, physicsClientId=executingServer)
	known_geometries_planning.append(shelfbackM_p)
	known_geometries_executing.append(shelfbackM_e)
	print "shelf back: " + str(shelfbackM_e)
	### reset the base of Motoman
	motomanBasePosition = [shelfbasePosition[0]-shelfbase_dim[0]/2-0.8, shelfbasePosition[1], 0.0]
	motomanBaseOrientation = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
	p.resetBasePositionAndOrientation(motomanID_p, motomanBasePosition, 
									motomanBaseOrientation, physicsClientId=planningServer)
	p.resetBasePositionAndOrientation(motomanID_e, motomanBasePosition, 
									motomanBaseOrientation, physicsClientId=executingServer)
	### set motoman home configuration
	home_configuration = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
	# home_configuration = [0, np.pi/2, np.pi/2, np.pi, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, 
	# 														-np.pi/2, np.pi/2, np.pi, np.pi/2, np.pi, np.pi/2, np.pi/2]
	### left arm with the suction cup
	for i in range(0, 8):
		result_p = p.resetJointState(motomanID_p, i, home_configuration[i], physicsClientId=planningServer)
		result_e = p.resetJointState(motomanID_e, i, home_configuration[i], physicsClientId=executingServer)
	### right arm with the fingered gripper
	for i in range(11, 18):
		result_p = p.resetJointState(motomanID_p, i, home_configuration[i-3], physicsClientId=planningServer)
		result_e = p.resetJointState(motomanID_e, i, home_configuration[i-3], physicsClientId=executingServer)
########################################################################################################################

'''
### specify the objects in a chosen scene ###
if bt == "table" and scene == "1":
	path = table_path + "/scenario1"
	print "--------Welcome to " + bt + " sceneario " + scene + "--------"
	try:
		os.mkdir(path)
	except OSError:
		print "Creation of the directory %s falied\n" % path
	else:
		pass

	### generate hypothesis (including the true pose) for each objects
	### 7 objects includes baseball as the target object	
	####################################################################################################################
	Objects = dict()
	### objIdx, meshFile, objectName, objectRole, scale, true_pos, true_angles(radians), mass, nHypos
	Objects[0] = [0, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
		"target", 2, [0.20, 0.0, 0.07+table_dim[2]], [math.pi/5.8, math.pi/0.4, math.pi/2.3], 3.2, nHypos]
	Objects[1] = [1, "/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", "tissueBox", 
		"normal", 1.65, [0.06, 0.16, 0.1+table_dim[2]], [0.0, -math.pi/2, -math.pi/3], 3.5, nHypos]
	Objects[2] = [2, "/mesh/crayola_24_ct/crayola_24_ct.obj", "crayola", 
		"normal", 2, [0.13, -0.14, 0.1+table_dim[2]], [math.pi/2, 0.0, math.pi/4], 3.8, nHypos]
	Objects[3] = [3, "/mesh/dove_beauty_bar/dove_beauty_bar.obj", "doveBar", 
		"normal", 2, [-0.16, 0.175, 0.09+table_dim[2]], [-math.pi, 0.0, math.pi/1.3], 4.0, nHypos]
	Objects[4] = [4, "/mesh/dr_browns_bottle_brush/dr_browns_bottle_brush.obj", "brush", 
		"normal", 1, [0.17, -0.40, 0.02+table_dim[2]], [0.0, 0.0, -math.pi / 4.5], 1.7, nHypos]
	Objects[5] = [5, "/mesh/up_glucose_bottle/up_glucose_bottle.obj", "glucoseBottle", 
		"normal", 1.5, [0.2, 0.38, 0.1+table_dim[2]], [math.pi / 2, 0.0, math.pi], 2.6, nHypos]
	Objects[6] = [6, "/mesh/dasani_water_bottle/dasani_water_bottle.obj", "waterBottle", 
		"normal", 2, [-0.1, -0.18, 0.2+table_dim[2]], [math.pi/2, 0.0, 0.0], 4.2, nHypos]
	Objects[7] = [7, "/mesh/soft_white_lightbulb/soft_white_lightbulb.obj", "lightbulb", 
		"phantom", 1.5, [-0.14, 0.41, 0.05+table_dim[2]], [0.0, -math.pi, 9*math.pi/10.0], 2.2, 1]
	### pick goal offset
	goalPos_offset = [0.0, 0.0, 0.16]
	goalEuler = [0.0, math.pi, 0.0]  ### tabletop picking
	x_ll = kukaBasePosition[0] - 0.2
	x_ul = tablePosition[0] + table_dim[0]/2
	y_ll = -table_dim[1]/2
	y_ul = table_dim[1]/2
	z_ll = table_dim[2]
	z_ul = table_dim[2] + 0.7
	####################################################################################################################

if bt == "table" and scene == "2":
	path = table_path + "/scenario2"
	print "--------Welcome to " + bt + " sceneario " + scene + "--------"
	try:
		os.mkdir(path)
	except OSError:
		print "Creation of the directory %s falied\n" % path
	else:
		pass

	### generate hypothesis (including the true pose) for each objects
	### 10 objects includes baseball as the target object
	####################################################################################################################
	Objects = dict()
	### objIdx, meshFile, objectName, objectRole, scale, true_pos, true_angles(radians), mass, nHypos
	Objects[0] = [0, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
			"target", 2, [0.16, 0.215, 0.07+table_dim[2]], [2*math.pi/7.0, math.pi/4.8, 7*math.pi/6.2], 3.2, nHypos]
	Objects[1] = [1, "/mesh/dove_beauty_bar/dove_beauty_bar.obj", "doveBar", 
			"normal", 2, [0.27, 0.38, 0.05+table_dim[2]], [math.pi/2, 0.0, math.pi/4.3], 4.0, nHypos]
	Objects[2] = [2, "/mesh/dasani_water_bottle/dasani_water_bottle.obj", "waterBottle", 
			"normal", 2, [-0.09, 0.31, 0.2+table_dim[2]], [math.pi/2, 0.0, math.pi/2.6], 4.2, nHypos]
	Objects[3] = [3, "/mesh/ticonderoga_12_pencils/ticonderoga_12_pencils.obj", "pencils", 
			"normal", 1, [0.16, -0.08, 0.01+table_dim[2]], [0.0, -math.pi/2, -math.pi/8.8], 3.4, nHypos]
	Objects[4] = [4, "/mesh/up_glucose_bottle/up_glucose_bottle.obj", "glucoseBottle", 
			"normal", 1.5, [0.28, 0.11, 0.1+table_dim[2]], [math.pi / 2, 0.0, math.pi], 2.6, nHypos]
	Objects[5] = [5, "/mesh/crayola_24_ct/crayola_24_ct.obj", "crayola", 
			"normal", 2, [-0.09, 0.16, 0.09+table_dim[2]], [math.pi/2, 0.0, math.pi/3], 3.8, nHypos]
	Objects[6] = [6, "/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", "tissueBox", 
			"normal", 1.5, [0.0, 0.0, 0.1+table_dim[2]], [0.0, -math.pi/2, -math.pi/3], 3.5, nHypos]
	Objects[7] = [7, "/mesh/dr_browns_bottle_brush/dr_browns_bottle_brush.obj", "brush", 
			"normal", 1, [-0.21, -0.23, 0.02+table_dim[2]], [0.0, 0.0, 1.7*math.pi / 4.5], 1.7, nHypos]
	Objects[8] = [8, "/mesh/elmers_washable_no_run_school_glue/elmers_washable_no_run_school_glue.obj", "glue", 
			"normal", 1.5, [0.03, 0.31, 0.11+table_dim[2]], [math.pi / 2, 0.0, 67*math.pi/180], 1.8, nHypos]
	Objects[9] = [9, "/mesh/soft_white_lightbulb/soft_white_lightbulb.obj", "lightbulb", 
			"phantom", 1.5, [0.12, 0.52, 0.05+table_dim[2]], [0.0, math.pi, 4*math.pi/6.7], 2.2, 1]
	### pick goal offset
	goalPos_offset = [0.0, 0.0, 0.16]
	goalEuler = [0.0, math.pi, 0.0] ### tabletop picking
	x_ll = kukaBasePosition[0] - 0.2
	x_ul = tablePosition[0] + table_dim[0]/2
	y_ll = -table_dim[1]/2
	y_ul = table_dim[1]/2
	z_ll = table_dim[2]
	z_ul = table_dim[2]+0.7
	####################################################################################################################

if bt == "shelf" and scene == "1":
	path = shelf_path + "/scenario1"
	print "--------Welcome to " + bt + " sceneario " + scene + "--------"
	try:
		os.makedirs(path)
	except OSError:
		print "Creation of the directory %s falied\n" % path
	else:
		pass
	### generate hypothesis (including the true pose) for each objects
	### 8 objects includes baseball as the target object
	####################################################################################################################
	Objects = dict()
	### objIdx, meshFile, objectName, objectRole, scale, true_pos, true_angles(radians), mass, nHypos
	Objects[0] = [0, "/mesh/dasani_water_bottle/dasani_water_bottle.obj", "waterBottle", 
		"target", 1.7, [0.0362, 0.33, 0.17+middleflatPosition[2]+flat_dim[2]/2], [math.pi/2, 0.0, 0.0], 4.2, nHypos]
	Objects[1] = [1, "/mesh/soft_white_lightbulb/soft_white_lightbulb.obj", "lightbulb", 
		"normal", 1.5, [-0.12, 0.18, 0.07+middleflatPosition[2]+flat_dim[2]/2], [math.pi/2, 0.0, math.pi/5.7], 2.2, nHypos]
	Objects[2] = [2, "/mesh/elmers_washable_no_run_school_glue/elmers_washable_no_run_school_glue.obj", "glue", 
		"normal", 1.5, [-0.065, 0.44, 0.1+middleflatPosition[2]+flat_dim[2]/2], [math.pi / 2, 0.0, 25*math.pi/180], 1.8, nHypos]
	Objects[3] = [3, "/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", "tissueBox", 
		"normal", 1.5, [-0.2, -0.31, 0.06+middleflatPosition[2]+flat_dim[2]/2], [0.0, 0.0, -math.pi/1.2], 3.5, nHypos]
	Objects[4] = [4, "/mesh/dove_beauty_bar/dove_beauty_bar.obj", "doveBar", 
		"normal", 2, [0.1, 0.24, 0.05+shelfbase_dim[2]], [math.pi/2, 0.0, math.pi/4.3], 4.0, nHypos]
	Objects[5] = [5, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
		"normal", 2, [-0.2, 0.36, 0.07+shelfbase_dim[2]], [3*math.pi/5.5, 7*math.pi/6.2, math.pi/1.9], 3.2, nHypos]
	Objects[6] = [6, "/mesh/up_glucose_bottle/up_glucose_bottle.obj", "glucoseBottle", 
		"normal", 1.5, [-0.23, -0.43, 0.1+shelfbase_dim[2]], [math.pi / 2, 0.0, math.pi], 2.6, nHypos]
	Objects[7] = [7, "/mesh/dr_browns_bottle_brush/dr_browns_bottle_brush.obj", "brush", 
		"normal", 1, [0.07, -0.32, 0.02+shelfbase_dim[2]], [0.0, 0.0, -math.pi/3.1], 1.7, nHypos]
	### pick goal offset
	goalPos_offset = [-0.12, 0.0, 0.0]
	goalEuler = [math.pi/2, 0.0, 0.0] ### side picking
	x_ll = kukaBasePosition[0] - standingBase_dim[0] / 2 - 0.2
	x_ul = shelfbackPosition[0] - shelfback_dim[0] / 2
	y_ll = middleflankPosition[1] + flank_dim[1] / 2
	y_ul = leftflankPosition[1] - flank_dim[1] / 2
	z_ll = middleflatPosition[2] + flat_dim[2] / 2
	z_ul = topflatPosition[2] - flat_dim[2] / 2
	####################################################################################################################

if bt == "shelf" and scene == "2":
	path = shelf_path + "/scenario2"
	print "--------Welcome to " + bt + " sceneario " + scene + "--------"
	try:
		os.makedirs(path)
	except OSError:
		print "Creation of the directory %s falied\n" % path
	else:
		pass
	### generate hypothesis (including the true pose) for each objects
	### 11 objects includes baseball as the target object
	####################################################################################################################
	Objects = dict()
	### objIdx, meshFile, objectName, objectRole, scale, true_pos, true_angles(radians), mass, nHypos
	Objects[0] = [0, "/mesh/dasani_water_bottle/dasani_water_bottle.obj", "waterBottle", 
		"target", 1.8, [0.152, 0.45, 0.2+middleflatPosition[2]+flat_dim[2]/2], [math.pi/2, 0.0, -math.pi/5.2], 4.2, nHypos]
	Objects[1] = [1, "/mesh/dove_beauty_bar/dove_beauty_bar.obj", "doveBar", 
		"normal", 2, [-0.2, 0.43, 0.05+middleflatPosition[2]+flat_dim[2]/2], [math.pi/2, 0.0, math.pi/2.9], 4.0, nHypos]
	Objects[2] = [2, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
		"normal", 2, [0.15, -0.39, 0.07+shelfbase_dim[2]], [3*math.pi/4.0, 2.5*math.pi, math.pi/6.2], 3.2, nHypos]
	Objects[3] = [3, "/mesh/ticonderoga_12_pencils/ticonderoga_12_pencils.obj", "pencils", 
		"normal", 2, [0.05, -0.26, 0.01+middleflatPosition[2]+flat_dim[2]/2], [0.0, -math.pi/2, -math.pi/3.9], 3.4, nHypos]
	Objects[4] = [4, "/mesh/elmers_washable_no_run_school_glue/elmers_washable_no_run_school_glue.obj", "glue", 
		"normal", 1.5, [-0.19, -0.48, 0.11+middleflatPosition[2]+flat_dim[2]/2], [math.pi / 2, 0.0, 25*math.pi/180], 1.8, nHypos]
	Objects[5] = [5, "/mesh/folgers_classic_roast_coffee/folgers_classic_roast_coffee.obj", "coffeeJar", 
		"normal", 2, [-0.2, -0.24, 0.12+shelfbase_dim[2]], [math.pi/2, 0.0, -math.pi/3.9], 5.6, nHypos]
	Objects[6] = [6, "/mesh/up_glucose_bottle/up_glucose_bottle.obj", "glucoseBottle", 
		"normal", 1.5, [-0.08, 0.3, 0.1+middleflatPosition[2]+flat_dim[2]/2], [math.pi / 2, 0.0, math.pi], 2.6, nHypos]
	Objects[7] = [7, "/mesh/crayola_24_ct/crayola_24_ct.obj", "crayola", 
		"normal", 2, [-0.3, 0.24, 0.1+middleflatPosition[2]+flat_dim[2]/2], [0.0, 0.0, -math.pi/6.1], 3.8, nHypos]
	Objects[8] = [8, "/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", "tissueBox", 
		"normal", 1.5, [-0.2, 0.34, 0.085+shelfbase_dim[2]], [0.0, 0.0, -math.pi/1.2], 3.5, nHypos]
	Objects[9] = [9, "/mesh/soft_white_lightbulb/soft_white_lightbulb.obj", "lightbulb", 
		"phantom", 1.5, [0.14, 0.18, 0.05+middleflatPosition[2]+flat_dim[2]/2], [0.0, 0.0, math.pi/6.0], 2.2, 1]
	Objects[10] = [10, "/mesh/dr_browns_bottle_brush/dr_browns_bottle_brush.obj", "brush", 
		"invisible", 1, [0.09, -0.21, 0.03+shelfbase_dim[2]], [0.0, 0.0, -math.pi / 3.1], 1.7, 1]
	### pick goal offset
	goalPos_offset = [-0.12, 0.0, 0.04]
	goalEuler = [math.pi/2, 0.0, 0.0] ### side picking
	x_ll = kukaBasePosition[0] - standingBase_dim[0] / 2 - 0.2
	x_ul = shelfbackPosition[0] - shelfback_dim[0] / 2
	y_ll = middleflankPosition[1] + flank_dim[1] / 2
	y_ul = leftflankPosition[1] - flank_dim[1] / 2
	z_ll = middleflatPosition[2] + flat_dim[2] / 2
	z_ul = topflatPosition[2] - flat_dim[2] / 2
	####################################################################################################################

### At here we finish selecting the specific scene for specific benchmark type
### generate ground truth
startTime = time.clock()
truePoses, nObjectInExecuting = utils_revamp.trueScene_generation(bt, scene, Objects, executingServer)
print "Time for generating the ground truth" + str(time.clock() - startTime) + " second."


# if mode == "g":
# 	### Now generate planning scene based on nHypos and noiseLevel
# 	hypotheses, nObjectInPlanning = utils_revamp.planScene_generation(Objects, bt, known_geometries_planning, nHypos, 
# 												transErrors[noiseLevel-1], orientErrors[noiseLevel-1], planningServer)
'''

time.sleep(10000)