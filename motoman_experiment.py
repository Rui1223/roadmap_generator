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
planningServer = p.connect(p.GUI)
executingServer = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
### set the real-time physics simulation ###
# p.setGravity(0.0, 0.0, -9.8, executingServer)
# p.setRealTimeSimulation(1, executingServer)
known_geometries_planning = []
known_geometries_executing = []

### Introduce Motoman arm ###
motomanID_p = p.loadURDF("motoman.urdf", useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION, physicsClientId=planningServer)
motomanID_e = p.loadURDF("motoman.urdf", useFixedBase=True, physicsClientId=executingServer)
known_geometries_planning.append(motomanID_p)
known_geometries_executing.append(motomanID_e)
### preserve the following five lines for test purposes ###
# print "Motoman Robot: " + str(motomanID_p)
# num_joints = p.getNumJoints(motomanID_p, planningServer)
# print "Num of joints: " + str(num_joints)
# for i in range(num_joints):
# 	print(p.getJointInfo(motomanID_p, i, planningServer))


########## information related to Motoman ###########
motoman_ee_idx = 10
### There is a torso joint which connect the lower and upper body (-2.957 ~ 2.957)
### But so far we decide to make that torso joint fixed
### For each arm, there are 10 joints and 7 of them are revolute joints
### There are total 14 revolute joints for each arm
### lower limits for null space
ll = [-3.13, -1.90, -2.95, -2.36, -3.13, -1.90, -3.13, -3.13, -1.90, -2.95, -2.36, -3.13, -1.90, -3.13]
### upper limits for null space
ul = [3.13, 1.90, 2.95, 2.36, 3.13, 1.90, 3.13, 3.13, 1.90, -2.95, 2.36, 3.13, 1.90, 3.13]
### joint ranges for null space
jr = [6.26, 3.80, 5.90, 4.72, 6.26, 3.80, 6.26, 6.26, 3.80, 5.90, 4.72, 6.26, 3.80, 6.26]
### restposes for null space
rp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


### set up several parameter before generating the benchmark ###
bt = sys.argv[1] # choose "table" or "shelf"
scene = sys.argv[2] # choose "1" or "2"
nHypos = int(sys.argv[3]) # choose from (1-7)
noiseLevel = int(sys.argv[4]) # choose from (1-7)
transErrors = [0.005, 0.01, 0.015, 0.02, 0.025, 0.03, 0.035]
orientErrors = [5, 10, 15, 20, 25, 30, 35]
orientErrors = [float(format(ii * math.pi/180, '.3f')) for ii in orientErrors]
nsamples = int(sys.argv[5]) # choose from (1000-5000), if the sampling is heuristic, 1000-2000 may be enough


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
	table_dim = np.array([0.555, 1.11, 0.59])
	tablePosition = [0, 0, table_dim[2]/2]
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
	### create the known geometries - standingBase  ###
	standingBase_dim = np.array([0.915, 0.62, 0.19])
	standingBasePosition = [tablePosition[0]-table_dim[0]/2-standingBase_dim[0]/2-0.01, 
																tablePosition[1], standingBase_dim[2]/2]
	standingBase_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX, 
								halfExtents=standingBase_dim/2, physicsClientId=planningServer)
	standingBase_v_p = p.createVisualShape(shapeType=p.GEOM_BOX, 
								halfExtents=standingBase_dim/2, physicsClientId=planningServer)
	standingBaseM_p = p.createMultiBody(baseCollisionShapeIndex=standingBase_c_p, baseVisualShapeIndex=standingBase_v_p,
								basePosition=standingBasePosition, physicsClientId=planningServer)
	standingBase_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX, 
								halfExtents=standingBase_dim/2, physicsClientId=executingServer)
	standingBase_v_e = p.createVisualShape(shapeType=p.GEOM_BOX, 
								halfExtents=standingBase_dim/2, physicsClientId=executingServer)
	standingBaseM_e = p.createMultiBody(baseCollisionShapeIndex=standingBase_c_e, baseVisualShapeIndex=standingBase_v_e,
								basePosition=standingBasePosition, physicsClientId=executingServer)
	known_geometries_planning.append(standingBaseM_p)
	known_geometries_executing.append(standingBaseM_e)
	print "standing base: " + str(standingBaseM_e)
	### reset the base of motoman
	motomanBasePosition = [standingBasePosition[0], standingBasePosition[1], 
													standingBasePosition[2]+standingBase_dim[2]/2+0.005]
	motomanBaseOrientation = [0, 0, 0, 1]
	p.resetBasePositionAndOrientation(motomanID_p, motomanBasePosition, 
									motomanBaseOrientation, physicsClientId=planningServer)
	p.resetBasePositionAndOrientation(motomanID_e, motomanBasePosition, 
									motomanBaseOrientation, physicsClientId=executingServer)
	### set motoman home configuration
	home_configuration = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

else:
	shelf_path = "newChapter/shelf"
	try:
		os.mkdir(shelf_path)
	except OSError:
		print "Creation of the directory %s falied\n" % shelf_path
	else:
		pass
	print "---------Enter to shelf scene!----------"
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
	home_configuration = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

########################################################################################################################

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

	### Try different YCB objects in the scene to see the time for importing
	###################################################################################################################
	Objects = dict()
	### objIdx, meshFile, objectName, objectRole, scale, true_pos, 
	### true_angles(radians), mass, nHypos, prob(not in the scene)
	Objects[0] = [0, "/mesh/009_gelatin_box/gelatin_box_512_reduced.obj", "gelatin box", 
		"target", 1, [0.0, 0.29, 0.002+table_dim[2]], [0, 0, math.pi/7], 1.4, nHypos, 0.08]
	# Objects[0] = [0, "/mesh/006_mustard_bottle_64/mustard_bottle_64.obj", "mustard bottle",
	# 	"target", 1, [0.2, 0.2, 0.1+table_dim[2]], [0, 0, 0], 2.1, nHypos, 0.16]
	Objects[1] = [1, "/mesh/006_mustard_bottle_512/mustard_bottle_512_reduced.obj", "mustard bottle",
		"normal", 1, [-0.145, 0.37, 0.005+table_dim[2]], [0, 0, math.pi/4], 2.1, nHypos, 0.16]
	# Objects[0] = [0, "/mesh/006_mustard_bottle_512/mustard_bottle_512.obj", "mustard bottle",
	# 	"normal", 1, [0.4, 0.4, 0.1+table_dim[2]], [0, 0, 0], 2.1, nHypos, 0.16]
	Objects[2] = [2, "/mesh/021_bleach_cleanser/bleach_cleanser_512_reduced.obj", "bleach cleanser",
		"normal", 1, [0.01, 0.37, 0.004+table_dim[2]], [0, 0, 0], 4.2, nHypos, 0.11]
	Objects[3] = [3, "/mesh/003_cracker_box/cracker_box_512_reduced.obj", "cracker box",
		"normal", 1, [-0.02, 0.18, 0.005+table_dim[2]], [0, 0, math.pi/2], 4.8, nHypos, 0.10]
	Objects[4] = [4, "/mesh/008_pudding_box/pudding_box_512_reduced.obj", "pudding box",
		"phantom", 1, [0.05, 0.33, 0.002+table_dim[2]], [0, 0, -3*math.pi/4], 1.4, nHypos, 0.86]

	### pick goal offset
	goalPos_offset = [0.0, 0.0, 0.05]
	goalEuler = [0.0, math.pi, 0.0] ### overhan grasps
	x_ll = motomanBasePosition[0] - 0.35
	x_ul = tablePosition[0] + table_dim[0]/2
	y_ll = -table_dim[1]/2
	y_ul = table_dim[1]/2
	z_ll = table_dim[2]
	z_ul = table_dim[2] + 0.9






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
	goalPos_offset = [0.0, 0.0, 0.08]
	goalEuler = [0.0, math.pi, 0.0] ### tabletop picking
	x_ll = motomanBasePosition[0] - 0.35
	x_ul = tablePosition[0] + table_dim[0]/2
	y_ll = -table_dim[1]/2
	y_ul = table_dim[1]/2
	z_ll = table_dim[2]
	z_ul = table_dim[2] + 0.9
	####################################################################################################################

# if bt == "shelf" and scene == "1":
# 	path = shelf_path + "/scenario2"
# 	print "--------Welcome to " + bt + " sceneario " + scene + "--------"
# 	try:
# 		os.makedirs(path)
# 	except OSError:
# 		print "Creation of the directory %s falied\n" % path
# 	else:
# 		pass
# 	### generate hypothesis (including the true pose) for each objects
# 	### 11 objects includes baseball as the target object
# 	####################################################################################################################
# 	Objects = dict()
# 	### objIdx, meshFile, objectName, objectRole, scale, true_pos, true_angles(radians), mass, nHypos
# 	Objects[0] = [0, "/mesh/dasani_water_bottle/dasani_water_bottle.obj", "waterBottle", 
# 		"target", 1.8, [0.152, 0.45, 0.2+middleflatPosition[2]+flat_dim[2]/2], [math.pi/2, 0.0, -math.pi/5.2], 4.2, nHypos]
# 	Objects[1] = [1, "/mesh/dove_beauty_bar/dove_beauty_bar.obj", "doveBar", 
# 		"normal", 2, [-0.2, 0.43, 0.05+middleflatPosition[2]+flat_dim[2]/2], [math.pi/2, 0.0, math.pi/2.9], 4.0, nHypos]
# 	Objects[2] = [2, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
# 		"normal", 2, [0.15, -0.39, 0.07+shelfbase_dim[2]], [3*math.pi/4.0, 2.5*math.pi, math.pi/6.2], 3.2, nHypos]
# 	Objects[3] = [3, "/mesh/ticonderoga_12_pencils/ticonderoga_12_pencils.obj", "pencils", 
# 		"normal", 2, [0.05, -0.26, 0.01+middleflatPosition[2]+flat_dim[2]/2], [0.0, -math.pi/2, -math.pi/3.9], 3.4, nHypos]
# 	Objects[4] = [4, "/mesh/elmers_washable_no_run_school_glue/elmers_washable_no_run_school_glue.obj", "glue", 
# 		"normal", 1.5, [-0.19, -0.48, 0.11+middleflatPosition[2]+flat_dim[2]/2], [math.pi / 2, 0.0, 25*math.pi/180], 1.8, nHypos]
# 	Objects[5] = [5, "/mesh/folgers_classic_roast_coffee/folgers_classic_roast_coffee.obj", "coffeeJar", 
# 		"normal", 2, [-0.2, -0.24, 0.12+shelfbase_dim[2]], [math.pi/2, 0.0, -math.pi/3.9], 5.6, nHypos]
# 	Objects[6] = [6, "/mesh/up_glucose_bottle/up_glucose_bottle.obj", "glucoseBottle", 
# 		"normal", 1.5, [-0.08, 0.3, 0.1+middleflatPosition[2]+flat_dim[2]/2], [math.pi / 2, 0.0, math.pi], 2.6, nHypos]
# 	Objects[7] = [7, "/mesh/crayola_24_ct/crayola_24_ct.obj", "crayola", 
# 		"normal", 2, [-0.3, 0.24, 0.1+middleflatPosition[2]+flat_dim[2]/2], [0.0, 0.0, -math.pi/6.1], 3.8, nHypos]
# 	Objects[8] = [8, "/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", "tissueBox", 
# 		"normal", 1.5, [-0.2, 0.34, 0.085+shelfbase_dim[2]], [0.0, 0.0, -math.pi/1.2], 3.5, nHypos]
# 	Objects[9] = [9, "/mesh/soft_white_lightbulb/soft_white_lightbulb.obj", "lightbulb", 
# 		"phantom", 1.5, [0.14, 0.18, 0.05+middleflatPosition[2]+flat_dim[2]/2], [0.0, 0.0, math.pi/6.0], 2.2, 1]
# 	Objects[10] = [10, "/mesh/dr_browns_bottle_brush/dr_browns_bottle_brush.obj", "brush", 
# 		"invisible", 1, [0.09, -0.21, 0.03+shelfbase_dim[2]], [0.0, 0.0, -math.pi / 3.1], 1.7, 1]
# 	### pick goal offset
# 	goalPos_offset = [-0.12, 0.0, 0.04]
# 	goalEuler = [math.pi/2, 0.0, 0.0] ### side picking
# 	x_ll = motomanBasePosition[0] - 0.35
# 	x_ul = shelfbackPosition[0] - shelfback_dim[0] / 2
# 	y_ll = middleflankPosition[1] + flank_dim[1] / 2
# 	y_ul = leftflankPosition[1] - flank_dim[1] / 2
# 	z_ll = middleflatPosition[2] + flat_dim[2] / 2
# 	z_ul = topflatPosition[2] - flat_dim[2] / 2
	####################################################################################################################

### At here we finish selecting the specific scene for specific benchmark type
### Now let's generate ground truth of the specific scene and specific benchmark
startTime = time.clock()
truePoses, nObjectInExecuting = utils_motoman.trueScene_generation(bt, scene, Objects, executingServer)
print "Time elapsed to load the objects as the ground truth: ", time.clock() - startTime
startTime = time.clock()
hypotheses, mostPromisingHypoIdxes, nObjectInPlanning = utils_motoman.planScene_generation(Objects, bt, 
			known_geometries_planning, transErrors[noiseLevel-1], orientErrors[noiseLevel-1], planningServer)
print "Time elapsed to load the objects in the planning scene: ", time.clock() - startTime
print "most promisings: " + str(mostPromisingHypoIdxes)

### Now we can generate "labelWeights.txt" file
currentlabelWeightFile = path + "/labelWeights.txt"
f_labelWeights = open(currentlabelWeightFile, "w")
for hypo in hypotheses:
	f_labelWeights.write(str(hypo.hypoIdx) + " " + str(hypo.objIdx) + " " + str(hypo.prob) + "\n")
f_labelWeights.close()
### Now we can generate "mostPromisingLabels.txt" file
currentMostPromisingLabelsFile = path + "/mostPromisingLabels.txt"
f_mostPromisingLabels = open(currentMostPromisingLabelsFile, "w")
for mphi in mostPromisingHypoIdxes:
	f_mostPromisingLabels.write(str(mphi) + " ")
f_mostPromisingLabels.write("\n")
f_mostPromisingLabels.close()


'''
##############################################roadmap generation########################################################
startTime  = time.clock()
### specify q_start and set of q_goal first
q_start = home_configuration
### generate goal configurations
goalSet = []
goalHypos = []
MaxGoalsPerPose = 5
MaxTrialsPerPose = 7
MaxTrialsPerEE = 7

### for each target hypotheses
for t_hp in xrange(Objects[0][8]):
	# print "***********For Hypo " + str(t_hp) + "***************"
	temp_goalsForThatHypo = []
	temp_survivalForThatHypo = []
	### specify the position of the goal pose for that particular target hypothsis
	goal_pose_pos = []
	for i in xrange(len(goalPos_offset)):
		goal_pose_pos.append(hypotheses[t_hp].pos[i] + goalPos_offset[i])
	# print "goal_pose_pos: " + str(goal_pose_pos)
	temp_trials_pose = 0
	while temp_trials_pose < MaxTrialsPerPose:
		# print "-----A new pose-----"
		for j in range(1, 8):
			result_p = p.resetJointState(motomanID_p, j, home_configuration[j-1], physicsClientId=planningServer)
		for j in range(11, 18):
			result_p = p.resetJointState(motomanID_p, j, home_configuration[j-4], physicsClientId=planningServer)
		### specify the quaternion of that particular goal pose
		temp_goal_pose_quat = p.getQuaternionFromEuler([goalEuler[0], goalEuler[1], 
													goalEuler[2]+random.uniform(-math.pi, math.pi)])
		goal_pose_quat = [temp_goal_pose_quat[0], temp_goal_pose_quat[1], temp_goal_pose_quat[3], temp_goal_pose_quat[2]]
		### now start to exploring the IKs
		temp_trials_ee = 0
		while temp_trials_ee < MaxTrialsPerEE:
			q_goal = p.calculateInverseKinematics(motomanID_p, motoman_ee_idx, goal_pose_pos, goal_pose_quat, 
																			ll, ul, jr, physicsClientId=planningServer)
			for j in range(1, 8):
				result_p = p.resetJointState(motomanID_p, j, q_goal[j-1], physicsClientId=planningServer)
			for j in range(11, 18):
				result_p = p.resetJointState(motomanID_p, j, q_goal[j-4], physicsClientId=planningServer)
			p.stepSimulation(planningServer)
			### check collision for robot self and known obstacles
			isCollisionSelf = utils_motoman.collisionCheck_selfCollision(motomanID_p, planningServer)
			isCollisionKnownObs = utils_motoman.collisionCheck_knownObs(motomanID_p, known_geometries_planning, 
																									planningServer)
			if isCollisionSelf or isCollisionKnownObs:
				# print "Collision with robot itself or known obstacles, ee +2!"
				temp_trials_ee += 2 ## This may be a bad pose, so let's check it in a quicker way
				# raw_input("Press Enter to continue")
				continue
			else:
				### check collision condition with all hypos of objects
				### (The opposite of the collision probability)
				collidedHypos = utils_motoman.collisionCheck_hypos(motomanID_p, hypotheses, planningServer)
				# print "Collide with Hypos: " + str(collidedHypos)
				### compute the survivability
				if t_hp in collidedHypos:
					### the end effector collides with the pose it deems as the target pose
					temp_survival = 0.0
					temp_trials_ee += 1
					# print "Survival: " + str(temp_survival)
					# raw_input("Press Enter to continue")
					continue
				else:
					temp_survival = 1.0
					collisionPerObj = [0.0] * nObjectInPlanning
					for ch in collidedHypos:
						if hypotheses[ch].objIdx != 0:
							collisionPerObj[hypotheses[ch].objIdx] += hypotheses[ch].prob
					for cpobs in collisionPerObj:
						temp_survival *= (1 - cpobs)
					# print "Survival: " + str(temp_survival)
					# raw_input("Press Enter to continue")
					### add the q_goals and its corresponding survivability
					temp_goalsForThatHypo.append(q_goal)
					temp_survivalForThatHypo.append(temp_survival)
					temp_trials_ee += 1
		### finish the current pose
		temp_trials_pose += 1

	### You are here since you finish generating poses for a particular hypothesis
	### sort temp_survivalForThatHypo and pick top (MaxGoalsPerPose) ones
	idx_rank = sorted( range(len(temp_survivalForThatHypo)), key=lambda k: temp_survivalForThatHypo[k], reverse=True )
	if len(idx_rank) >= MaxGoalsPerPose:
		### add top (MaxGoalsPerPose) ones
		for mm in xrange(MaxGoalsPerPose):
			goalSet.append(temp_goalsForThatHypo[idx_rank[mm]])
			goalHypos.append(t_hp)
print "goalHypos:" + str(goalHypos)


############# start sampling ##############
currentSamplesFile = path + "/samples.txt"
f_samples = open(currentSamplesFile, "w")
nodes = []
temp_counter = 0

while temp_counter < nsamples:
	### sample a cartesian ee pose and calculate the IK solution
	temp_x = float(format(random.uniform(x_ll, x_ul), '.2f'))
	temp_y = float(format(random.uniform(y_ll, y_ul), '.2f'))
	temp_z = float(format(random.uniform(z_ll, z_ul), '.2f'))
	ikSolution = p.calculateInverseKinematics(motomanID_p, motoman_ee_idx, 
									[temp_x, temp_y, temp_z], ll, ul, jr, physicsClientId=planningServer)
	for j in range(1, 8):
		result_p = p.resetJointState(motomanID_p, j, ikSolution[j-1], physicsClientId=planningServer)
	for j in range(11, 18):
		result_p = p.resetJointState(motomanID_p, j, ikSolution[j-4], physicsClientId=planningServer)
	p.stepSimulation(planningServer)
	### check collision for robot self and known obstacles
	isCollisionSelf = utils_motoman.collisionCheck_selfCollision(motomanID_p, planningServer)
	# if isCollisionSelf:
	# 	print "self collision occurs during the sampling"
	isCollisionKnownObs = utils_motoman.collisionCheck_knownObs(motomanID_p, known_geometries_planning, 
																							planningServer)
	if (not isCollisionSelf) and (not isCollisionKnownObs):
		nodes.append(ikSolution)
		### write it into a sample file
		f_samples.write(str(temp_counter) + " " + str(ikSolution[0]) + " " + str(ikSolution[1]) + " " \
			+ str(ikSolution[2]) + " " + str(ikSolution[3]) + " " + str(ikSolution[4]) + " " \
			+ str(ikSolution[5]) + " " + str(ikSolution[6]) + "\n")
		temp_counter += 1

############## connect neighbors to build roadmaps #############
startTime = time.clock()
connectivity = np.zeros((nsamples, nsamples))
tree = spatial.KDTree(nodes)
neighbors_const = 1.5 * math.e * (1 + 1/(len(home_configuration)/2))
num_neighbors = int(neighbors_const * math.log(nsamples))
if num_neighbors >= nsamples:
	num_neighbors = nsamples - 1
print "num_neighbors: " + str(num_neighbors)
currentRoadmapFile = path + "/roadmap.txt"
f_roadmap = open(currentRoadmapFile, "w")
### for each node
for i in xrange(len(nodes)):
	queryNode = nodes[i]
	knn = tree.query(queryNode, k=num_neighbors, p=2)
	### for each neighbor
	for j in xrange(len(knn[1])):
		if knn[1][j] == i or connectivity[i][knn[1][j]] == 1:
			### if the neighbor is the query node itself
			### or the connectivity has been checked before
			### then skip the edge checking procedure
			continue
		### Otherwise, check the edge validity (in terms of collision with robot itself and known obstacles)
		### between the query node and the the current neighbor
		neighbor = nodes[knn[1][j]]
		isEdgeValid = utils_motoman.checkEdgeValidity(queryNode, neighbor, motomanID_p, 
																known_geometries_planning, planningServer)
		if isEdgeValid:
			### write this edge information with their cost and labels into the txt file
			### It is a valid edge in terms of no collision with robot itself and known obstacles
			### Let's check the collision status for each hypothesis for the purpose of labeling
			temp_labels = utils_motoman.label_the_edge(queryNode, neighbor, motomanID_p, hypotheses, planningServer)
			f_roadmap.write(str(i) + " " + str(knn[1][j]) + " " + format(knn[0][j], '.4f') + " ")
			for tl in temp_labels:
				f_roadmap.write(str(tl) + " ")
			f_roadmap.write("\n")
			### update connectivity information
			connectivity[i][knn[1][j]] = 1
			connectivity[knn[1][j]][i] = 1

	if i % 100 == 99:
		print "finish labeling and connecting neighbors for node " + str(i)
print "finish all the neighbors"
print "Time elapsed: ", time.clock() - startTime
########################################################

############## add the start node and goal nodes to the roadmap ##########
nodes.append(q_start)
f_samples.write(str(temp_counter) + " " + str(q_start[0]) + " " + str(q_start[1]) + " " \
	+ str(q_start[2]) + " " + str(q_start[3]) + " " + str(q_start[4]) + " " + \
	str(q_start[5]) + " " + str(q_start[6]) + "\n")
temp_counter += 1
### connect the start to the roadmap
tree = spatial.KDTree(nodes)
queryNode = nodes[temp_counter-1]
knn = tree.query(queryNode, k=num_neighbors, p=2)
highestSurvival_neighborIdx = -1;
highestSurvivalSofar = -1.0
highestSurvival_labels = []
highestSurvival_cost = -1.0

### for each neighbor
for j in xrange(len(knn[1])):
	if knn[1][j] == (temp_counter-1):
		continue
	else:
		### check collision
		neighbor = nodes[knn[1][j]]
		isEdgeValid = utils_motoman.checkEdgeValidity(queryNode, neighbor, motomanID_p, 
																known_geometries_planning, planningServer)
		if isEdgeValid:
			### examine the survivability of the edge
			### first get the labels
			temp_labels = utils_motoman.label_the_edge(queryNode, neighbor, motomanID_p, hypotheses, planningServer)
			### compute the survivability
			temp_survival = 1.0
			collisionPerObj = [0.0] * nObjectInPlanning
			for tl in temp_labels:
				collisionPerObj[hypotheses[tl].objIdx] += hypotheses[tl].prob
			for cpobs in collisionPerObj:
				temp_survival *= (1 - cpobs)
			if temp_survival > highestSurvivalSofar:
				highestSurvivalSofar = temp_survival
				highestSurvival_neighborIdx = knn[1][j]
				highestSurvival_labels = temp_labels
				highestSurvival_cost = knn[0][j]
### finish all the survival computation for all k nearest neighbors
### Now connect the start to the one neighbor with the largest survival (lower cost for tie breaker)
if (highestSurvival_neighborIdx != -1):
	f_roadmap.write(str(temp_counter-1) + " " + str(highestSurvival_neighborIdx) + " " \
									+ format(highestSurvival_cost, '.4f') + " ")
	for hl in highestSurvival_labels:
		f_roadmap.write(str(hl) + " ")
	f_roadmap.write("\n")
	print "successfully connect the start to the roadmap\n"

### loop through goalSet
goalConnectSuccess = False
for q_goal, hypo in zip(goalSet, goalHypos):
	nodes.append(q_goal)
	f_samples.write(str(temp_counter) + " " + str(q_goal[0]) + " " + str(q_goal[1]) + " " \
		+ str(q_goal[2]) + " " + str(q_goal[3]) + " " + str(q_goal[4]) + " " + \
		str(q_goal[5]) + " " + str(q_goal[6]) + " " + str(hypo) + "\n")
	temp_counter += 1
	### connect the goal to the roadmap
	tree = spatial.KDTree(nodes)
	queryNode = nodes[temp_counter-1]
	knn = tree.query(queryNode, k=num_neighbors, p=2)	
	### for each neighbor
	for j in xrange(len(knn[1])):
		if knn[1][j] == (temp_counter - 1):
			continue
		else:
			### check collision
			neighbor = nodes[knn[1][j]]
			isEdgeValid = utils_motoman.checkEdgeValidity(queryNode, neighbor, motomanID_p, 
																known_geometries_planning, planningServer)
			if isEdgeValid:
				### examine the survivability of the edge
				### first get the labels
				temp_labels = utils_motoman.label_the_edge(queryNode, neighbor, motomanID_p, hypotheses, planningServer)
				f_roadmap.write( str(temp_counter-1) + " " + str(knn[1][j]) + " " + format(knn[0][j], '.4f') + " " )
				for temp_l in temp_labels:
					f_roadmap.write(str(temp_l) + " ")
				f_roadmap.write("\n")
				goalConnectSuccess = True
if goalConnectSuccess:
	print "successfully connect the goal to the roadmap\n"
f_samples.close()
f_roadmap.close()
print "roadmap generated with " + str(len(nodes)) + " nodes in " + str(time.clock() - startTime) + " second."
print len(nodes)
########################################################################################################################

############ call planning algorithms ################
print "start planning..."
executeFile = "../robust_planning_20_icra/main_motoman" + " " + str(sys.argv[1]) \
														+ " " + str(sys.argv[2]) + " " + str(sys.argv[5])
subprocess.call(executeFile, shell=True)
utils_motoman.executeAllTraj_example(home_configuration, motomanID_e, truePoses, path, executingServer)
'''
time.sleep(10000)















################## has to go back and solve self collision ############################
# # goal_pose_pos = [motomanBasePosition[0]+0.05, motomanBasePosition[1], 1.1]
# goal_pose_pos = [0.20, 0.15, 0.4]
# temp_goal_pose_quat = p.getQuaternionFromEuler([goalEuler[0], goalEuler[1], 
# 											goalEuler[2]+random.uniform(-math.pi, math.pi)])
# goal_pose_quat = [temp_goal_pose_quat[0], temp_goal_pose_quat[1], temp_goal_pose_quat[3], temp_goal_pose_quat[2]]
# q_goal = p.calculateInverseKinematics(motomanID_p, motoman_ee_idx, goal_pose_pos, goal_pose_quat, 
# 																	ll, ul, jr, physicsClientId=planningServer)
# for j in range(1, 8):
# 	result_p = p.resetJointState(motomanID_p, j, q_goal[j-1], physicsClientId=planningServer)
# for j in range(11, 18):
# 	result_p = p.resetJointState(motomanID_p, j, q_goal[j-4], physicsClientId=planningServer)
# p.stepSimulation(planningServer)
# isCollisionSelf = utils_motoman.collisionCheck_selfCollision(motomanID_p, planningServer)
# if isCollisionSelf:
# 	print "self collision occurs!"
# raw_input("put the arm back to home configuration by pressing enter...")
# for j in range(1, 8):
# 	result_p = p.resetJointState(motomanID_p, j, home_configuration[j-1], physicsClientId=planningServer)
# for j in range(11, 18):
# 	result_p = p.resetJointState(motomanID_p, j, home_configuration[j-4], physicsClientId=planningServer)
# p.stepSimulation(planningServer)
# isCollisionSelf = utils_motoman.collisionCheck_selfCollision(motomanID_p, planningServer)
# if isCollisionSelf:
# 	print "self collision occurs!"
