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

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
kuka_ee_idx = 8
static_geometries = []

# Introduce Kuka arm 
kukaID = p.loadURDF("kuka.urdf", useFixedBase=True)
static_geometries.append(kukaID)


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


nHypo = 1

# generate the static geometries
######################################################################
if sys.argv[1] == "table":
	print "---------Enter to table scene!---------"
	home_configuration = [0,0,0,-np.pi/2,0,np.pi/2,0]
	for i in range(1,8):
		result = p.resetJointState(kukaID,i,home_configuration[i-1])
	#reset the base of Kuka
	p.resetBasePositionAndOrientation(kukaID, [-0.7, 0, 0], [0, 0, 0, 1])
	# create the static geometries - table
	table_dim = np.array([0.7, 1.3, 0.45])
	tablePosition=[0, 0, table_dim[2]/2]
	table_c = p.createCollisionShape(shapeType=p.GEOM_BOX,
							halfExtents=table_dim/2)
	table_v = p.createVisualShape(shapeType=p.GEOM_BOX,
							halfExtents=table_dim/2)
	tableM = p.createMultiBody(baseCollisionShapeIndex=table_c,
					baseVisualShapeIndex=table_v,basePosition=tablePosition)
	static_geometries.append(tableM)
######################################################################	
	if sys.argv[2] == "1":
		# generate hypothesis (including the true pose) for each objects
		# 5 objects includes baseball as the target object
		##########################################################################
		Objects = dict()
		# meshfile, meshType, objectRole, scale, true_pos, true_angles, uncertainty, nHypo
		Objects[0] = [0, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
			"target", 2, [0.2, -0.21, 0.07+table_dim[2]], [0.0, 0.0, 0.0], [0.05, 0.05], nHypo]
		Objects[1] = [1, "/mesh/crayola_24_ct/crayola_24_ct.obj", 
		"crayola", "normal", 2, [0.15, 0.0, 0.1+table_dim[2]], 
												[math.pi/2, 0.0, 0.0], [0.04, 0.03, 0.18], nHypo]
		Objects[2] = [2, "/mesh/dasani_water_bottle/dasani_water_bottle.obj", "waterBottle", 
			"normal", 2, [-0.15, -0.29, 0.2+table_dim[2]], 
												[math.pi/2, 0.0, 0.0], [0.05, 0.04, 0.3], nHypo]
		Objects[3] = [3, "/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", 
		"tissueBox", "normal", 1.5, [0.05, -0.42, 0.1+table_dim[2]], 
										[0.0, math.pi/2, -math.pi/5], [0.03, 0.03, 0.24], nHypo]
		Objects[4] = [4, "/mesh/dove_beauty_bar/dove_beauty_bar.obj", "doveBar", 
			"normal", 2, [-0.12, 0.13, 0.05+table_dim[2]], 
										[math.pi/2, 0.0, math.pi/6.7], [0.03, 0.06, 0.01], nHypo]
		# pick goal offset												
		goalPos_offset = [-0.03, 0.0, 0.12]

	elif sys.argv[2] == "2":
		# generate hypothesis (including the true pose) for each objects
		# 7 objects includes baseball as the target object
		##########################################################################
		Objects = dict()
		# meshfile, meshType, objectRole, scale, true_pos, true_angles, uncertainty, nHypo
		Objects[0] = [0, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
			"target", 2, [0.13, -0.02, 0.07+table_dim[2]], [0.0, 0.0, 0.0], [0.05, 0.05], nHypo]
		Objects[1] = [1, "/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", 
					"tissueBox", "normal", 1.5, [0.05, 0.16, 0.1+table_dim[2]], 
										[0.0, math.pi/2, -math.pi/3], [0.03, 0.03, 0.24], nHypo]
		Objects[2] = [2, "/mesh/crayola_24_ct/crayola_24_ct.obj", 
					"crayola", "normal", 2, [0.15, -0.15, 0.1+table_dim[2]], 
										[math.pi/2, 0.0, -math.pi/4], [0.04, 0.03, 0.18], nHypo]
		# Objects[2] = [2, "/mesh/elmers_washable_no_run_school_glue/elmers_washable_no_run_school_glue.obj", 
		# 			"glue", "normal", 1.5, [0.05, -0.19, 0.33], [math.pi / 2, 
		#										0.0, 25*math.pi/180], [0.07, 0.07, 0.5], nHypo]
		Objects[3] = [3, "/mesh/dove_beauty_bar/dove_beauty_bar.obj", "doveBar", 
							"normal", 2, [-0.11, -0.19, 0.05+table_dim[2]], 
										[math.pi/2, 0.0, math.pi/1.3], [0.03, 0.06, 0.01], nHypo]
		Objects[4] = [4, "/mesh/dr_browns_bottle_brush/dr_browns_bottle_brush.obj", 
					"brush", "normal", 1, [0.17, -0.40, 0.02+table_dim[2]], 
											[0.0, 0.0, -math.pi / 4.5], [0.028, 0.02, 0.2], nHypo]
		Objects[5] = [5, "/mesh/soft_white_lightbulb/soft_white_lightbulb.obj", 
				"lightbulb", "phantom", 1.5, [-0.2, 0.36, 0.05+table_dim[2]], 
											[0.0, 0.0, math.pi/10.0], [0.032, 0.035, 0.06], nHypo]
		Objects[6] = [6, "/mesh/up_glucose_bottle/up_glucose_bottle.obj", 
				"glucoseBottle", "normal", 1.5, [0.2, 0.41, 0.1+table_dim[2]], 
											[math.pi / 2, 0.0, math.pi], [0.04, 0.04, 0.12], nHypo]
		# pick goal offset
		goalPos_offset = [-0.03, 0.0, 0.127]

	elif sys.argv[2] == "3":
		# generate hypothesis (including the true pose) for each objects
		# 10 objects includes baseball as the target object
		##########################################################################
		Objects = dict()
		# meshfile, meshType, objectRole, scale, true_pos, true_angles, uncertainty, nHypo
		Objects[0] = [0, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
				"target", 2, [0.16, 0.23, 0.05+table_dim[2]], [0.0, 0.0, 0.0], [0.05, 0.05], nHypo]
		# Objects[1] = [1, "/mesh/dove_beauty_bar/dove_beauty_bar.obj", "doveBar", 
		# 		"normal", 2, [0.26, 0.53, 0.05+table_dim[2]], [math.pi/2, 0.0, 
		# 												math.pi/4.3], [0.03, 0.06, 0.01], nHypo]
		# Objects[2] = [2, "/mesh/dasani_water_bottle/dasani_water_bottle.obj", "waterBottle", 
		# 		"normal", 2, [-0.09, 0.31, 0.2+table_dim[2]], [math.pi/2, 0.0, 
		# 												math.pi/2.6], [0.05, 0.04, 0.3], nHypo]
		# Objects[3] = [3, "/mesh/ticonderoga_12_pencils/ticonderoga_12_pencils.obj", "pencils", 
		# 		"normal", 1, [0.16, -0.08, 0.01+table_dim[2]], 
		# 							[0.0, math.pi/2, -math.pi/8.8], [0.05, 0.04, 0.25], nHypo]
		# Objects[4] = [4, "/mesh/elmers_washable_no_run_school_glue/elmers_washable_no_run_school_glue.obj", 
		# 		"glue", "invisible", 1.5, [0.03, 0.38, 0.11+table_dim[2]], 
		# 							[math.pi / 2, 0.0, 67*math.pi/180], [0.07, 0.07, 0.5], nHypo]
		# # Objects[5] = [5, "/mesh/folgers_classic_roast_coffee/folgers_classic_roast_coffee.obj", 
		# # 		"coffeeJar", "normal", 2, [-0.2, -0.09, 0.12+table_dim[2]], 
		# # 								[math.pi/2, 0.0, -math.pi/3.9], [0.026, 0.01, 0.24], nHypo]
		# Objects[5] = [5, "/mesh/up_glucose_bottle/up_glucose_bottle.obj", 
		# 		"glucoseBottle", "normal", 1.5, [0.26, 0.13, 0.1+table_dim[2]], 
		# 									[math.pi / 2, 0.0, math.pi], [0.04, 0.04, 0.12], nHypo]
		# Objects[6] = [6, "/mesh/crayola_24_ct/crayola_24_ct.obj", 
		# 		"crayola", "normal", 2, [-0.09, 0.16, 0.09+table_dim[2]], 
		# 										[math.pi/2, 0.0, math.pi/3], [0.04, 0.03, 0.18], nHypo]
		# Objects[7] = [7, "/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", 
		# 		"tissueBox", "normal", 1.5, [0.0, 0.0, 0.1+table_dim[2]], 
		# 									[0.0, math.pi/2, -math.pi/3], [0.03, 0.03, 0.24], nHypo]
		# Objects[8] = [8, "/mesh/dr_browns_bottle_brush/dr_browns_bottle_brush.obj", 
		# 		"brush", "normal", 1, [-0.07, -0.33, 0.02+table_dim[2]], 
		# 									[0.0, 0.0, -math.pi / 4.5], [0.028, 0.02, 0.2], nHypo]
		# Objects[9] = [9, "/mesh/soft_white_lightbulb/soft_white_lightbulb.obj", 
		# 		"lightbulb", "phantom", 1.5, [0.15, 0.45, 0.05+table_dim[2]], 
		# 									[0.0, 0.0, math.pi/10.0], [0.032, 0.035, 0.06], nHypo]
		# pick goal offset
		goalPos_offset = [-0.03, 0.0, 0.14]
###############################################################################################

if sys.argv[1] == "shelf": 
	print "---------Enter to shelf scene!---------"
	home_configuration = [0,0,0,0,0,0,0]
	for i in range(1,8):
		result = p.resetJointState(kukaID,i,home_configuration[i-1])
	#standingBase
	standingBase_dim = np.array([0.3, 0.3, 0.65])
	standingBase_c = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=standingBase_dim/2)
	standingBase_v = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=standingBase_dim/2)
	standingBasePosition=[-0.6, 0.0, standingBase_dim[2]/2]
	standingBaseM = p.createMultiBody(baseCollisionShapeIndex=standingBase_c,
					baseVisualShapeIndex=standingBase_v,basePosition=standingBasePosition)
	static_geometries.append(standingBaseM)
	print "standing base: " + str(standingBaseM)
	#reset the base of Kuka
	kukaBasePosition = [standingBasePosition[0], standingBasePosition[1], standingBase_dim[2]+0.005]
	kukaBaseOrientation = utils.euler_to_quaternion(-math.pi/2, math.pi, 0.0)
	# kukaBaseOrientation = utils.euler_to_quaternion(math.pi/2, math.pi, 0.0)
	# kukaBaseOrientation = utils.euler_to_quaternion(0.0, 0.0, 0.0)
	p.resetBasePositionAndOrientation(kukaID, kukaBasePosition, kukaBaseOrientation)
	# create the static geometries - shelf
	# shelfbase
	shelfbase_dim = np.array([0.7, 1.3, 0.3])
	shelfbase_c = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=shelfbase_dim/2)
	shelfbase_v = p.createVisualShape(shapeType=p.GEOM_BOX,
					halfExtents=shelfbase_dim/2, rgbaColor=[0.41, 0.41, 0.41, 1])
	shelfbasePosition = [0.0, 0.0, shelfbase_dim[2]/2]
	shelfbaseM = p.createMultiBody(baseCollisionShapeIndex=shelfbase_c,
					baseVisualShapeIndex=shelfbase_v,basePosition=shelfbasePosition)
	static_geometries.append(shelfbaseM)
	print "shelf base: " + str(shelfbaseM)
	# the shape and visual of the flank
	flank_dim = np.array([shelfbase_dim[0], 0.06, 1.2])
	flank_c = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=flank_dim/2)
	flank_v = p.createVisualShape(shapeType=p.GEOM_BOX,
					halfExtents=flank_dim/2, rgbaColor=[0.63, 0.32, 0.18, 1])
	#leftflank
	leftflankPosition = [0.0, shelfbase_dim[1]/2-flank_dim[1]/2, shelfbase_dim[2]+flank_dim[2]/2]
	leftflankM = p.createMultiBody(baseCollisionShapeIndex=flank_c,
					baseVisualShapeIndex=flank_v,basePosition=leftflankPosition)
	static_geometries.append(leftflankM)
	print "left flank: " + str(leftflankM)
	#rightflank
	rightflankPosition = [0.0, -shelfbase_dim[1]/2+flank_dim[1]/2, shelfbase_dim[2]+flank_dim[2]/2]
	rightflankM = p.createMultiBody(baseCollisionShapeIndex=flank_c,
					baseVisualShapeIndex=flank_v,basePosition=rightflankPosition)
	static_geometries.append(rightflankM)
	print "right flank: " + str(rightflankM)
	#middleflank
	middleflankPosition = [0.0, 0.0, shelfbase_dim[2]+flank_dim[2]/2]
	middleflankM = p.createMultiBody(baseCollisionShapeIndex=flank_c,
					baseVisualShapeIndex=flank_v,basePosition=middleflankPosition)
	static_geometries.append(middleflankM)
	print "middle flank: " + str(middleflankM)
	#the shape and visual of the flat
	flat_dim = np.array([shelfbase_dim[0], shelfbase_dim[1], 0.06])
	flat_c = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=flat_dim/2)
	flat_v = p.createVisualShape(shapeType=p.GEOM_BOX,
					halfExtents=flat_dim/2, rgbaColor=[0.41, 0.41, 0.41, 1])
	#middleflat
	middleflatPosition = [0.0, 0.0, shelfbase_dim[2]+flank_dim[2]/2]
	middleflatM = p.createMultiBody(baseCollisionShapeIndex=flat_c,
					baseVisualShapeIndex=flat_v,basePosition=middleflatPosition)
	static_geometries.append(middleflatM)	
	print "middle flat: " + str(middleflatM)
	#topflat
	topflatPosition = [0.0, 0.0, shelfbase_dim[2]+flank_dim[2]+flat_dim[2]/2]
	topflatM = p.createMultiBody(baseCollisionShapeIndex=flat_c,
					baseVisualShapeIndex=flat_v,basePosition=topflatPosition)
	static_geometries.append(topflatM)
	print "top flat: " + str(topflatM)
	#back
	shelfback_dim = np.array([0.02, shelfbase_dim[1], flank_dim[2]+flat_dim[2]])
	shelfback_c = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=shelfback_dim/2)
	shelfback_v = p.createVisualShape(shapeType=p.GEOM_BOX,
					halfExtents=shelfback_dim/2, rgbaColor=[0.63, 0.32, 0.18, 1])
	shelfbackPosition = [shelfbase_dim[0]/2-shelfback_dim[0]/2, 0.0, shelfbase_dim[2]+shelfback_dim[2]/2]
	shelfbackM = p.createMultiBody(baseCollisionShapeIndex=shelfback_c,
					baseVisualShapeIndex=shelfback_v,basePosition=shelfbackPosition)
	static_geometries.append(shelfbackM)
	print "shelf back: " + str(shelfbackM)
######################################################################
	if sys.argv[2] == "1":
		# generate hypothesis (including the true pose) for each objects
		# 8 objects includes baseball as the target object
		##########################################################################
		Objects = dict()
		# meshfile, meshType, objectRole, scale, true_pos, true_angles, uncertainty, nHypo
		Objects[0] = [0, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
			"target", 2, [0.04, 0.33, 0.07+middleflatPosition[2]+flat_dim[2]/2], 
														[0.0, 0.0, 0.0], [0.05, 0.05], nHypo]
		# Objects[1] = [1, "/mesh/soft_white_lightbulb/soft_white_lightbulb.obj", 
		# 	"lightbulb", "normal", 1.5, [-0.12, 0.25, 0.05+middleflatPosition[2]+flat_dim[2]/2], 
		# 											[0.0, 0.0, math.pi/10.0], [0.032, 0.035, 0.06], nHypo]
		# Objects[2] = [2, "/mesh/elmers_washable_no_run_school_glue/elmers_washable_no_run_school_glue.obj", 
		# 	"glue", "normal", 1.5, [-0.05, 0.51, 0.13+middleflatPosition[2]+flat_dim[2]/2], 
		# 								[math.pi / 2, 0.0, 25*math.pi/180], [0.07, 0.07, 0.5], nHypo]
		# Objects[3] = [3, "/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", 
		# 	"tissueBox", "normal", 1.5, [-0.2, -0.31, 0.1+middleflatPosition[2]+flat_dim[2]/2], 
		# 										[0.0, 0.0, -math.pi/1.2], [0.03, 0.03, 0.24], nHypo]
		# Objects[4] = [4, "/mesh/dove_beauty_bar/dove_beauty_bar.obj", "doveBar", 
		# 	"normal", 2, [0.1, 0.24, 0.05+shelfbase_dim[2]], 
		# 									[math.pi/2, 0.0, math.pi/4.3], [0.03, 0.06, 0.01], nHypo]
		# Objects[5] = [5, "/mesh/dasani_water_bottle/dasani_water_bottle.obj", "waterBottle", 
		# 	"normal", 2, [-0.2, 0.36, 0.07+shelfbase_dim[2]], 
		# 										[0.0, 0.0, -math.pi/5.2], [0.05, 0.04, 0.3], nHypo]
		# Objects[6] = [6, "/mesh/up_glucose_bottle/up_glucose_bottle.obj", 
		# 	"glucoseBottle", "normal", 1.5, [-0.23, -0.43, 0.1+shelfbase_dim[2]], 
		# 									[math.pi / 2, 0.0, math.pi], [0.04, 0.04, 0.12], nHypo]
		# Objects[7] = [7, "/mesh/dr_browns_bottle_brush/dr_browns_bottle_brush.obj", 
		# 	"brush", "normal", 1, [0.07, -0.32, 0.02+shelfbase_dim[2]], 
		# 									[0.0, 0.0, -math.pi / 3.1], [0.028, 0.02, 0.2], nHypo]
		# pick goal offset
		goalPos_offset = [0.0, 0.0, 0.13]
		goalEuler = [0.0, 0.0, 0.0]

	elif sys.argv[2] == "2":
		# generate hypothesis (including the true pose) for each objects
		# 11 objects includes baseball as the target object
		##########################################################################
		Objects = dict()
		# meshfile, meshType, objectRole, scale, true_pos, true_angles, uncertainty, nHypo
		Objects[0] = [0, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
			"target", 2, [-0.08, 0.3, 0.07+middleflatPosition[2]+flat_dim[2]/2], 
														[0.0, 0.0, 0.0], [0.05, 0.05], nHypo]
		Objects[1] = [1, "/mesh/dove_beauty_bar/dove_beauty_bar.obj", "doveBar", 
			"normal", 2, [-0.13, 0.14, 0.05+middleflatPosition[2]+flat_dim[2]/2], 
									[math.pi/2, 0.0, math.pi/2.9], [0.03, 0.06, 0.01], nHypo]
		Objects[2] = [2, "/mesh/dasani_water_bottle/dasani_water_bottle.obj", "waterBottle", 
			"normal", 2, [-0.2, 0.45, 0.2+middleflatPosition[2]+flat_dim[2]/2], 
									[math.pi/2, 0.0, -math.pi/5.2], [0.05, 0.04, 0.3], nHypo]
		Objects[3] = [3, "/mesh/ticonderoga_12_pencils/ticonderoga_12_pencils.obj", "pencils", 
			"normal", 2, [0.05, -0.26, 0.01+middleflatPosition[2]+flat_dim[2]/2], 
									[0.0, math.pi/2, -math.pi/3.9], [0.05, 0.04, 0.25], nHypo]
		Objects[4] = [4, "/mesh/elmers_washable_no_run_school_glue/elmers_washable_no_run_school_glue.obj", 
			"glue", "normal", 1.5, [-0.19, -0.48, 0.11+middleflatPosition[2]+flat_dim[2]/2], 
									[math.pi / 2, 0.0, 25*math.pi/180], [0.07, 0.07, 0.5], nHypo]
		Objects[5] = [5, "/mesh/folgers_classic_roast_coffee/folgers_classic_roast_coffee.obj", 
			"coffeeJar", "normal", 2, [-0.2, -0.24, 0.12+shelfbase_dim[2]], 
									[math.pi/2, 0.0, -math.pi/3.9], [0.026, 0.01, 0.24], nHypo]
		Objects[6] = [6, "/mesh/up_glucose_bottle/up_glucose_bottle.obj", 
			"glucoseBottle", "normal", 1.5, [0.15, -0.46, 0.1+shelfbase_dim[2]], 
										[math.pi / 2, 0.0, math.pi], [0.04, 0.04, 0.12], nHypo]
		Objects[7] = [7, "/mesh/crayola_24_ct/crayola_24_ct.obj", 
			"crayola", "normal", 2, [-0.11, 0.43, 0.07+shelfbase_dim[2]], 
											[0.0, 0.0, math.pi/3.1], [0.04, 0.03, 0.18], nHypo]
		Objects[8] = [8, "/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", 
			"tissueBox", "normal", 1.5, [-0.2, 0.2, 0.085+shelfbase_dim[2]], 
											[0.0, 0.0, -math.pi/1.2], [0.03, 0.03, 0.24], nHypo]
		Objects[9] = [9, "/mesh/dr_browns_bottle_brush/dr_browns_bottle_brush.obj", 
			"brush", "invisible", 1, [0.07, -0.32, 0.03+shelfbase_dim[2]], 
										[0.0, 0.0, -math.pi / 3.1], [0.028, 0.02, 0.2], nHypo]
		Objects[10] = [10, "/mesh/soft_white_lightbulb/soft_white_lightbulb.obj", 
			"lightbulb", "phantom", 1.5, [0.14, 0.18, 0.05+middleflatPosition[2]+flat_dim[2]/2], 
													[0.0, 0.0, math.pi/6.0], [0.032, 0.035, 0.06], nHypo]
		# pick goal offset
		goalPos_offset = [-0.0, 0.0, 0.12]



### Collect meshes for all the objects ###
meshDict = dict()
for i in xrange(len(Objects)):
	meshDict[i] = utils.createMesh(Objects[i][0], Objects[i][1], Objects[i][2], 
		Objects[i][3], Objects[i][4], Objects[i][5], Objects[i][6], Objects[i][7], Objects[i][8])

'''
# goal nodes
goal_pose_pos = []
for i in xrange(len(goalPos_offset)):
	# The goal object always has the index zero
	goal_pose_pos.append(Objects[0][5][i] + goalPos_offset[i])
goal_pose_quat = utils.euler_to_quaternion(goalEuler[0], goalEuler[1], goalEuler[2])

goalCollision = True
while goalCollision:
	q_goal = p.calculateInverseKinematics(kukaID, kuka_ee_idx, 
									goal_pose_pos, goal_pose_quat, ll, ul, jr, rp)
	for j in range(1,8):
		result = p.resetJointState(kukaID,j,q_goal[j-1])
	p.stepSimulation()
	# check collision for both static geometry & objects
	isCollision1 = utils.collisionCheck_staticG(kukaID, static_geometries)
	if isCollision1:
		pass
	else:
		isCollision2 = utils.collisionCheck_objects(kukaID, meshDict)
		if not isCollision2:
			goalCollision = False
	# if goalCollision:
	# 	## put the kuka arm back to home configuration for next IK solution
	# 	for i in range(1,8):
	# 		result = p.resetJointState(kukaID,i,home_configuration[i-1])
print "collision for the goal? " + " " + str(goalCollision) 
print "goal state: " + str(q_goal)
'''

'''
##############start sampling##################
f = open("kuka_"+ sys.argv[1] + sys.argv[2] + "_samples.txt", "w")

nodes = []

nsamples = 5000
# f.write(str(nsamples)+"\n")
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
	if isCollision == False:
		nodes.append(ikSolution)
		## write it into a roadmap file
		f.write(str(temp_counter) + " " + str(j1) + " " + str(j2) + " " \
			+ str(j3) + " " + str(j4) + " " + str(j5) + " " + str(j6) + " " \
			+ str(j7) + "\n")
		temp_counter += 1


############ connect neighbors to build roadmaps #############
connectivity = np.zeros((nsamples, nsamples))
tree = spatial.KDTree(nodes)
neighbors_const = 2 * math.e * (1 + 1/len(home_configuration))
num_neighbors = int(neighbors_const * math.log(nsamples))
if num_neighbors >= nsamples:
	num_neighbors = nsamples -1 
print "num_neighbors: " + str(num_neighbors)

f1 = open("kuka_"+ sys.argv[1] + sys.argv[2] + "_roadmap.txt", "w")
nedge = 0
# create a list to record the number of neighbors connected for each node
neighbors_counts = [0] * nsamples

# for each node
for i in xrange(len(nodes)):
	queryNode = nodes[i]
	knn = tree.query(queryNode, k=num_neighbors, p=2)
	# for each neighbor
	for j in xrange(len(knn[0])):
		if knn[1][j] == i or connectivity[i][knn[1][j]] == 1:
			# if the neighbor is the query node itself
			# or the connectivity has been checked before
			# then skip the edge checking procedure
			continue
		# Otherwise, check the edge validity
		# between the query node and the the current neighbor
		neighbor = nodes[knn[1][j]]
		isEdgeValid = utils.checkEdgeValidity(queryNode, neighbor, kukaID, 
															static_geometries)
		if isEdgeValid:
			# write this edge information with their cost into the txt file
			nedge += 1
			f1.write(str(i) + " " + str(knn[1][j]) + " " + str(knn[0][j]) + "\n")
			# update connectivity information
			connectivity[i][knn[1][j]] = 1
			connectivity[knn[1][j]][i] = 1
			# update the counting for neighbors
			neighbors_counts[i] += 1
			neighbors_counts[knn[1][j]] += 1
	if i % 100 == 0:
		print "finish connecting neighbors for node " + str(i)
# print neighbors_counts
print "finish all the neighbors"
######################################################################


###### query the start node and goal nodes ######
q_start = home_configuration
nodes.append(q_start)
f.write(str(temp_counter) + " " + str(q_start[0]) + " " + str(q_start[1]) + " " \
	+ str(q_start[2]) + " " + str(q_start[3]) + " " + str(q_start[4]) + " " + \
	str(q_start[5]) + " " + str(q_start[6]) + "\n")
temp_counter += 1
## connect the start to the roadmap
tree = spatial.KDTree(nodes)
queryNode = nodes[temp_counter-1]
knn = tree.query(queryNode, k=(nsamples-10), p=2)
# for each neighbor
for j in xrange(len(knn[1])):
	if knn[1][j] == (temp_counter-1):
		continue
	else:
		# check collision
		neighbor = nodes[knn[1][j]]
		isEdgeValid = utils.checkEdgeValidity(queryNode, neighbor, kukaID, 
															static_geometries)
		if isEdgeValid:
			# write this edge information with their cost into the txt file
			nedge += 1
			f1.write(str(temp_counter-1) + " " + str(knn[1][j]) + " " \
				+ str(knn[0][j]) + "\n")
			print "successfully connecting the start to the roadmap\n"
			break # we just need to connect it, not necessary

# goal nodes
## put the kuka arm back to home configuration for next IK solution
for i in range(1,8):
	result = p.resetJointState(kukaID,i,home_configuration[i-1])
goal_pose_pos = []
for i in xrange(len(goalPos_offset)):
	# The goal object always has the index zero
	goal_pose_pos.append(Objects[0][5][i] + goalPos_offset[i])
goal_pose_quat = utils.euler_to_quaternion(goalEuler[0], goalEuler[1], goalEuler[2])

goalCollision = True
while goalCollision:
	q_goal = p.calculateInverseKinematics(kukaID, kuka_ee_idx, goal_pose_pos, 
													goal_pose_quat, ll, ul, jr)
	for j in range(1,8):
		result = p.resetJointState(kukaID,j,q_goal[j-1])
	p.stepSimulation()
	# check collision for both static geometry & objects
	isCollision1 = utils.collisionCheck_staticG(kukaID, static_geometries)
	if isCollision1:
		pass
	else:
		isCollision2 = utils.collisionCheck_objects(kukaID, meshDict)
		if not isCollision2:
			goalCollision = False
	# if goalCollision:
	# 	## put the kuka arm back to home configuration for next IK solution
	# 	for i in range(1,8):
	# 		result = p.resetJointState(kukaID,i,home_configuration[i-1])
print "collision for the goal? " + " " + str(goalCollision)
print "goal state: " + str(q_goal)
## pass goal collision checker
nodes.append(q_goal)

f.write(str(temp_counter) + " " + str(q_goal[0]) + " " + str(q_goal[1]) + " " \
	+ str(q_goal[2]) + " " + str(q_goal[3]) + " " + str(q_goal[4]) + " " + \
	str(q_goal[5]) + " " + str(q_goal[6]))
temp_counter += 1

## connect the goal to the roadmap
tree = spatial.KDTree(nodes)
queryNode = nodes[temp_counter-1]
knn = tree.query(queryNode, k=(nsamples-10), p=2)
# for each neighbor
for j in xrange(len(knn[1])):
	if knn[1][j] == (temp_counter - 1):
		continue
	else:
		# check collision
		neighbor = nodes[knn[1][j]]
		isEdgeValid = utils.checkEdgeValidity(queryNode, neighbor, kukaID, 
															static_geometries)
		if isEdgeValid:
			# write this edge information with their cost into the txt file
			nedge += 1
			f1.write(str(temp_counter-1) + " " + str(knn[1][j]) + " " \
				+ str(knn[0][j]))
			print "successfully connecting the goal to the roadmap\n"
			break # we just need to connect it, not necessary
		else:
			print "The edge is not valid..." + str(j)

print (len(nodes))
'''


## execute the trajectory in the scene without the objects
traj_file = "kuka_shelf1_traj.txt"
utils.executeTrajectory(traj_file, kukaID)


time.sleep(10000)




'''
############# check IK solver ################
# generate goal state to test the feasibility of the ground truth scene
goal_pose_pos = []
for i in xrange(len(goalPos_offset)):
	goal_pose_pos.append(Objects[0][5][i] + goalPos_offset[i])

isCollision = True
while isCollision:
	q_goal = p.calculateInverseKinematics(kukaID, kuka_ee_idx, 
										goal_pose_pos, ll, ul, jr, rp)
	for j in range(1,8):
		result = p.resetJointState(kukaID,j,q_goal[j-1])
	p.stepSimulation()
	# check collision for both static geometry & objects
	isCollision1 = utils.collisionCheck_staticG(kukaID, static_geometries)
	if isCollision1:
		pass
	else:
		isCollision2 = utils.collisionCheck_objects(kukaID, meshDict)
		if not isCollision2:
			isCollision = False
	if isCollision:
		## put the kuka arm back to home configuration for next IK solution
		for i in range(1,8):
			result = p.resetJointState(kukaID,i,home_configuration[i-1])


############# collision checker for samples ##################
# let's look at these joint configurations (if they are valid or feasible)
n_line = 0
f = open("kuka_"+ sys.argv[1] + "_" + sys.argv[2] + "_samples.txt", "r")
for line in f:
	line = line.split()
	n_line += 1

	if (n_line >=2):
		## These are the lines to read all the samples (joint configurations)
		idx = int(line[0])
		temp_j1 = float(line[1])
		temp_j2 = float(line[2])
		temp_j3 = float(line[3])
		temp_j4 = float(line[4])
		temp_j5 = float(line[5])
		temp_j6 = float(line[6])
		temp_j7 = float(line[7])
		temp_ikSolution = [temp_j1, temp_j2, temp_j3, 
							temp_j4, temp_j5, temp_j6, temp_j7]
		for j in range(1, 8):
			result = p.resetJointState(kukaID,j,temp_ikSolution[j-1])
		p.stepSimulation()
		time.sleep(0.1)

'''