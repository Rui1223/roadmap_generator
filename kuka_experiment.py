from __future__ import division
import pybullet as p
import pybullet_data
import utils1

import math
import random
import time
import numpy as np

import sys
import os
import subprocess

from scipy import spatial
import cPickle as pickle

planningServer = p.connect(p.GUI)
executingServer = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
kuka_ee_idx = 8

#lower limits for null space
ll = [-2.967, -2.06, -2.96, -2.06, -2.96, -2.06, -3.05]
#upper limits for null space
ul = [2.967, 2.06, 2.96, 2.06, 2.96, 2.06, 3.05]
#joint ranges for null space
jr = [5.934, 4.12, 5.92, 4.12, 5.92, 4.12, 6.10]
#restposes for null space
rp = [0, 0, 0, -np.pi/2, 0, np.pi/2, 0]


static_geometries_planning = []
static_geometries_executing = []
# Introduce Kuka arm
kukaID_p = p.loadURDF("kuka.urdf", useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION or p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT, physicsClientId=planningServer)
kukaID_e = p.loadURDF("kuka.urdf", useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION or p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT, physicsClientId=executingServer)
# static_geometries_planning.append(kukaID_p)
static_geometries_executing.append(kukaID_e)
print "Kuka Robot: " + str(kukaID_e)

# num_joints = p.getNumJoints(kukaID_e, executingServer)
# print "Num of joints: " + str(num_joints)
# for i in range(num_joints):
# 	print(p.getJointInfo(kukaID_e, i, executingServer))


bt = sys.argv[1]
scene = sys.argv[2]
nHypo = int(sys.argv[3])
noiseLevel = float(sys.argv[4])
nsamples = int(sys.argv[5])

# generate the static geometries
######################################################################
if bt == "table":
	table_path = "example/table"
	try:
		os.mkdir(table_path)
	except OSError:
		print "Creation of the directory %s falied\n" % table_path
	else:
		pass

	print "---------Enter to table scene!---------"
	# create the static geometries - table
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
	static_geometries_planning.append(tableM_p)
	static_geometries_executing.append(tableM_e)
	print "table: " + str(tableM_e)
	#reset the base of Kuka
	kukaBasePosition = [-table_dim[0]/2-0.2, 0, 0]
	kukaBaseOrientation = [0, 0, 0, 1]
	p.resetBasePositionAndOrientation(kukaID_p, kukaBasePosition, 
									kukaBaseOrientation, physicsClientId=planningServer)
	p.resetBasePositionAndOrientation(kukaID_e, kukaBasePosition, 
									kukaBaseOrientation, physicsClientId=executingServer)
	# set Kuka home configuration
	home_configuration = [0,0,0,-np.pi/2,0,np.pi/2,0]
	for i in range(1,8):
		result_p = p.resetJointState(kukaID_p, i, home_configuration[i-1], 
															physicsClientId=planningServer)
		result_e = p.resetJointState(kukaID_e, i, home_configuration[i-1], 
															physicsClientId=executingServer)
######################################################################	
	# if scene == "3":
	# 	path = table_path + "/scenario3"
	# 	try:
	# 		os.mkdir(path)
	# 	except OSError:
	# 		print "Creation of the directory %s falied\n" % path
	# 	else:
	# 		pass

	# 	# generate hypothesis (including the true pose) for each objects
	# 	# 5 objects includes baseball as the target object
	# 	##########################################################################
	# 	Objects = dict()
	# 	# meshfile, meshType, objectRole, scale, true_pos, true_angles, uncertainty, nHypo
	# 	# Objects[0] = [0, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
	# 	# 	"target", 2, [0.17, -0.19, 0.07+table_dim[2]], [math.pi/2.5, math.pi/5.6, math.pi/3.4], 
	# 	# 																[0.05, 0.05, 1.3], nHypo]
	# 	# Objects[1] = [1, "/mesh/crayola_24_ct/crayola_24_ct.obj", 
	# 	# "crayola", "normal", 2, [0.15, -0.03, 0.1+table_dim[2]], 
	# 	# 								[math.pi/2, 0.0, -math.pi/2.5], [0.04, 0.03, 0.69], nHypo]
	# 	# Objects[2] = [2, "/mesh/dasani_water_bottle/dasani_water_bottle.obj", "waterBottle", 
	# 	# 	"normal", 2, [-0.15, -0.28, 0.2+table_dim[2]], 
	# 	# 										[math.pi/2, 0.0, 0.0], [0.05, 0.04, 0.36], nHypo]
	# 	# Objects[3] = [3, "/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", 
	# 	# "tissueBox", "normal", 1.5, [0.05, -0.34, 0.1+table_dim[2]], 
	# 	# 								[0.0, -math.pi/2, -math.pi/5], [0.03, 0.03, 0.34], nHypo]
	# 	# Objects[4] = [4, "/mesh/dove_beauty_bar/dove_beauty_bar.obj", "doveBar", 
	# 	# 	"normal", 2, [-0.12, 0.13, 0.05+table_dim[2]], 
	# 	# 								[math.pi/2, 0.0, math.pi/6.7], [0.03, 0.06, 0.34], nHypo]

	# 	Objects[0] = [0, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
	# 		"target", 2, [0.17, -0.19, 0.07+table_dim[2]], [math.pi/2.5, math.pi/5.6, math.pi/3.4], 
	# 											[0.013*noiseLevel, 0.006*noiseLevel, 0.3*noiseLevel], nHypo]
	# 	Objects[1] = [1, "/mesh/crayola_24_ct/crayola_24_ct.obj", 
	# 		"crayola", "normal", 2, [0.15, -0.03, 0.1+table_dim[2]], [math.pi/2, 0.0, -math.pi/2.5], 
	# 											[0.01*noiseLevel, 0.008*noiseLevel, 0.14*noiseLevel], nHypo]
	# 	Objects[2] = [2, "/mesh/dasani_water_bottle/dasani_water_bottle.obj", "waterBottle", 
	# 					"normal", 2, [-0.15, -0.28, 0.2+table_dim[2]], [math.pi/2, 0.0, 0.0], 
	# 											[0.017*noiseLevel, 0.01*noiseLevel, 0.13*noiseLevel], nHypo]
	# 	Objects[3] = [3, "/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", 
	# 		"tissueBox", "normal", 1.5, [0.05, -0.34, 0.1+table_dim[2]], [0.0, -math.pi/2, -math.pi/5], 
	# 											[0.008*noiseLevel, 0.006*noiseLevel, 0.015*noiseLevel], nHypo]
	# 	Objects[4] = [4, "/mesh/dove_beauty_bar/dove_beauty_bar.obj", "doveBar", 
	# 		"normal", 2, [-0.12, 0.13, 0.05+table_dim[2]], [math.pi/2, 0.0, math.pi/6.7], 
	# 											[0.008*noiseLevel, 0.015*noiseLevel, 0.08*noiseLevel], nHypo]
	# 	# pick goal offset												
	# 	goalPos_offset = [0.0, 0.0, 0.16]
	# 	goalEuler = [0.0, math.pi, 0.0] ## tabletop picking
	# 	x_ll = kukaBasePosition[0] - 0.2
	# 	x_ul = tablePosition[0] + table_dim[0]/2
	# 	y_ll = -table_dim[1]/2
	# 	y_ul = table_dim[1]/2
	# 	z_ll = table_dim[2]
	# 	z_ul = table_dim[2]+0.7


	if scene == "1":
		path = table_path + "/scenario1"
		try:
			os.mkdir(path)
		except OSError:
			print "Creation of the directory %s falied\n" % path
		else:
			pass

		# generate hypothesis (including the true pose) for each objects
		# 7 objects includes baseball as the target object
		##########################################################################
		Objects = dict()
		# meshfile, meshType, objectRole, scale, true_pos, true_angles, uncertainty, nHypo
		Objects[0] = [0, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
			"target", 2, [0.20, 0.0, 0.07+table_dim[2]], [math.pi/5.8, math.pi/0.4, math.pi/2.3], 
											[0.012, 0.013, 0.45]]
		Objects[1] = [1, "/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", 
			"tissueBox", "normal", 1.5, [0.06, 0.16, 0.1+table_dim[2]], [0.0, -math.pi/2, -math.pi/3], 
											[0.00875, 0.01875, 0.06]]
		Objects[2] = [2, "/mesh/crayola_24_ct/crayola_24_ct.obj", 
			"crayola", "normal", 2, [0.13, -0.14, 0.1+table_dim[2]], [math.pi/2, 0.0, math.pi/4], 
											[0.01, 0.01875, 0.018]]
		Objects[3] = [3, "/mesh/dove_beauty_bar/dove_beauty_bar.obj", "doveBar", 
			"normal", 2, [-0.16, 0.175, 0.09+table_dim[2]], [-math.pi, 0.0, math.pi/1.3], 
											[0.0075, 0.015, 0.0025]]
		Objects[4] = [4, "/mesh/dr_browns_bottle_brush/dr_browns_bottle_brush.obj", 
			"brush", "normal", 1, [0.17, -0.40, 0.02+table_dim[2]], [0.0, 0.0, -math.pi / 4.5], 
											[0.007, 0.005, 0.05]]
		Objects[5] = [5, "/mesh/up_glucose_bottle/up_glucose_bottle.obj", 
			"glucoseBottle", "normal", 1.5, [0.2, 0.38, 0.1+table_dim[2]], [math.pi / 2, 0.0, math.pi], 
											[0.01, 0.01, 0.03]]
		Objects[6] = [6, "/mesh/soft_white_lightbulb/soft_white_lightbulb.obj", 
			"lightbulb", "phantom", 1.5, [-0.14, 0.41, 0.05+table_dim[2]], [0.0, -math.pi, 9*math.pi/10.0], 
										[0.008, 0.00875, 0.015]]
		Objects[7] = [7, "/mesh/dasani_water_bottle/dasani_water_bottle.obj", "waterBottle", 
				"normal", 2, [-0.1, -0.18, 0.2+table_dim[2]], [math.pi/2, 0.0, 0.0], 
										[0.017, 0.01, 0.13]]
		# pick goal offset
		goalPos_offset = [0.0, 0.0, 0.16]
		goalEuler = [0.0, math.pi, 0.0] ## tabletop picking
		x_ll = kukaBasePosition[0] - 0.2
		x_ul = tablePosition[0] + table_dim[0]/2
		y_ll = -table_dim[1]/2
		y_ul = table_dim[1]/2
		z_ll = table_dim[2]
		z_ul = table_dim[2]+0.7

	elif scene == "2":
		path = table_path + "/scenario2"
		try:
			os.mkdir(path)
		except OSError:
			print "Creation of the directory %s falied\n" % path
		else:
			pass

		# generate hypothesis (including the true pose) for each objects
		# 10 objects includes baseball as the target object
		##########################################################################
		Objects = dict()
		# meshfile, meshType, objectRole, scale, true_pos, true_angles, uncertainty, nHypo
		Objects[0] = [0, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
				"target", 2, [0.16, 0.23, 0.07+table_dim[2]], 
						[2*math.pi/7.0, math.pi/4.8, 7*math.pi/6.2], [0.05, 0.05, 1.04]]
		Objects[1] = [1, "/mesh/dove_beauty_bar/dove_beauty_bar.obj", "doveBar", 
				"normal", 2, [0.26, 0.35, 0.05+table_dim[2]], [math.pi/2, 0.0, 
														math.pi/4.3], [0.03, 0.06, 0.01]]
		Objects[2] = [2, "/mesh/dasani_water_bottle/dasani_water_bottle.obj", "waterBottle", 
				"normal", 2, [-0.09, 0.31, 0.2+table_dim[2]], [math.pi/2, 0.0, 
														math.pi/2.6], [0.05, 0.04, 0.67]]
		Objects[3] = [3, "/mesh/ticonderoga_12_pencils/ticonderoga_12_pencils.obj", "pencils", 
				"normal", 1, [0.16, -0.08, 0.01+table_dim[2]], 
									[0.0, -math.pi/2, -math.pi/8.8], [0.05, 0.04, 0.32]]
		Objects[4] = [4, "/mesh/up_glucose_bottle/up_glucose_bottle.obj", 
				"glucoseBottle", "normal", 1.5, [0.26, 0.13, 0.1+table_dim[2]], 
											[math.pi / 2, 0.0, math.pi], [0.04, 0.04, 0.42]]
		Objects[5] = [5, "/mesh/crayola_24_ct/crayola_24_ct.obj", 
				"crayola", "normal", 2, [-0.09, 0.16, 0.09+table_dim[2]], 
											[math.pi/2, 0.0, math.pi/3], [0.04, 0.03, 0.18]]
		Objects[6] = [6, "/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", 
				"tissueBox", "normal", 1.5, [0.0, 0.0, 0.1+table_dim[2]], 
										[0.0, -math.pi/2, -math.pi/3], [0.03, 0.03, 0.24]]
		Objects[7] = [7, "/mesh/dr_browns_bottle_brush/dr_browns_bottle_brush.obj", 
				"brush", "normal", 1, [-0.21, -0.23, 0.02+table_dim[2]], 
											[0.0, 0.0, 1.7*math.pi / 4.5], [0.028, 0.02, 0.2]]
		Objects[8] = [8, "/mesh/soft_white_lightbulb/soft_white_lightbulb.obj", 
				"lightbulb", "phantom", 1.5, [0.12, 0.52, 0.05+table_dim[2]], 
										[0.0, math.pi, 4*math.pi/6.7], [0.032, 0.035, 0.06]]
		Objects[9] = [9, "/mesh/elmers_washable_no_run_school_glue/elmers_washable_no_run_school_glue.obj", 
				"glue", "normal", 1.5, [0.03, 0.31, 0.11+table_dim[2]], 
									[math.pi / 2, 0.0, 67*math.pi/180], [0.07, 0.07, 0.5]]
		# pick goal offset
		goalPos_offset = [0.0, 0.0, 0.16]
		goalEuler = [0.0, math.pi, 0.0] ## tabletop picking
		x_ll = kukaBasePosition[0] - 0.2
		x_ul = tablePosition[0] + table_dim[0]/2
		y_ll = -table_dim[1]/2
		y_ul = table_dim[1]/2
		z_ll = table_dim[2]
		z_ul = table_dim[2]+0.7
###############################################################################################

if bt == "shelf":
	shelf_path = "example/shelf"
	try:
		os.mkdir(shelf_path)
	except OSError:
		print "Creation of the directory %s falied\n" % shelf_path
	else:
		pass

	print "---------Enter to shelf scene!---------"
	#standingBase
	standingBase_dim = np.array([0.3, 0.3, 0.65])
	standingBasePosition=[-0.6, 0.0, standingBase_dim[2]/2]
	standingBase_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX, 
					halfExtents=standingBase_dim/2, physicsClientId=planningServer)
	standingBase_v_p = p.createVisualShape(shapeType=p.GEOM_BOX, 
					halfExtents=standingBase_dim/2, physicsClientId=planningServer)
	standingBaseM_p = p.createMultiBody(baseCollisionShapeIndex=standingBase_c_p,
							baseVisualShapeIndex=standingBase_v_p,
								basePosition=standingBasePosition, physicsClientId=planningServer)
	standingBase_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX, 
					halfExtents=standingBase_dim/2, physicsClientId=executingServer)
	standingBase_v_e = p.createVisualShape(shapeType=p.GEOM_BOX, 
					halfExtents=standingBase_dim/2, physicsClientId=executingServer)
	standingBaseM_e = p.createMultiBody(baseCollisionShapeIndex=standingBase_c_e,
							baseVisualShapeIndex=standingBase_v_e,
								basePosition=standingBasePosition, physicsClientId=executingServer)
	static_geometries_planning.append(standingBaseM_p)
	static_geometries_executing.append(standingBaseM_e)
	print "standing base: " + str(standingBaseM_e)
	# create the static geometries - shelf
	# shelfbase
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
	static_geometries_planning.append(shelfbaseM_p)
	static_geometries_executing.append(shelfbaseM_e)
	print "shelf base: " + str(shelfbaseM_e)
	# the shape and visual of the flank
	flank_dim = np.array([shelfbase_dim[0], 0.06, 1.2])
	flank_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX, 
								halfExtents=flank_dim/2, physicsClientId=planningServer)
	flank_v_p = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=flank_dim/2, 
							rgbaColor=[0.63, 0.32, 0.18, 1], physicsClientId=planningServer)
	flank_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX, 
								halfExtents=flank_dim/2, physicsClientId=executingServer)
	flank_v_e = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=flank_dim/2, 
							rgbaColor=[0.63, 0.32, 0.18, 1], physicsClientId=executingServer)
	#leftflank
	leftflankPosition = [0.0, shelfbase_dim[1]/2-flank_dim[1]/2, shelfbase_dim[2]+flank_dim[2]/2]
	leftflankM_p = p.createMultiBody(baseCollisionShapeIndex=flank_c_p, 
								baseVisualShapeIndex=flank_v_p, 
									basePosition=leftflankPosition, physicsClientId=planningServer)
	leftflankM_e = p.createMultiBody(baseCollisionShapeIndex=flank_c_e, 
								baseVisualShapeIndex=flank_v_e, 
									basePosition=leftflankPosition, physicsClientId=executingServer)
	static_geometries_planning.append(leftflankM_p)
	static_geometries_executing.append(leftflankM_e)
	print "left flank: " + str(leftflankM_e)
	#rightflank
	rightflankPosition = [0.0, -shelfbase_dim[1]/2+flank_dim[1]/2, shelfbase_dim[2]+flank_dim[2]/2]
	rightflankM_p = p.createMultiBody(baseCollisionShapeIndex=flank_c_p,
								baseVisualShapeIndex=flank_v_p, 
									basePosition=rightflankPosition, physicsClientId=planningServer)
	rightflankM_e = p.createMultiBody(baseCollisionShapeIndex=flank_c_e,
								baseVisualShapeIndex=flank_v_e, 
									basePosition=rightflankPosition, physicsClientId=executingServer)
	static_geometries_planning.append(rightflankM_p)
	static_geometries_executing.append(rightflankM_e)
	print "right flank: " + str(rightflankM_e)
	#middleflank
	middleflankPosition = [0.0, 0.0, shelfbase_dim[2]+flank_dim[2]/2]
	middleflankM_p = p.createMultiBody(baseCollisionShapeIndex=flank_c_p,
							baseVisualShapeIndex=flank_v_p, 
								basePosition=middleflankPosition, physicsClientId=planningServer)
	middleflankM_e = p.createMultiBody(baseCollisionShapeIndex=flank_c_e,
							baseVisualShapeIndex=flank_v_e, 
								basePosition=middleflankPosition, physicsClientId=executingServer)
	static_geometries_planning.append(middleflankM_p)
	static_geometries_executing.append(middleflankM_e)
	print "middle flank: " + str(middleflankM_e)
	#the shape and visual of the flat
	flat_dim = np.array([shelfbase_dim[0], shelfbase_dim[1], 0.06])
	flat_c_p = p.createCollisionShape(shapeType=p.GEOM_BOX, 
									halfExtents=flat_dim/2, physicsClientId=planningServer)
	flat_v_p = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=flat_dim/2, 
							rgbaColor=[0.41, 0.41, 0.41, 1], physicsClientId=planningServer)
	flat_c_e = p.createCollisionShape(shapeType=p.GEOM_BOX, 
									halfExtents=flat_dim/2, physicsClientId=executingServer)
	flat_v_e = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=flat_dim/2, 
							rgbaColor=[0.41, 0.41, 0.41, 1], physicsClientId=executingServer)
	#middleflat
	middleflatPosition = [0.0, 0.0, shelfbase_dim[2]+flank_dim[2]/2]
	middleflatM_p = p.createMultiBody(baseCollisionShapeIndex=flat_c_p,
						baseVisualShapeIndex=flat_v_p, 
							basePosition=middleflatPosition, physicsClientId=planningServer)
	middleflatM_e = p.createMultiBody(baseCollisionShapeIndex=flat_c_e,
						baseVisualShapeIndex=flat_v_e, 
							basePosition=middleflatPosition, physicsClientId=executingServer)
	static_geometries_planning.append(middleflatM_p)
	static_geometries_executing.append(middleflatM_e)
	print "middle flat: " + str(middleflatM_e)
	#topflat
	topflatPosition = [0.0, 0.0, shelfbase_dim[2]+flank_dim[2]+flat_dim[2]/2]
	topflatM_p = p.createMultiBody(baseCollisionShapeIndex=flat_c_p,
						baseVisualShapeIndex=flat_v_p, 
							basePosition=topflatPosition, physicsClientId=planningServer)
	topflatM_e = p.createMultiBody(baseCollisionShapeIndex=flat_c_e,
						baseVisualShapeIndex=flat_v_e, 
							basePosition=topflatPosition, physicsClientId=executingServer)
	static_geometries_planning.append(topflatM_p)
	static_geometries_executing.append(topflatM_e)
	print "top flat: " + str(topflatM_e)
	#back
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
	static_geometries_planning.append(shelfbackM_p)
	static_geometries_executing.append(shelfbackM_e)
	print "shelf back: " + str(shelfbackM_e)
	#reset the base of Kuka
	kukaBasePosition = [standingBasePosition[0], standingBasePosition[1], standingBase_dim[2]+0.005]
	# kukaBaseOrientation = p.getQuaternionFromEuler([-math.pi/2, math.pi, 0.0])
	# kukaBaseOrientation = p.getQuaternionFromEuler([math.pi/2, math.pi, 0.0])
	kukaBaseOrientation = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
	p.resetBasePositionAndOrientation(kukaID_p, kukaBasePosition, 
									kukaBaseOrientation, physicsClientId=planningServer)
	p.resetBasePositionAndOrientation(kukaID_e, kukaBasePosition, 
									kukaBaseOrientation, physicsClientId=executingServer)
	# set Kuka home configuration
	home_configuration = [0,0,0,0,0,0,0]
	for i in range(1,8):
		result_p = p.resetJointState(kukaID_p, i, home_configuration[i-1], 
															physicsClientId=planningServer)
		result_e = p.resetJointState(kukaID_e, i, home_configuration[i-1], 
															physicsClientId=executingServer)
######################################################################
	if scene == "1":
		path = shelf_path + "/scenario1"
		try:
			os.makedirs(path)
		except OSError:
			print "Creation of the directory %s falied\n" % path
		else:
			pass

		# generate hypothesis (including the true pose) for each objects
		# 8 objects includes baseball as the target object
		##########################################################################
		Objects = dict()
		# meshfile, meshType, objectRole, scale, true_pos, true_angles, uncertainty, nHypo
		Objects[0] = [0, "/mesh/dasani_water_bottle/dasani_water_bottle.obj", "waterBottle", 
			"target", 1.7, [0.04, 0.33, 0.17+middleflatPosition[2]+flat_dim[2]/2], 
												[math.pi/2, 0.0, 0.0], [0.05, 0.04, 0.3]]
		Objects[1] = [1, "/mesh/soft_white_lightbulb/soft_white_lightbulb.obj", 
			"lightbulb", "normal", 1.5, [-0.12, 0.15, 0.07+middleflatPosition[2]+flat_dim[2]/2], 
										[math.pi/2, 0.0, math.pi/5.7], [0.032, 0.035, 0.06]]
		Objects[2] = [2, "/mesh/elmers_washable_no_run_school_glue/elmers_washable_no_run_school_glue.obj", 
			"glue", "normal", 1.5, [-0.065, 0.44, 0.1+middleflatPosition[2]+flat_dim[2]/2], 
										[math.pi / 2, 0.0, 25*math.pi/180], [0.07, 0.02, 0.5]]
		Objects[3] = [3, "/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", 
			"tissueBox", "normal", 1.5, [-0.2, -0.31, 0.06+middleflatPosition[2]+flat_dim[2]/2], 
												[0.0, 0.0, -math.pi/1.2], [0.03, 0.03, 0.45]]
		Objects[4] = [4, "/mesh/dove_beauty_bar/dove_beauty_bar.obj", "doveBar", 
			"normal", 2, [0.1, 0.24, 0.05+shelfbase_dim[2]], 
											[math.pi/2, 0.0, math.pi/4.3], [0.03, 0.06, 0.01]]
		Objects[5] = [5, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
			"normal", 2, [-0.2, 0.36, 0.07+shelfbase_dim[2]], 
							[3*math.pi/5.5, 7*math.pi/6.2, math.pi/1.9], [0.05, 0.05, 0.57]]
		Objects[6] = [6, "/mesh/up_glucose_bottle/up_glucose_bottle.obj", 
			"glucoseBottle", "normal", 1.5, [-0.23, -0.43, 0.1+shelfbase_dim[2]], 
											[math.pi / 2, 0.0, math.pi], [0.04, 0.04, 0.21]]
		Objects[7] = [7, "/mesh/dr_browns_bottle_brush/dr_browns_bottle_brush.obj", 
			"brush", "normal", 1, [0.07, -0.32, 0.02+shelfbase_dim[2]], 
											[0.0, 0.0, -math.pi/3.1], [0.028, 0.02, 0.2]]
		# pick goal offset
		goalPos_offset = [-0.12, 0.0, 0.0]
		goalEuler = [math.pi/2, 0.0, 0.0] ## side picking
		x_ll = kukaBasePosition[0] - standingBase_dim[0] / 2 - 0.2
		x_ul = shelfbackPosition[0] - shelfback_dim[0] / 2
		y_ll = middleflankPosition[1] + flank_dim[1] / 2
		y_ul = leftflankPosition[1] - flank_dim[1] / 2
		z_ll = middleflatPosition[2] + flat_dim[2] / 2
		z_ul = topflatPosition[2] - flat_dim[2] / 2

	elif scene == "2":
		path = shelf_path + "/scenario2"
		try:
			os.mkdir(path)
		except OSError:
			print "Creation of the directory %s falied\n" % path
		else:
			pass

		# generate hypothesis (including the true pose) for each objects
		# 11 objects includes baseball as the target object
		##########################################################################
		Objects = dict()
		# objectIndex, meshfile, meshType, objectRole, scale, true_pos, true_angles, uncertainty, nHypo
		Objects[0] = [0, "/mesh/dasani_water_bottle/dasani_water_bottle.obj", "waterBottle", 
			"target", 2.0, [0.15, 0.45, 0.2+middleflatPosition[2]+flat_dim[2]/2], 
									[math.pi/2, 0.0, -math.pi/5.2], [0.05, 0.04, 0.32]]
		Objects[1] = [1, "/mesh/dove_beauty_bar/dove_beauty_bar.obj", "doveBar", 
			"normal", 2, [-0.2, 0.43, 0.05+middleflatPosition[2]+flat_dim[2]/2], 
									[math.pi/2, 0.0, math.pi/2.9], [0.03, 0.03, 0.01]]
		Objects[2] = [2, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
			"normal", 2, [0.15, -0.39, 0.07+shelfbase_dim[2]], 
								[3*math.pi/4.0, 2.5*math.pi, math.pi/6.2], [0.04, 0.02, 1.2]]
		Objects[3] = [3, "/mesh/ticonderoga_12_pencils/ticonderoga_12_pencils.obj", "pencils", 
			"normal", 2, [0.05, -0.26, 0.01+middleflatPosition[2]+flat_dim[2]/2], 
									[0.0, -math.pi/2, -math.pi/3.9], [0.05, 0.04, 0.25]]
		Objects[4] = [4, "/mesh/elmers_washable_no_run_school_glue/elmers_washable_no_run_school_glue.obj", 
			"glue", "normal", 1.5, [-0.19, -0.48, 0.11+middleflatPosition[2]+flat_dim[2]/2], 
									[math.pi / 2, 0.0, 25*math.pi/180], [0.07, 0.07, 0.5]]
		Objects[5] = [5, "/mesh/folgers_classic_roast_coffee/folgers_classic_roast_coffee.obj", 
			"coffeeJar", "normal", 2, [-0.2, -0.24, 0.12+shelfbase_dim[2]], 
									[math.pi/2, 0.0, -math.pi/3.9], [0.024, 0.017, 0.24]]
		Objects[6] = [6, "/mesh/up_glucose_bottle/up_glucose_bottle.obj", 
			"glucoseBottle", "normal", 1.5, [-0.08, 0.3, 0.1+middleflatPosition[2]+flat_dim[2]/2], 
										[math.pi / 2, 0.0, math.pi], [0.04, 0.04, 0.12]]
		# Objects[7] = [7, "/mesh/crayola_24_ct/crayola_24_ct.obj", 
		# 	"crayola", "normal", 2, [-0.3, 0.2, 0.07+middleflatPosition[2]+flat_dim[2]/2], 
		# 									[0.0, 0.0, -math.pi/6.1], [0.04, 0.03, 0.18]]
		Objects[7] = [7, "/mesh/crayola_24_ct/crayola_24_ct.obj", 
			"crayola", "normal", 2, [-0.3, 0.24, 0.1+middleflatPosition[2]+flat_dim[2]/2], 
											[math.pi/2, 0.0, -math.pi/6.1], [0.04, 0.03, 0.18]]
		Objects[8] = [8, "/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", 
			"tissueBox", "normal", 1.5, [-0.2, 0.34, 0.085+shelfbase_dim[2]], 
											[0.0, 0.0, -math.pi/1.2], [0.03, 0.03, 0.54]]
		Objects[9] = [9, "/mesh/soft_white_lightbulb/soft_white_lightbulb.obj", 
			"lightbulb", "phantom", 1.5, [0.14, 0.18, 0.05+middleflatPosition[2]+flat_dim[2]/2], 
											[0.0, 0.0, math.pi/6.0], [0.032, 0.035, 0.06]]
		Objects[10] = [10, "/mesh/dr_browns_bottle_brush/dr_browns_bottle_brush.obj", 
			"brush", "invisible", 1, [0.09, -0.21, 0.03+shelfbase_dim[2]], 
										[0.0, 0.0, -math.pi / 3.1], [0.028, 0.02, 0.2]]
		# pick goal offset
		goalPos_offset = [-0.12, 0.0, 0.04]
		goalEuler = [math.pi/2, 0.0, 0.0] ## side picking
		x_ll = kukaBasePosition[0] - standingBase_dim[0] / 2 - 0.2
		x_ul = shelfbackPosition[0] - shelfback_dim[0] / 2
		y_ll = middleflankPosition[1] + flank_dim[1] / 2
		y_ul = leftflankPosition[1] - flank_dim[1] / 2
		z_ll = middleflatPosition[2] + flat_dim[2] / 2
		z_ul = topflatPosition[2] - flat_dim[2] / 2

## At here we finish selecting the specific scene for specific benchmark type
## generate ground truth
truePoses, nObjectInExecuting = utils1.trueScene_generation(bt, scene, Objects, executingServer)
## Now generate planning scene based on nhypo and noiseLevel
meshSet, nObjectInPlanning = utils1.planScene_generation(Objects, bt, static_geometries_planning,
															path, nHypo, noiseLevel, planningServer)

######################################roadmap generation###############################################
startTime = time.clock()
###### specify q_start and set of q_goal first ######
## q_start ##
q_start = home_configuration

## set home configuration
for i in range(1,8):
	result = p.resetJointState(kukaID_p, i, home_configuration[i-1], physicsClientId=planningServer)
# raw_input("Press Enter to Continue")
goalSet = []
goalHypo = []
# goalSurvivalThreshold = 0.5
MaxGoalsPerPose = 5
MaxTrialsPerPose = 7
MaxTrialsPerEE = 7

## for each target hypothesis ##
for hp in xrange(nHypo):
	temp_goalsForThatHypo = []
	temp_survivalForThatHypo = []
	# print "***********For Hypo " + str(hp) + "***************"
	temp_trials_pose = 0
	# temp_ngoals = 0
	# while temp_ngoals < MaxGoalsPerPose and temp_trials_pose < MaxTrialsPerPose:
	while temp_trials_pose < MaxTrialsPerPose:
		# print "-----A new ee pose-----"
		for i in range(1,8):
			result = p.resetJointState(kukaID_p, i, home_configuration[i-1], 
																	physicsClientId=planningServer)
		## specify the position and quaternion of the goal pose based on the target hypothesis ##
		goal_pose_pos = []
		for i in xrange(len(goalPos_offset)):
			goal_pose_pos.append(meshSet[hp].pos[i] + goalPos_offset[i])
		temp_goal_pose_quat = p.getQuaternionFromEuler([goalEuler[0], goalEuler[1], 
													goalEuler[2]+random.uniform(-math.pi, math.pi)])
		goal_pose_quat = [temp_goal_pose_quat[0], temp_goal_pose_quat[1], 
													temp_goal_pose_quat[3], temp_goal_pose_quat[2]]
		temp_trials_ee = 0
		# while temp_ngoals < MaxGoalsPerPose and temp_trials_ee < MaxTrialsPerEE:
		while temp_trials_ee < MaxTrialsPerEE:
			q_goal = p.calculateInverseKinematics(kukaID_p, kuka_ee_idx, goal_pose_pos, 
										goal_pose_quat, ll, ul, jr, physicsClientId=planningServer)
			for j in range(1,8):
				result = p.resetJointState(kukaID_p, j, q_goal[j-1], physicsClientId=planningServer)
			p.stepSimulation(planningServer)
			## check collision for static geometry
			isCollision1 = utils1.collisionCheck_staticG(kukaID_p, 
													static_geometries_planning, planningServer)
			if isCollision1:
				# print "static collision for the current IK..."
				# raw_input("Press Enter to Continue")
				temp_trials_ee += 2
				continue
			else:
				## check collision condition with all hypos of objects
				## (The opposite of the collision probability)
				collidedHypos = utils1.collision_with_hypos(kukaID_p, meshSet, planningServer)
				# print "Collide with Hypos: " + str(collidedHypos)
				## compute the survivability
				if hp in collidedHypos:
					## the end effector collides with the pose it deems as the target pose
					temp_survival = 0.0
					temp_trials_ee += 1
					# print "Survival: " + str(temp_survival)
					# raw_input("Press Enter to Continue")
					continue;
				else:
					temp_survival = 1.0
					collisionPerObj = [0.0] * nObjectInPlanning
					for ch in collidedHypos:
						if ch not in range(0, nHypo):
							collisionPerObj[meshSet[ch].objIndx] += meshSet[ch].prob
					for cpobs in collisionPerObj:
						temp_survival *= (1 - cpobs)
					# print "Survival: " + str(temp_survival)
					# raw_input("Press Enter to Continue")
					## add the q_goal and its corresponding survivability
					temp_goalsForThatHypo.append(q_goal)
					temp_survivalForThatHypo.append(temp_survival)
					temp_trials_ee += 1

				# if temp_survival >= goalSurvivalThreshold:
				# 	## accept this goal
				# 	goalSet.append(q_goal)
				# 	goalHypo.append(hp)
				# 	temp_ngoals += 1
				
		temp_trials_pose += 1

	## sort temp_survivalForThatHypo and pick top (MaxGoalsPerPose) ones
	idx_rank = sorted( range(len(temp_survivalForThatHypo)), key=lambda k: temp_survivalForThatHypo[k], reverse=True )
	if len(idx_rank) >= MaxGoalsPerPose:
		## add top (MaxGoalsPerPose) ones
		for mm in xrange(MaxGoalsPerPose):
			goalSet.append(temp_goalsForThatHypo[idx_rank[mm]])
			goalHypo.append(hp)
	else:
		for idx in idx_rank:
			goalSet.append(temp_goalsForThatHypo[idx])
			goalHypo.append(hp)

print goalHypo
##########################################################################################################


##############start sampling##################
currentSamplesFile = path + "/samples.txt"
f_samples = open(currentSamplesFile, "w")
nodes = []
temp_counter = 0

while temp_counter < nsamples:
	# sample a cartesian ee pose and calculate the IK solution
	temp_x = float(format(random.uniform(x_ll, x_ul), '.2f'))
	temp_y = float(format(random.uniform(y_ll, y_ul), '.2f'))
	temp_z = float(format(random.uniform(z_ll, z_ul), '.2f'))
	ikSolution = p.calculateInverseKinematics(kukaID_p, kuka_ee_idx, 
					[temp_x, temp_y, temp_z], ll, ul, jr, physicsClientId=planningServer)
	for j in range(1,8):
		result = p.resetJointState(kukaID_p, j, ikSolution[j-1], physicsClientId=planningServer)
	p.stepSimulation(planningServer)
	isCollision = utils1.collisionCheck_staticG(kukaID_p, static_geometries_planning, planningServer)
	if isCollision == False:
		nodes.append(ikSolution)
		## write it into a roadmap file
		f_samples.write(str(temp_counter) + " " + str(ikSolution[0]) + " " + str(ikSolution[1]) + " " \
			+ str(ikSolution[2]) + " " + str(ikSolution[3]) + " " + str(ikSolution[4]) + " " \
			+ str(ikSolution[5]) + " " + str(ikSolution[6]) + "\n")
		temp_counter += 1
######################################################################################

# startTime = time.clock()
############ connect neighbors to build roadmaps #############
connectivity = np.zeros((nsamples, nsamples))
tree = spatial.KDTree(nodes)
neighbors_const = 1.5 * math.e * (1 + 1/len(home_configuration))
num_neighbors = int(neighbors_const * math.log(nsamples))
if num_neighbors >= nsamples:
	num_neighbors = nsamples -1 
print "num_neighbors: " + str(num_neighbors)

currentRoadmapFile = path + "/roadmap.txt"
f_roadmap = open(currentRoadmapFile, "w")
# for each node
for i in xrange(len(nodes)):
	queryNode = nodes[i]
	knn = tree.query(queryNode, k=num_neighbors, p=2)
	# for each neighbor
	for j in xrange(len(knn[1])):
		if knn[1][j] == i or connectivity[i][knn[1][j]] == 1:
			# if the neighbor is the query node itself
			# or the connectivity has been checked before
			# then skip the edge checking procedure
			continue
		# Otherwise, check the edge validity (in terms of collision with static geometry)
		# between the query node and the the current neighbor
		neighbor = nodes[knn[1][j]]
		isEdgeValid = utils1.checkEdgeValidity(queryNode, neighbor, kukaID_p, 
														static_geometries_planning, planningServer)
		if isEdgeValid:
			# write this edge information with their cost and labels into the txt file
			# It is a valid edge in terms of static geometry
			# Let's check the collision status for each hypothesis for the purpose of labeling
			temp_labels = utils1.label_the_edge(queryNode, neighbor, kukaID_p, meshSet, planningServer)
			f_roadmap.write(str(i) + " " + str(knn[1][j]) + " " + format(knn[0][j], '.4f') + " ")
			for tl in temp_labels:
				f_roadmap.write(str(tl) + " ")
			f_roadmap.write("\n")
			# update connectivity information
			connectivity[i][knn[1][j]] = 1
			connectivity[knn[1][j]][i] = 1

	if i % 100 == 99:
		print "finish labeling and connecting neighbors for node " + str(i)

print "finish all the neighbors"
# print "Time elapsed: ", time.clock() - startTime
######################################################################

###### add the start node and goal nodes to the roadmap######
nodes.append(q_start)
f_samples.write(str(temp_counter) + " " + str(q_start[0]) + " " + str(q_start[1]) + " " \
	+ str(q_start[2]) + " " + str(q_start[3]) + " " + str(q_start[4]) + " " + \
	str(q_start[5]) + " " + str(q_start[6]) + "\n")
temp_counter += 1
## connect the start to the roadmap
tree = spatial.KDTree(nodes)
queryNode = nodes[temp_counter-1]
knn = tree.query(queryNode, k=num_neighbors, p=2)

highestSurvival_neighborIdx = -1;
highestSurvivalSofar = -1.0
highestSurvival_labels = []
highestSurvival_cost = -1.0

# for each neighbor
for j in xrange(len(knn[1])):
	if knn[1][j] == (temp_counter-1):
		continue
	else:
		# check collision
		neighbor = nodes[knn[1][j]]
		isEdgeValid = utils1.checkEdgeValidity(queryNode, neighbor, kukaID_p, 
													static_geometries_planning, planningServer)
		if isEdgeValid:
			## examine the survivability of the edge
			## first get the labels
			temp_labels = utils1.label_the_edge(queryNode, neighbor, kukaID_p, meshSet, planningServer)
			## compute the survivability
			temp_survival = 1.0
			collisionPerObj = [0.0] * nObjectInPlanning
			for tl in temp_labels:
				collisionPerObj[meshSet[tl].objIndx] += meshSet[tl].prob
			for cpobs in collisionPerObj:
				temp_survival *= (1 - cpobs)
			# print "Survival: " + str(temp_survival)
			# raw_input("Press Enter to Continue")
			if temp_survival > highestSurvivalSofar:
				highestSurvivalSofar = temp_survival
				highestSurvival_neighborIdx = knn[1][j]
				highestSurvival_labels = temp_labels
				highestSurvival_cost = knn[0][j]

## finish all the survival computation for all k nearest neighbors
## Now connect the start to the one neighbor with the largest survival (lower cost for tie breaker)
if (highestSurvival_neighborIdx != -1):
	f_roadmap.write(str(temp_counter-1) + " " + str(highestSurvival_neighborIdx) + " " \
										+ format(highestSurvival_cost, '.4f') + " ")
	for hl in highestSurvival_labels:
		f_roadmap.write(str(hl) + " ")
	f_roadmap.write("\n")
	print "successfully connect the start to the roadmap\n"

## Loop through goalSet
for q_goal, hypo in zip(goalSet, goalHypo):
	# print "For goal: " + str(temp_counter)
	nodes.append(q_goal)
	f_samples.write(str(temp_counter) + " " + str(q_goal[0]) + " " + str(q_goal[1]) + " " \
		+ str(q_goal[2]) + " " + str(q_goal[3]) + " " + str(q_goal[4]) + " " + \
		str(q_goal[5]) + " " + str(q_goal[6]) + " " + str(hypo) + "\n")
	temp_counter += 1
	## connect the goal to the roadmap
	tree = spatial.KDTree(nodes)
	queryNode = nodes[temp_counter-1]
	knn = tree.query(queryNode, k=num_neighbors, p=2)
	
	# connectTimes = 0
	# highestSurvival_neighborIdx = -1;
	# highestSurvivalSofar = -1.0
	# highestSurvival_labels = []
	# highestSurvival_cost = -1.0

	# for each neighbor
	for j in xrange(len(knn[1])):
		if knn[1][j] == (temp_counter - 1):
			continue
		else:
			# check collision
			neighbor = nodes[knn[1][j]]
			isEdgeValid = utils1.checkEdgeValidity(queryNode, neighbor, kukaID_p, 
														static_geometries_planning, planningServer)
			if isEdgeValid:
				## examine the survivability of the edge
				## first get the labels
				temp_labels = utils1.label_the_edge(queryNode, neighbor, kukaID_p, meshSet, planningServer)
				# ## compute the survivability
				# temp_survival = 1.0
				# collisionPerObj = [0.0] * nObjectInPlanning
				# for tl in temp_labels:
				# 	collisionPerObj[meshSet[tl].objIndx] += meshSet[tl].prob
				# for cpobs in collisionPerObj:
				# 	temp_survival *= (1 - cpobs)
				# # print "Survival: " + str(temp_survival)
				# # raw_input("Press Enter to Continue")
				# if temp_survival > highestSurvivalSofar:
				# 	highestSurvivalSofar = temp_survival
				# 	highestSurvival_neighborIdx = knn[1][j]
				# 	highestSurvival_labels = temp_labels
				# 	highestSurvival_cost = knn[0][j]
				# 	# print "which neighbor: " + str(j)
				# 	# print "highestSurvival_labels: "
				# 	# for l in highestSurvival_labels:
				# 	# 	print str(l) + " "
				# 	# print "highestSurvivalSofar: " + str(highestSurvivalSofar)
				# 	# print "highestSurvival_neighborIdx: " + str(highestSurvival_neighborIdx)
				# 	# print "highestSurvival_cost: " + str(highestSurvival_cost)
					# print "         ";
				f_roadmap.write( str(temp_counter-1) + " " + str(knn[1][j]) + " " \
					+ format(knn[0][j], '.4f') + " " )
				for temp_l in temp_labels:
					f_roadmap.write(str(temp_l) + " ")
				f_roadmap.write("\n")
				# print "successfully connect the goal " + str(temp_counter-1) + " to the roadmap\n"

	## finish all the survival computation for all k nearest neighbors for a single specific goal
	## Now connect that specific goal to the one neighbor with the largest survival (lower cost for tie breaker)
	# if (highestSurvival_neighborIdx != -1):
	# 	f1.write(str(temp_counter-1) + " " + str(highestSurvival_neighborIdx) + " " \
	# 										+ format(highestSurvival_cost, '.4f') + " ")
	# 	for hl in highestSurvival_labels:
	# 		f1.write(str(hl) + " ")
	# 	f1.write("\n")
	# 	print "successfully connect the goal " + str(temp_counter-1) + " to the roadmap\n"

f_samples.close()
f_roadmap.close()
print "roadmap generated with " + str(len(nodes)) + " nodes in " + str(time.clock() - startTime) + " second."

print len(nodes)


##########################################################################################
# Call planning algorithm
print "start planning..."
executeFile = "../robust_planning_20_icra/main1" + " " + str(sys.argv[1]) \
											+ " " + str(sys.argv[2]) + " " + str(sys.argv[5])
subprocess.call(executeFile, shell=True)

raw_input("Press Enter to Continue")
## Put the kuka back to its home configuration
for i in range(1,8):
	result = p.resetJointState(kukaID_e, i, home_configuration[i-1], physicsClientId=executingServer)
	
# raw_input("Press Enter to Continue")
# Let's execute the path
## execute the trajectory in the scene without the objects
startExeTime = time.clock() 
print "Astar trajectory"
astar_traj_file = path + "/Astartraj.txt"
utils1.executeTrajectory(astar_traj_file, kukaID_e, executingServer)
print "total time: " + str(time.clock() - startTime)

# raw_input("Press Enter to Continue")
## Put the kuka back to its home configuration
for i in range(1,8):
	result = p.resetJointState(kukaID_e, i, home_configuration[i-1], physicsClientId=executingServer)

# raw_input("Press Enter to Continue")
startExeTime = time.clock() 
print "MCR greedy trajectory"
mcrg_traj_file = path + "/MCRGtraj.txt"
utils1.executeTrajectory(mcrg_traj_file, kukaID_e, executingServer)
print "total time: " + str(time.clock() - startTime)

# raw_input("Press Enter to Continue")
## Put the kuka back to its home configuration
for i in range(1,8):
	result = p.resetJointState(kukaID_e, i, home_configuration[i-1], physicsClientId=executingServer)

# raw_input("Press Enter to Continue")
startExeTime = time.clock() 
print "MCR exact trajectory"
mcre_traj_file = path + "/MCREtraj.txt"
utils1.executeTrajectory(mcre_traj_file, kukaID_e, executingServer)
print "total time: " + str(time.clock() - startTime)

# raw_input("Press Enter to Continue")
## Put the kuka back to its home configuration
for i in range(1,8):
	result = p.resetJointState(kukaID_e, i, home_configuration[i-1], physicsClientId=executingServer)

# raw_input("Press Enter to Continue")
startExeTime = time.clock() 
print "MaxSuccess greedy trajectory"
msg_traj_file = path + "/MSGtraj.txt"
utils1.executeTrajectory(msg_traj_file, kukaID_e, executingServer)
print "total time: " + str(time.clock() - startTime)

# raw_input("Press Enter to Continue")
## Put the kuka back to its home configuration
for i in range(1,8):
	result = p.resetJointState(kukaID_e, i, home_configuration[i-1], physicsClientId=executingServer)

# raw_input("Press Enter to Continue")
startExeTime = time.clock() 
print "MaxSuccess exact trajectory"
mse_traj_file = path + "/MSEtraj.txt"
utils1.executeTrajectory(mse_traj_file, kukaID_e, executingServer)
print "total time: " + str(time.clock() - startTime)


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
	isCollision1 = utils1.collisionCheck_staticG(kukaID, static_geometries)
	if isCollision1:
		pass
	else:
		isCollision2 = utils1.collisionCheck_hypos(kukaID, mesheSet)
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


'''
###### specify q_start and q_goal first ######
## q_start ##
q_start = home_configuration
## goal ##
## set home configuration
for i in range(1,8):
	result = p.resetJointState(kukaID_p, i, home_configuration[i-1], physicsClientId=planningServer)
goal_pose_pos = []

for i in xrange(len(goalPos_offset)):
	# The goal object always has the index 0 (zero)
	goal_pose_pos.append(Objects[0][5][i] + goalPos_offset[i])
goal_pose_quat = utils1.euler_to_quaternion(goalEuler[0], goalEuler[1], goalEuler[2])

goalCollision = True
while goalCollision:
	q_goal = p.calculateInverseKinematics(kukaID_p, kuka_ee_idx, goal_pose_pos, 
										goal_pose_quat, ll, ul, jr, physicsClientId=planningServer)
	for j in range(1,8):
		result = p.resetJointState(kukaID_p, j, q_goal[j-1], physicsClientId=planningServer)
	p.stepSimulation(planningServer)
	# check collision for both static geometry & objects
	isCollision1 = utils1.collisionCheck_staticG(kukaID_p, static_geometries_planning, planningServer)
	if isCollision1:
		pass
	else:
		isCollision2 = utils1.collisionCheck_hypos(kukaID_p, meshSet, planningServer)
		if not isCollision2:
			goalCollision = False
	print "collision for the goal? " + " " + str(goalCollision)
	time.sleep(0.2)
	# if goalCollision:
	# 	## put the kuka arm back to home configuration for next IK solution
	# 	for i in range(1,8):
	# 		result = p.resetJointState(kukaID,i,home_configuration[i-1])
##############################################################################
'''


'''
##################################################################################
## generate IK for ground truth
for i in range(1,8):
	result = p.resetJointState(kukaID_p, i, home_configuration[i-1], 
															physicsClientId=planningServer)
goalSurvivalThreshold = 0.5
MaxGoalsPerPose = 10
MaxTrialsPerPose = 7
MaxTrialsPerEE = 7

## for each target hypothesis ##
for hp in xrange(Objects[0][8]):
	print "***********For Hypo " + str(hp) + "***************"
	temp_trials_pose = 0
	temp_ngoals = 0
	while temp_ngoals < MaxGoalsPerPose and temp_trials_pose < MaxTrialsPerPose:
		print "-----A new ee pose-----"
		# for i in range(1,8):
		# 	result = p.resetJointState(kukaID_p, i, home_configuration[i-1], 
		# 															physicsClientId=planningServer)
		## specify the position and quaternion of the goal pose based on the target hypothesis ##
		goal_pose_pos = []
		for i in xrange(len(goalPos_offset)):
			goal_pose_pos.append(meshSet[hp].pos[i] + goalPos_offset[i])
		temp_goal_pose_quat = p.getQuaternionFromEuler([goalEuler[0], goalEuler[1], 
													goalEuler[2]+random.uniform(-math.pi, math.pi)])
		goal_pose_quat = [temp_goal_pose_quat[0], temp_goal_pose_quat[1], 
													temp_goal_pose_quat[3], temp_goal_pose_quat[2]]
		temp_trials_ee = 0
		while temp_ngoals < MaxGoalsPerPose and temp_trials_ee < MaxTrialsPerEE:
			# for i in range(1,8):
			# 	result = p.resetJointState(kukaID_p, i, home_configuration[i-1], 
			# 															physicsClientId=planningServer)
			q_goal = p.calculateInverseKinematics(kukaID_p, kuka_ee_idx, goal_pose_pos, 
										goal_pose_quat, ll, ul, jr, physicsClientId=planningServer)
			for j in range(1,8):
				result = p.resetJointState(kukaID_p, j, q_goal[j-1], physicsClientId=planningServer)
			p.stepSimulation(planningServer)
			## check collision for static geometry
			isCollision1 = utils1.collisionCheck_staticG(kukaID_p, 
													static_geometries_planning, planningServer)
			if isCollision1:
				print "static collision for the current IK..."
				raw_input("Press Enter to Continue")
				temp_trials_ee += 2
				continue
			else:
				## check collision condition with all hypos of objects
				## (The opposite of the collision probability)
				collidedHypos = utils1.collision_with_hypos(kukaID_p, meshSet, planningServer)
				print "Collide with Hypos: " + str(collidedHypos)
				## compute the survivability
				if hp in collidedHypos:
					# the end effector collides with the pose it deems as the target pose
					temp_survival = 0.0
				else:
					temp_survival = 1.0
					collisionPerObj = [0.0] * nObjectInPlanning
					for ch in collidedHypos:
						if ch not in range(0, Objects[0][8]):
							collisionPerObj[meshSet[ch].objIndx] += meshSet[ch].prob
					for cpobs in collisionPerObj:
						temp_survival *= (1 - cpobs)
				print "Survival: " + str(temp_survival)
				raw_input("Press Enter to Continue")

				if temp_survival >= goalSurvivalThreshold:
					## accept this goal
					# goalSet.append(q_goal)
					# goalHypo.append(hp)
					temp_ngoals += 1
				
				temp_trials_ee += 1
		
		temp_trials_pose += 1
###########################################################
'''