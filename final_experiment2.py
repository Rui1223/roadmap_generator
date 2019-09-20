from __future__ import division
import pybullet as p
import pybullet_data
import utils2

import math
import random
import time
import numpy as np

import sys
import os
import subprocess

from scipy import spatial
import cPickle as pickle

planningServer = p.connect(p.DIRECT)
executingServer = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
kuka_ee_idx = 8

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

# p.setTimeStep(5000, planningServer)

benchmarkType = ["shelf"]
scenarios = ["1", "2"]
nroadmapPerParam = 5 ## can increase later if have more time

# nHypos = [1, 2, 3, 4, 5, 6, 7]
# noiseLevel = [1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0]
nHypos = [4]
# noiseLevel = [2.5]


nHypo_standard = 4
noiseLevel_stardard = 2.3


## loop through each benchmark type
for bt in benchmarkType:
	static_geometries_planning = []
	static_geometries_executing = []
	# Introduce Kuka arm
	kukaID_p = p.loadURDF("kuka.urdf", useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION or p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT, physicsClientId=planningServer)
	kukaID_e = p.loadURDF("kuka.urdf", useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION or p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT, physicsClientId=executingServer)
	static_geometries_planning.append(kukaID_p)
	static_geometries_executing.append(kukaID_e)
	print "Kuka Robot: " + str(kukaID_e)
	# generate the static geometries
	################################
	if bt == "table":
		benchmarkType_path = "benchmark/new_experiments/table"
		try:
			os.mkdir(benchmarkType_path)
		except OSError:
			print "Creation of the directory %s falied\n" % benchmarkType_path
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
	else:
		benchmarkType_path = "benchmark/new_experiments/shelf"
		try:
			os.mkdir(benchmarkType_path)
		except OSError:
			print "Creation of the directory %s falied\n" % benchmarkType_path
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
	################################
	## loop through each scene
	for scene in scenarios:
		scene_path = benchmarkType_path + "/scenario" + str(scene)
		try:
			os.mkdir(scene_path)
		except OSError:
			print "Creation of the directory %s falied\n" % scene_path
		else:
			pass
		if bt == "table" and scene == "1":
			print "Welcome to " + bt + " sceneario " + scene
			## define 7 objects: baseball as the target ##
			##############################################
			Objects = dict()
			# meshfile, meshType, objectRole, scale, true_pos, true_angles, uncertainty, nHypo
			Objects[0] = [0, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
				"target", 2, [0.165, 0.0, 0.07+table_dim[2]], [math.pi/5.8, math.pi/0.4, math.pi/2.3], 
												[0.012, 0.013, 0.45]]
			# Objects[0] = [0, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
			# 	"target", 2, [0.20, 0.0, 0.07+table_dim[2]], [math.pi/5.8, math.pi/0.4, math.pi/2.3], 
			# 									[0.012, 0.013, 0.45]]
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
			# goalPos_offset = [0.08, 0.0, 0.17]
			goalEuler = [0.0, math.pi, 0.0] ## tabletop picking
			x_ll = kukaBasePosition[0] - 0.2
			x_ul = tablePosition[0] + table_dim[0]/2
			y_ll = -table_dim[1]/2
			y_ul = table_dim[1]/2
			z_ll = table_dim[2]
			z_ul = table_dim[2]+0.7

		elif bt == "table" and scene == "2":
			print "Welcome to " + bt + " sceneario " + scene
			## define 10 objects: baseball as the target ##
			###############################################
			Objects = dict()
			# meshfile, meshType, objectRole, scale, true_pos, true_angles, uncertainty, nHypo
			Objects[0] = [0, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
					"target", 1.9, [0.11, 0.23, 0.07+table_dim[2]], 
							[2*math.pi/7.0, math.pi/4.8, 7*math.pi/6.2], [0.05, 0.05, 1.04]]
			# Objects[0] = [0, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
			# 		"target", 2, [0.16, 0.23, 0.07+table_dim[2]], 
			# 				[2*math.pi/7.0, math.pi/4.8, 7*math.pi/6.2], [0.05, 0.05, 1.04]]
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

		elif bt == "shelf" and scene == "1":
			print "Welcome to " + bt + " sceneario " + scene
			## define 8 objects: water bottle as the target ##
			##################################################
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

		else:
			print "Welcome to " + bt + " sceneario " + scene
			## define 11 objects: water bottle as the target ##
			##################################################				
			Objects = dict()
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
		## generate the ground truth
		truePoses, nObjectInExecuting = utils2.trueScene_generation(bt, scene, Objects, executingServer)

		########## Let's loop through nHypos and noiseLevel respectively ##########

		hypo_folder = scene_path + "/nHypos"
		hypo_roadmaps_folder = hypo_folder + "/roadmaps"
		hypo_traj_folder = hypo_folder + "/trajectories"
		noiseLevel_folder = scene_path + "/noiseLevel"
		noiseLevel_roadmaps_folder = noiseLevel_folder + "/roadmaps"
		noiseLevel_traj_folder = noiseLevel_folder + "/trajectories"
		try:
			os.mkdir(hypo_folder)
			os.mkdir(hypo_roadmaps_folder)
			os.mkdir(hypo_traj_folder)
			os.mkdir(noiseLevel_folder)
			os.mkdir(noiseLevel_roadmaps_folder)
			os.mkdir(noiseLevel_traj_folder)
		except OSError:
			pass
		else:
			pass

		## four files to save the data 
		# nhypo_obs_file = hypo_folder + "/nhypos_obs.txt"
		# nhypo_success_file = hypo_folder + "/nhypos_success.txt"
		# nhypo_time_file = hypo_folder + "/nhypos_time.txt"
		# nhypo_cost_file = hypo_folder + "/nhypos_cost.txt"
		# f_nhpyo_obs = open(nhypo_obs_file, "w")
		# f_nhypo_success = open(nhypo_success_file, "w")
		# f_nhypo_time = open(nhypo_time_file, "w")
		# f_nhypo_cost = open(nhypo_cost_file, "w")

		## first deal with nHypos
		cc = 1
		for nh in nHypos:
			## generate planning scenes ## 
			## For each parameter, generate 5 planning scenes. (5 roadmaps)
			nn = 1
			while nn < nroadmapPerParam+1:
				print "----------------------------------------"
				print "nHypo: " + str(nh) + ", current trials: " + str(nn)
				meshSet, nObjectInPlanning = utils2.planScene_generation(Objects, bt, static_geometries_planning, 
												hypo_roadmaps_folder, nh, noiseLevel_stardard, cc, nn, planningServer)
				
				nsamples = int(sys.argv[1])
				utils2.roadmap_generation(home_configuration, kukaID_p, kuka_ee_idx, ll, ul, jr,
									static_geometries_planning, meshSet, 
												goalPos_offset, goalEuler, x_ll, x_ul, y_ll, y_ul, z_ll, z_ul,
													nh, nObjectInPlanning, cc, nn, 
													nsamples, hypo_roadmaps_folder, planningServer)
				## After saving the roadmaps
				## (1) it is the time to delete the planning scene (those hypos)
				## (2) it is the time the planning algorithm start to work
				utils2.deletePlanScene(meshSet, planningServer)
				print "start planning..."
				executeFile = "../robust_planning_20_icra/main2" + " " + str(bt) + " " + str(scene) + " " \
						+ str("1") + " " + str(cc) + " " + str(nn) + " " + str(nsamples) + " " + str(nh)
				subprocess.call(executeFile, shell=True)
				## check the failureIndicator file to see if this round of search is worth executing
				f = open(hypo_traj_folder + "/failureIndicator_" + str(cc) + "_" + str(nn) + ".txt")
				for line in f:
					failure_indicator = line.split()
				if failure_indicator[0] == "0":
					print "This is executable!"
					## Now we should have the corresponding trajectory in the "trajectories" folder
					## Execute these trajectories (5 trajs)
					utils2.executeAllTraj(home_configuration, kukaID_e, truePoses,
													hypo_traj_folder, cc, nn, executingServer)
					nn += 1
				else:
					print "At least one of the search fails. We will not execute this. Generate another scene."
			cc += 1
		
		# ## second deal with noiseLevel
		# cc = 1
		# for nol in noiseLevel:
		# 	## generate planning scenes ## 
		# 	## For each parameter, generate 5 planning scenes. (5 roadmaps)
		# 	nn = 1
		# 	while nn < nroadmapPerParam+1:
		# 		print "----------------------------------------"
		# 		print "noiseLevel: " + str(nol) + ", current trials: " + str(nn)
		# 		meshSet, nObjectInPlanning = utils2.planScene_generation(Objects, bt, static_geometries_planning, 
		# 									noiseLevel_roadmaps_folder, nHypo_standard, nol, cc, nn, planningServer)
		# 		nsamples = int(sys.argv[1])
		# 		utils2.roadmap_generation(home_configuration, kukaID_p, kuka_ee_idx, ll, ul, jr,
		# 							static_geometries_planning, meshSet, 
		# 										goalPos_offset, goalEuler, x_ll, x_ul, y_ll, y_ul, z_ll, z_ul,
		# 											nHypo_standard, nObjectInPlanning, cc, nn, 
		# 											nsamples, noiseLevel_roadmaps_folder, planningServer)
		# 		## After saving the roadmaps
		# 		## (1) it is the time to delete the planning scene (those hypos)
		# 		## (2) it is the time the planning algorithm start to work
		# 		utils2.deletePlanScene(meshSet, planningServer)
		# 		print "start planning..."
		# 		executeFile = "../robust_planning_20_icra/main2" + " " + str(bt) + " " + str(scene) + " " \
		# 				+ str("2") + " " + str(cc) + " " + str(nn) + " " + str(nsamples) + " " + str(nHypo_standard)
		# 		subprocess.call(executeFile, shell=True)
		# 		## check the failureIndicator file to see if this round of search is worth executing
		# 		f = open(noiseLevel_traj_folder + "/failureIndicator_" + str(cc) + "_" + str(nn) + ".txt")
		# 		for line in f:
		# 			failure_indicator = line.split()
		# 		if failure_indicator[0] == "0":
		# 			print "This is executable!"
		# 			## Now we should have the corresponding trajectory in the "trajectories" folder
		# 			## Execute these trajectories (5 trajs)
		# 			utils2.executeAllTraj(home_configuration, kukaID_e, noiseLevel_traj_folder, cc, nn, executingServer)
		# 			nn += 1
		# 		else:
		# 			print "At least one of the search fails. We will not execute this. Generate another scene."

		# 	cc += 1		


	
		## finish the current scene, before switch let's delete the ground truth
		utils2.deleteTruePoses(truePoses, executingServer)

	## finish the current benchmark type, delete static_geometries for both true and planning scene
	utils2.deleteStaticGeometries(static_geometries_planning, planningServer)
	utils2.deleteStaticGeometries(static_geometries_executing, executingServer)
	

time.sleep(10)
p.disconnect()