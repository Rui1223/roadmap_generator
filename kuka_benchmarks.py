from __future__ import division
import pybullet as p
import time
import numpy as np
import pybullet_data
import utils
import IPython
import math
import sys

from scipy import spatial
import cPickle as pickle

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
kuka_ee_idx = 8
static_geometries = []

# Introduce Kuka arm 
######################################################################
kukaID = p.loadURDF("kuka.urdf",[0,0,0],useFixedBase=True)
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

home_configuration = [0,0,0,-np.pi/2,0,np.pi/2,0]
for i in range(1,8):
	result = p.resetJointState(kukaID,i,home_configuration[i-1])
#######################################################################

# generate the static geometries
######################################################################
if sys.argv[1] == "table":
	print "---------Enter to table scene!---------"
	#reset the base of Kuka
	p.resetBasePositionAndOrientation(kukaID, [-0.7, 0, 0], [0, 0, 0, 1])
	# create the static geometries - table
	table_dim = np.array([0.7, 1.3, 0.45])
	basePosition=[0, 0, table_dim[2]/2]
	table_c = p.createCollisionShape(shapeType=p.GEOM_BOX,
							halfExtents=table_dim/2)
	table_v = p.createVisualShape(shapeType=p.GEOM_BOX,
							halfExtents=table_dim/2)
	tableM = p.createMultiBody(baseCollisionShapeIndex=table_c,
					baseVisualShapeIndex=table_v,basePosition=basePosition)
	static_geometries.append(tableM)
######################################################################	
	if sys.argv[2] == "benchmark1":
		# generate hypothesis (including the true pose) for each objects
		# 5 objects includes baseball as the target object
		##########################################################################
		Objects = dict()
		# meshfile, meshType, objectRole, scale, true_pos, true_angles, uncertainty, nHypo
		Objects[0] = [0, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
			"target", 2, [0.2, -0.21, 0.07+table_dim[2]], [0.0, 0.0, 0.0], [0.05, 0.05], 1]
		Objects[1] = [1, "/mesh/crayola_24_ct/crayola_24_ct.obj", 
		"crayola", "normal", 2, [0.15, 0.0, 0.1+table_dim[2]], 
												[math.pi/2, 0.0, 0.0], [0.04, 0.03, 0.18], 1]
		Objects[2] = [2, "/mesh/dasani_water_bottle/dasani_water_bottle.obj", "waterBottle", 
			"normal", 2, [-0.15, -0.29, 0.2+table_dim[2]], 
												[math.pi/2, 0.0, 0.0], [0.05, 0.04, 0.3], 1]
		Objects[3] = [3, "/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", 
		"tissueBox", "normal", 1.5, [0.05, -0.42, 0.1+table_dim[2]], 
										[0.0, math.pi/2, -math.pi/5], [0.03, 0.03, 0.24], 1]
		Objects[4] = [4, "/mesh/dove_beauty_bar/dove_beauty_bar.obj", "doveBar", 
			"normal", 2, [-0.12, 0.13, 0.05+table_dim[2]], 
										[math.pi/2, 0.0, math.pi/6.7], [0.03, 0.06, 0.01], 1]
		# pick goal offset												
		goalPos_offset = [-0.03, 0.0, 0.12]

	elif sys.argv[2] == "benchmark2":
		# generate hypothesis (including the true pose) for each objects
		# 7 objects includes baseball as the target object
		##########################################################################
		Objects = dict()
		# meshfile, meshType, objectRole, scale, true_pos, true_angles, uncertainty, nHypo
		Objects[0] = [0, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
			"target", 2, [0.13, -0.02, 0.07+table_dim[2]], [0.0, 0.0, 0.0], [0.05, 0.05], 1]
		Objects[1] = [1, "/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", 
					"tissueBox", "normal", 1.5, [0.05, 0.16, 0.1+table_dim[2]], 
										[0.0, math.pi/2, -math.pi/3], [0.03, 0.03, 0.24], 1]
		Objects[2] = [2, "/mesh/crayola_24_ct/crayola_24_ct.obj", 
					"crayola", "normal", 2, [0.15, -0.15, 0.1+table_dim[2]], 
										[math.pi/2, 0.0, -math.pi/4], [0.04, 0.03, 0.18], 1]
		# Objects[2] = [2, "/mesh/elmers_washable_no_run_school_glue/elmers_washable_no_run_school_glue.obj", 
		# 			"glue", "normal", 1.5, [0.05, -0.19, 0.33], [math.pi / 2, 
		#										0.0, 25*math.pi/180], [0.07, 0.07, 0.5], 1]
		Objects[3] = [3, "/mesh/dove_beauty_bar/dove_beauty_bar.obj", "doveBar", 
							"normal", 2, [-0.11, -0.19, 0.05+table_dim[2]], 
										[math.pi/2, 0.0, math.pi/1.3], [0.03, 0.06, 0.01], 1]
		Objects[4] = [4, "/mesh/dr_browns_bottle_brush/dr_browns_bottle_brush.obj", 
					"brush", "normal", 1, [0.17, -0.40, 0.02+table_dim[2]], 
											[0.0, 0.0, -math.pi / 4.5], [0.028, 0.02, 0.2], 1]
		Objects[5] = [5, "/mesh/soft_white_lightbulb/soft_white_lightbulb.obj", 
				"lightbulb", "phantom", 1.5, [-0.2, 0.36, 0.05+table_dim[2]], 
											[0.0, 0.0, math.pi/10.0], [0.032, 0.035, 0.06], 1]
		Objects[6] = [6, "/mesh/up_glucose_bottle/up_glucose_bottle.obj", 
				"glucoseBottle", "normal", 1.5, [0.2, 0.41, 0.1+table_dim[2]], 
											[math.pi / 2, 0.0, math.pi], [0.04, 0.04, 0.12], 1]
		# pick goal offset
		goalPos_offset = [-0.03, 0.0, 0.127]

	elif sys.argv[2] == "benchmark3":
		# generate hypothesis (including the true pose) for each objects
		# 11 objects includes baseball as the target object
		##########################################################################
		Objects = dict()
		# meshfile, meshType, objectRole, scale, true_pos, true_angles, uncertainty, nHypo
		Objects[0] = [0, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
				"target", 2, [0.22, 0.34, 0.05+table_dim[2]], [0.0, 0.0, 0.0], [0.05, 0.05], 3]
		Objects[1] = [1, "/mesh/dove_beauty_bar/dove_beauty_bar.obj", "doveBar", 
				"normal", 2, [0.03, 0.5, 0.05+table_dim[2]], [math.pi/2, 0.0, 
														math.pi/4.3], [0.03, 0.06, 0.01], 3]
		Objects[2] = [2, "/mesh/dasani_water_bottle/dasani_water_bottle.obj", "waterBottle", 
				"normal", 2, [-0.05, 0.17, 0.2+table_dim[2]], [math.pi/2, 0.0, 
														math.pi/2.6], [0.05, 0.04, 0.3], 3]
		Objects[3] = [3, "/mesh/ticonderoga_12_pencils/ticonderoga_12_pencils.obj", "pencils", 
				"normal", 1, [0.16, -0.04, 0.01+table_dim[2]], 
									[0.0, math.pi/2, -math.pi/3.9], [0.05, 0.04, 0.25], 3]
		Objects[4] = [4, "/mesh/elmers_washable_no_run_school_glue/elmers_washable_no_run_school_glue.obj", 
				"glue", "invisible", 1.5, [0.09, 0.25, 0.13+table_dim[2]], 
									[math.pi / 2, 0.0, 25*math.pi/180], [0.07, 0.07, 0.5], 1]
		Objects[5] = [5, "/mesh/folgers_classic_roast_coffee/folgers_classic_roast_coffee.obj", 
				"coffeeJar", "normal", 2, [-0.2, -0.14, 0.12+table_dim[2]], 
										[math.pi/2, 0.0, -math.pi/3.9], [0.026, 0.01, 0.24], 3]
		Objects[6] = [6, "/mesh/up_glucose_bottle/up_glucose_bottle.obj", 
				"glucoseBottle", "normal", 1.5, [0.26, 0.15, 0.1+table_dim[2]], 
											[math.pi / 2, 0.0, math.pi], [0.04, 0.04, 0.12], 3]
		Objects[7] = [7, "/mesh/crayola_24_ct/crayola_24_ct.obj", 
				"crayola", "normal", 2, [-0.26, 0.17, 0.06+table_dim[2]], 
														[0.0, 0.0, 0.0], [0.04, 0.03, 0.18], 3]
		Objects[8] = [8, "/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", 
				"tissueBox", "normal", 1.5, [-0.27, 0.4, 0.1+table_dim[2]], 
											[0.0, math.pi/2, -math.pi/3], [0.03, 0.03, 0.24], 3]
		Objects[9] = [9, "/mesh/soft_white_lightbulb/soft_white_lightbulb.obj", 
				"lightbulb", "phantom", 1.5, [-0.2, 0.36, 0.05+table_dim[2]], 
											[0.0, 0.0, math.pi/10.0], [0.032, 0.035, 0.06], 3]
		Objects[10] = [10, "/mesh/dr_browns_bottle_brush/dr_browns_bottle_brush.obj", 
				"brush", "normal", 1, [-0.07, -0.33, 0.02+table_dim[2]], 
											[0.0, 0.0, -math.pi / 4.5], [0.028, 0.02, 0.2], 3]
		# pick goal offset
		goalPos_offset = [-0.03, 0.0, 0.127]


### Collect meshes for all the objects ###
meshDict = dict()
for i in xrange(len(Objects)):
	meshDict[i] = utils.createMesh(Objects[i][0], Objects[i][1], Objects[i][2], 
		Objects[i][3], Objects[i][4], Objects[i][5], Objects[i][6], Objects[i][7], Objects[i][8])

# generate goal state
goal_pose_pos = []
for i in xrange(len(goalPos_offset)):
	goal_pose_pos.append(Objects[0][5][i] + goalPos_offset[i])
goal_pose_quat = [1, 0, 0, 0]

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



time.sleep(10000)