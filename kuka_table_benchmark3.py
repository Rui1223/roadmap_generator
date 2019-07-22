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

cam_info_filename = \
"/home/rui/Documents/research/roadmap_generator/cam_info.txt"

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
kuka_ee_idx = 8

# Introduce Kuka arm and reset the base
######################################################################
kukaID = p.loadURDF("kuka.urdf",[0,0,0],useFixedBase=True)
p.resetBasePositionAndOrientation(kukaID, [-0.6, 0, 0], [0, 0, 0, 1])

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
print "---------Enter to table scene!---------"
# create the static geometries - table
table_c = p.createCollisionShape(shapeType=p.GEOM_BOX,
						halfExtents=np.array([0.7, 1.3, 0.2])/2)
table_v = p.createVisualShape(shapeType=p.GEOM_BOX,
				halfExtents=np.array([0.7, 1.3, 0.2])/2)
tableM = p.createMultiBody(baseCollisionShapeIndex=table_c,
				baseVisualShapeIndex=table_v,basePosition=[0, 0, 0.1])
######################################################################	




# generate hypothesis (including the true pose) for each objects
# 11 objects includes baseball as the target object
##########################################################################
Objects = dict()
# meshfile, meshType, objectRole, scale, true_pos, true_angles, uncertainty, nHypo
Objects[0] = [0, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
	"target", 2, [0.22, 0.34, 0.25], [0.0, 0.0, 0.0], [0.05, 0.05], 3]
Objects[1] = [1, "/mesh/dove_beauty_bar/dove_beauty_bar.obj", "doveBar", 
	"normal", 2, [0.03, 0.5, 0.25], [math.pi/2, 0.0, math.pi/4.3], [0.03, 0.06, 0.01], 3]
Objects[2] = [2, "/mesh/dasani_water_bottle/dasani_water_bottle.obj", "waterBottle", 
	"normal", 2, [-0.05, 0.17, 0.4], [math.pi/2, 0.0, math.pi/2.6], [0.05, 0.04, 0.3], 3]
Objects[3] = [3, "/mesh/ticonderoga_12_pencils/ticonderoga_12_pencils.obj", "pencils", 
	"normal", 1, [0.16, -0.04, 0.21], [0.0, math.pi/2, -math.pi/3.9], [0.05, 0.04, 0.25], 3]
Objects[4] = [4, "/mesh/elmers_washable_no_run_school_glue/elmers_washable_no_run_school_glue.obj", 
"glue", "invisible", 1.5, [0.09, 0.25, 0.33], [math.pi / 2, 0.0, 25*math.pi/180], 
															[0.07, 0.07, 0.5], 1]
Objects[5] = [5, "/mesh/folgers_classic_roast_coffee/folgers_classic_roast_coffee.obj", 
"coffeeJar", "normal", 2, [-0.2, -0.14, 0.32], [math.pi/2, 0.0, -math.pi/3.9], 
														[0.026, 0.01, 0.24], 3]
Objects[6] = [6, "/mesh/up_glucose_bottle/up_glucose_bottle.obj", 
"glucoseBottle", "normal", 1.5, [0.26, 0.15, 0.3], [math.pi / 2, 0.0, math.pi], 
														[0.04, 0.04, 0.12], 3]
Objects[7] = [7, "/mesh/crayola_24_ct/crayola_24_ct.obj", 
"crayola", "normal", 2, [-0.26, 0.17, 0.26], [0.0, 0.0, 0.0], 
														[0.04, 0.03, 0.18], 3]
Objects[8] = [8, "/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", 
"tissueBox", "normal", 1.5, [-0.27, 0.4, 0.3], [0.0, math.pi/2, -math.pi/3], 
														[0.03, 0.03, 0.24], 3]
Objects[9] = [9, "/mesh/soft_white_lightbulb/soft_white_lightbulb.obj", 
"lightbulb", "phantom", 1.5, [-0.2, 0.36, 0.25], [0.0, 0.0, math.pi/10.0], 
														[0.032, 0.035, 0.06], 3]
Objects[10] = [10, "/mesh/dr_browns_bottle_brush/dr_browns_bottle_brush.obj", 
"brush", "normal", 1, [-0.07, -0.33, 0.22], [0.0, 0.0, -math.pi / 4.5], 
														[0.028, 0.02, 0.2], 3]
### Collect meshes for all the objects ###
meshDict = dict()
for i in xrange(len(Objects)):
	meshDict[i] = utils.createMesh(Objects[i][0], Objects[i][1], Objects[i][2], 
		Objects[i][3], Objects[i][4], Objects[i][5], Objects[i][6], 
												Objects[i][7], Objects[i][8])

##########################################################################

ee_trans = []
ee_quats = []
for line in open(cam_info_filename, 'r'):
	[x, y, z, qw, qx, qy, qz] = line.split()
	ee_trans.append(np.array([x, y, z]).astype(float))
	ee_quats.append(np.array([qx, qy, qz, qw]).astype(float))

num_poses = len(ee_trans)
object_postion = np.array([0.55,0,-0.22])
for i in range(0, num_poses):
	ee_trans[i] = ee_trans[i] + object_postion

nodes = []
for i in range(0, num_poses):
	ikSolution = p.calculateInverseKinematics(kukaID,kuka_ee_idx,
													ee_trans[i],ee_quats[i])

	for j in range(1,8):
		result = p.resetJointState(kukaID,j,ikSolution[j-1])

	p.stepSimulation()
	# time.sleep(1)
	contacts = p.getContactPoints(kukaID)
	if len(contacts) == 0:
		nodes.append(ikSolution)

for i in range(0, num_poses):
	# Use NULL space by specifying joint limits
	ikSolution = p.calculateInverseKinematics(kukaID,kuka_ee_idx,ee_trans[i],
									ee_quats[i], ll, ul, jr, home_configuration)

	for j in range(1,8):
		result = p.resetJointState(kukaID,j,ikSolution[j-1])

	p.stepSimulation()
	# time.sleep(1)
	contacts = p.getContactPoints(kukaID)
	if len(contacts) == 0:
		nodes.append(ikSolution)

print (len(nodes))
time.sleep(10000)






'''
baseball_pos = [0.22, 0.34, 0.25]
baseball_angles = [0.0, 0.0, 0.0]
baseballM = utils.createMesh("/mesh/rawlings_baseball/rawlings_baseball.obj", 2, 
										baseball_pos, baseball_angles)

waterBottle_pos = [-0.05, 0.17, 0.4]
waterBottle_angles = [math.pi/2, 0.0, math.pi/2.6]
waterBottleM = utils.createMesh(
	"/mesh/dasani_water_bottle/dasani_water_bottle.obj", 2, 
							waterBottle_pos, waterBottle_angles)

coffeeJar_pos = [-0.2, -0.14, 0.32]
coffeeJar_angles = [math.pi/2, 0.0, -math.pi/3.9]
coffeeJarM = utils.createMesh(
	"/mesh/folgers_classic_roast_coffee/folgers_classic_roast_coffee.obj", 2, 
							coffeeJar_pos, coffeeJar_angles)

pencils_pos = [0.16, -0.04, 0.21]
pencils_angles = [0.0, math.pi/2, -math.pi/3.9]
pencilsM = utils.createMesh(
	"/mesh/ticonderoga_12_pencils/ticonderoga_12_pencils.obj", 1, 
							pencils_pos, pencils_angles)

glucoseBottle_pos = [0.26, 0.15, 0.3]
glucoseBottle_angles = [math.pi / 2, 0.0, math.pi]
glucoseBottleM = utils.createMesh(
	"/mesh/up_glucose_bottle/up_glucose_bottle.obj", 1.5, 
									glucoseBottle_pos, glucoseBottle_angles)

tissueBox_pos = [-0.27, 0.4, 0.3]
tissueBox_angles = [0.0, math.pi/2, -math.pi/3]
tissueBoxM = utils.createMesh(
	"/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", 1.5, 
							tissueBox_pos, tissueBox_angles)

doveBar_pos = [0.03, 0.5, 0.25]
doveBar_angles = [math.pi/2, 0.0, math.pi/4.3]
doveBarM = utils.createMesh("/mesh/dove_beauty_bar/dove_beauty_bar.obj", 2, 
										doveBar_pos, doveBar_angles)

crayola_pos = [-0.26, 0.17, 0.26]
croyola_angles = [0.0, 0.0, 0.0]
crayolaM = utils.createMesh("/mesh/crayola_24_ct/crayola_24_ct.obj", 2, 
										crayola_pos, croyola_angles)


brush_pos = [-0.07, -0.33, 0.22]
brush_angles = [0.0, 0.0, -math.pi / 4.5]
brushM = utils.createMesh(
	"/mesh/dr_browns_bottle_brush/dr_browns_bottle_brush.obj", 1, 
												brush_pos, brush_angles)

### Phantom objects ###
# lightbulb_pos = [-0.2, 0.36, 0.25]
# lightbulb_angles = [0.0, 0.0, math.pi/10.0]
# lightbulbM = utils.createMesh(
# 	"/mesh/soft_white_lightbulb/soft_white_lightbulb.obj", 1.5, 
# 											lightbulb_pos, lightbulb_angles)

### invisible objects ###
glue_pos = [0.09, 0.25, 0.33]
glue_angles = [math.pi / 2, 0.0, 25*math.pi/180]
glueM = utils.createMesh(
"/mesh/elmers_washable_no_run_school_glue/elmers_washable_no_run_school_glue.obj", 
1.5, glue_pos, glue_angles)
'''