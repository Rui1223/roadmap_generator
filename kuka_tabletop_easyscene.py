from __future__ import division
import pybullet as p
import time
import numpy as np
import pybullet_data
import utils
import IPython
import math

from scipy import spatial

import cPickle as pickle

cam_info_filename = "/home/rui/Documents/research/roadmap_generator/cam_info.txt"

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.loadURDF("plane.urdf")
# p.setGravity(0, 0, -10)
kuka_ee_idx = 8

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


table_c = p.createCollisionShape(shapeType=p.GEOM_BOX,halfExtents=np.array([1.5, 2.0, 0.2])/2)
table_v = p.createVisualShape(shapeType=p.GEOM_BOX,halfExtents=np.array([1.5, 2.0, 0.2])/2)
p.createMultiBody(baseCollisionShapeIndex=table_c,baseVisualShapeIndex=table_v,basePosition=[0, 0, -0.1])

nHypotheses = 3
############################## Poses for 13 objects ######################################

####### green box ########
# the shape of the object
gbox_c = p.createCollisionShape(shapeType=p.GEOM_BOX,halfExtents=np.array([0.1, 0.1, 0.1])/2)
# the true pose of the object
gbox_TruePosition = [0.22, -0.4, 0.05]
gbox_TrueOrientation = [0, 0, 0.2419219, 1]
# Uncertainty #
# For the green box, the uncertainty lies in location (x,y) and the orientation along z axis.
gbox_px_variance = 0.05
gbox_py_variance = 0.01
gbox_qz_variance = 0.03
# Belief distribution for the green box: the poses are pretty close, for each pose I have kind of even belief
gbox_belief_variance = 1
gbox_belief_space = utils.belief_space_estimator(gbox_belief_variance, nHypotheses)
print "the belief space for the green box: "
print gbox_belief_space

# generate poses (includes the true pose)
gbox_pose_pos = []
gbox_pose_ori = []
for i in range(0, nHypotheses-1):	
	temp_px = float(np.random.normal(gbox_TruePosition[0], gbox_px_variance, 1))
	temp_py = float(np.random.normal(gbox_TruePosition[1], gbox_py_variance, 1))
	temp_qz = float(np.random.normal(gbox_TrueOrientation[2], gbox_qz_variance, 1))
	gbox_pose_pos.append([temp_px, temp_py, gbox_TruePosition[2]])
	gbox_pose_ori.append([gbox_TrueOrientation[0], gbox_TrueOrientation[1], temp_qz, gbox_TrueOrientation[3]])
gbox_pose_pos.append(gbox_TruePosition)
gbox_pose_ori.append(gbox_TrueOrientation)

print gbox_pose_pos
print gbox_pose_ori
print "----------------------"

# create the poses in the scene
for i in range(0, nHypotheses):
	gbox_v = p.createVisualShape(shapeType=p.GEOM_BOX,halfExtents=np.array([0.1, 0.1, 0.1])/2, rgbaColor=[0, 255, 0, gbox_belief_space[i]])
	p.createMultiBody(baseCollisionShapeIndex=gbox_c,baseVisualShapeIndex=gbox_v,basePosition=gbox_pose_pos[i],baseOrientation=gbox_pose_ori[i])
##########################

####### red sphere #######
# the shape of the object
rsphere_c = p.createCollisionShape(shapeType=p.GEOM_SPHERE,radius=0.07)
# the true pose of the object
rsphere_TruePosition = [0.16, 0.3, 0.07]
rsphere_TrueOrientation = [0, 0, -0.2419219, 1]
# Uncertainty #
# For the sphere, the uncertainty lies in location (x,y) and the orientation does not matter since it's symmetric
rsphere_px_variance = 0.08
rsphere_py_variance = 0.13
# Belief distribution for the red sphere: the poses are not that close, for each pose the belief has some obvious difference
rsphere_belief_variance = 7
rsphere_belief_space = utils.belief_space_estimator(rsphere_belief_variance, nHypotheses)
print "the belief space for the red sphere: "
print rsphere_belief_space

# generate poses (includes the true pose)
rsphere_pose_pos = []
rsphere_pose_ori = []
for i in range(0, nHypotheses-1):	
	temp_px = float(np.random.normal(rsphere_TruePosition[0], rsphere_px_variance, 1))
	temp_py = float(np.random.normal(rsphere_TruePosition[1], rsphere_py_variance, 1))
	rsphere_pose_pos.append([temp_px, temp_py, rsphere_TruePosition[2]])
	rsphere_pose_ori.append(rsphere_TrueOrientation)
rsphere_pose_pos.append(rsphere_TruePosition)
rsphere_pose_ori.append(rsphere_TrueOrientation)

print rsphere_pose_pos
print rsphere_pose_ori
print "----------------------"

# create the poses in the scene
for i in range(0, nHypotheses):
	rsphere_v = p.createVisualShape(shapeType=p.GEOM_SPHERE,radius=0.07, rgbaColor=[255, 0, 0, rsphere_belief_space[i]])
	p.createMultiBody(baseCollisionShapeIndex=rsphere_c,baseVisualShapeIndex=rsphere_v,basePosition=rsphere_pose_pos[i],baseOrientation=rsphere_pose_ori[i])
#############################

######### crayola #########
# the shape of the object
crayola_c = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="/mesh/crayola_24_ct/crayola_24_ct.obj", meshScale=[2, 2, 2])
# the true pose of the object
crayola_TruePosition = [-0.4, -0.7, 0.1]
crayola_EulerAngle_x = math.pi / 2
crayola_EulerAngle_y = 0.0
crayola_EulerAngle_z = 0.0
crayola_TrueOrientation = utils.euler_to_quaternion(crayola_EulerAngle_z, crayola_EulerAngle_y, crayola_EulerAngle_x)
# Uncertainty #
# For crayola, the uncertainty lies in location (x,y) and orientation around z axis
# the location are close
# the orientation varies a lot
crayola_px_variance = 0.03
crayola_py_variance = 0.03
crayola_oz_variance = 15*math.pi/180
# Belief distribution for the crayola: Have some distinction, but not very condifident
crayola_belief_variance = 4
crayola_belief_space = utils.belief_space_estimator(crayola_belief_variance, nHypotheses)
print "the belief space for the crayola: "
print crayola_belief_space

# generate poses (includes the true pose)
crayola_pose_pos = []
crayola_pose_ori = []
for i in range(0, nHypotheses-1):	
	temp_px = float(np.random.normal(crayola_TruePosition[0], crayola_px_variance, 1))
	temp_py = float(np.random.normal(crayola_TruePosition[1], crayola_py_variance, 1))
	temp_oz = float(np.random.normal(crayola_EulerAngle_z, crayola_oz_variance, 1))
	crayola_pose_pos.append([temp_px, temp_py, crayola_TruePosition[2]])
	crayola_pose_ori.append(utils.euler_to_quaternion(temp_oz, crayola_EulerAngle_y, crayola_EulerAngle_x))
crayola_pose_pos.append(crayola_TruePosition)
crayola_pose_ori.append(crayola_TrueOrientation)

print crayola_pose_pos
print crayola_pose_ori
print "----------------------"

# create the poses in the scene
crayola_v = p.createVisualShape(shapeType=p.GEOM_MESH,fileName="/mesh/crayola_24_ct/crayola_24_ct.obj", meshScale=[2, 2, 2])
for i in range(0, nHypotheses):
	p.createMultiBody(baseCollisionShapeIndex=crayola_c,baseVisualShapeIndex=crayola_v,basePosition=crayola_pose_pos[i],baseOrientation=crayola_pose_ori[i])
##########################

########### dasani_water_bottle ##########
# the shape of the object
dasani_water_bottle_c = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="/mesh/dasani_water_bottle/dasani_water_bottle.obj", meshScale=[2, 2, 2])
# the true pose of the object
dasani_water_bottle_TruePosition = [0.5, -0.4, 0.07]
dasani_water_bottle_EulerAngle_x = 0.0
dasani_water_bottle_EulerAngle_y = 0.0
dasani_water_bottle_EulerAngle_z = 0.0
dasani_water_bottle_TrueOrientation = utils.euler_to_quaternion(dasani_water_bottle_EulerAngle_z, dasani_water_bottle_EulerAngle_y, dasani_water_bottle_EulerAngle_x)
# Uncertainty #
# For dasani_water_bottle, the uncertainty lies in location (x,y) and orientation around z axis
# the location are close
# the orientation varies a lot
dasani_water_bottle_px_variance = 0.06
dasani_water_bottle_py_variance = 0.1
dasani_water_bottle_oz_variance = 30*math.pi/180
# Belief distribution for the dasani_water_bottle: Have some distinction, but not very condifident
dasani_water_bottle_belief_variance = 4
dasani_water_bottle_belief_space = utils.belief_space_estimator(dasani_water_bottle_belief_variance, nHypotheses)
print "the belief space for the dasani_water_bottle: "
print dasani_water_bottle_belief_space

# generate poses (includes the true pose)
dasani_water_bottle_pose_pos = []
dasani_water_bottle_pose_ori = []
for i in range(0, nHypotheses-1):	
	temp_px = float(np.random.normal(dasani_water_bottle_TruePosition[0], dasani_water_bottle_px_variance, 1))
	temp_py = float(np.random.normal(dasani_water_bottle_TruePosition[1], dasani_water_bottle_py_variance, 1))
	temp_oz = float(np.random.normal(dasani_water_bottle_EulerAngle_z, dasani_water_bottle_oz_variance, 1))
	dasani_water_bottle_pose_pos.append([temp_px, temp_py, dasani_water_bottle_TruePosition[2]])
	dasani_water_bottle_pose_ori.append(utils.euler_to_quaternion(temp_oz, dasani_water_bottle_EulerAngle_y, dasani_water_bottle_EulerAngle_x))
dasani_water_bottle_pose_pos.append(dasani_water_bottle_TruePosition)
dasani_water_bottle_pose_ori.append(dasani_water_bottle_TrueOrientation)

print dasani_water_bottle_pose_pos
print dasani_water_bottle_pose_ori
print "----------------------"

dasani_water_bottle_v = p.createVisualShape(shapeType=p.GEOM_MESH,fileName="/mesh/dasani_water_bottle/dasani_water_bottle.obj", meshScale=[2, 2, 2])
for i in range(0, nHypotheses):
	p.createMultiBody(baseCollisionShapeIndex=dasani_water_bottle_c,baseVisualShapeIndex=dasani_water_bottle_v,basePosition=dasani_water_bottle_pose_pos[i],baseOrientation=dasani_water_bottle_pose_ori[i])
################################

############ dove_beauty_bar ###############
# the shape of the object
dove_beauty_bar_c = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="/mesh/dove_beauty_bar/dove_beauty_bar.obj", meshScale=[2, 2, 2])
# the true pose of the object
dove_beauty_bar_TruePosition = [0.3, 0.6, 0.06]
dove_beauty_bar_EulerAngle_x = math.pi / 2
dove_beauty_bar_EulerAngle_y = 0.0
dove_beauty_bar_EulerAngle_z = 0.0
dove_beauty_bar_TrueOrientation = utils.euler_to_quaternion(dove_beauty_bar_EulerAngle_z, dove_beauty_bar_EulerAngle_y, dove_beauty_bar_EulerAngle_x)
# Uncertainty #
# For dove_beauty_bar, the uncertainty lies in location (x,y) and orientation around z axis
# the location are close
# the orientation varies a lot
dove_beauty_bar_px_variance = 0.04
dove_beauty_bar_py_variance = 0.08
dove_beauty_bar_oz_variance = 30*math.pi/180
# Belief distribution for the dove_beauty_bar Have some distinction, but not very condifident
dove_beauty_bar_belief_variance = 4
dove_beauty_bar_belief_space = utils.belief_space_estimator(dove_beauty_bar_belief_variance, nHypotheses)
print "the belief space for the dove_beauty_bar: "
print dove_beauty_bar_belief_space

# generate poses (includes the true pose)
dove_beauty_bar_pose_pos = []
dove_beauty_bar_pose_ori = []
for i in range(0, nHypotheses-1):	
	temp_px = float(np.random.normal(dove_beauty_bar_TruePosition[0], dove_beauty_bar_px_variance, 1))
	temp_py = float(np.random.normal(dove_beauty_bar_TruePosition[1], dove_beauty_bar_py_variance, 1))
	temp_oz = float(np.random.normal(dove_beauty_bar_EulerAngle_z, dove_beauty_bar_oz_variance, 1))
	dove_beauty_bar_pose_pos.append([temp_px, temp_py, dove_beauty_bar_TruePosition[2]])
	dove_beauty_bar_pose_ori.append(utils.euler_to_quaternion(temp_oz, dove_beauty_bar_EulerAngle_y, dove_beauty_bar_EulerAngle_x))
dove_beauty_bar_pose_pos.append(dove_beauty_bar_TruePosition)
dove_beauty_bar_pose_ori.append(dove_beauty_bar_TrueOrientation)

print dove_beauty_bar_pose_pos
print dove_beauty_bar_pose_ori
print "----------------------"

dove_beauty_bar_v = p.createVisualShape(shapeType=p.GEOM_MESH,fileName="/mesh/dove_beauty_bar/dove_beauty_bar.obj", meshScale=[2, 2, 2])
for i in range(0, nHypotheses):
	p.createMultiBody(baseCollisionShapeIndex=dove_beauty_bar_c,baseVisualShapeIndex=dove_beauty_bar_v,basePosition=dove_beauty_bar_pose_pos[i],baseOrientation=dove_beauty_bar_pose_ori[i])
###########################

############ dove_beauty_bar ###############
# the shape of the object
dr_browns_bottle_brush_c = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="/mesh/dr_browns_bottle_brush/dr_browns_bottle_brush.obj", meshScale=[1, 1, 1])
# the true pose of the object
dr_browns_bottle_brush_TruePosition = [0.3, -0.76, 0.025]
dr_browns_bottle_brush_EulerAngle_x = 0.0
dr_browns_bottle_brush_EulerAngle_y = 0.0
dr_browns_bottle_brush_EulerAngle_z = -math.pi / 2.5
dr_browns_bottle_brush_TrueOrientation = utils.euler_to_quaternion(dr_browns_bottle_brush_EulerAngle_z, dr_browns_bottle_brush_EulerAngle_y, dr_browns_bottle_brush_EulerAngle_x)
# Uncertainty #
# For dr_browns_bottle_brush, the uncertainty lies in location (x,y) and orientation around z axis
# the location are close
# the orientation varies a lot
dr_browns_bottle_brush_px_variance = 0.04
dr_browns_bottle_brush_py_variance = 0.08
dr_browns_bottle_brush_oz_variance = 20*math.pi/180
# Belief distribution for the dr_browns_bottle_brush Have some distinction, but not very condifident
dr_browns_bottle_brush_belief_variance = 7
dr_browns_bottle_brush_belief_space = utils.belief_space_estimator(dr_browns_bottle_brush_belief_variance, nHypotheses)
print "the belief space for the dr_browns_bottle_brush: "
print dr_browns_bottle_brush_belief_space

# generate poses (includes the true pose)
dr_browns_bottle_brush_pose_pos = []
dr_browns_bottle_brush_pose_ori = []
for i in range(0, nHypotheses-1):	
	temp_px = float(np.random.normal(dr_browns_bottle_brush_TruePosition[0], dr_browns_bottle_brush_px_variance, 1))
	temp_py = float(np.random.normal(dr_browns_bottle_brush_TruePosition[1], dr_browns_bottle_brush_py_variance, 1))
	temp_oz = float(np.random.normal(dr_browns_bottle_brush_EulerAngle_z, dr_browns_bottle_brush_oz_variance, 1))
	dr_browns_bottle_brush_pose_pos.append([temp_px, temp_py, dr_browns_bottle_brush_TruePosition[2]])
	dr_browns_bottle_brush_pose_ori.append(utils.euler_to_quaternion(temp_oz, dr_browns_bottle_brush_EulerAngle_y, dr_browns_bottle_brush_EulerAngle_x))
dr_browns_bottle_brush_pose_pos.append(dr_browns_bottle_brush_TruePosition)
dr_browns_bottle_brush_pose_ori.append(dr_browns_bottle_brush_TrueOrientation)

print dr_browns_bottle_brush_pose_pos
print dr_browns_bottle_brush_pose_ori
print "----------------------"

dr_browns_bottle_brush_v = p.createVisualShape(shapeType=p.GEOM_MESH,fileName="/mesh/dr_browns_bottle_brush/dr_browns_bottle_brush.obj", meshScale=[1, 1, 1])
for i in range(0, nHypotheses):
	p.createMultiBody(baseCollisionShapeIndex=dr_browns_bottle_brush_c,baseVisualShapeIndex=dr_browns_bottle_brush_v,basePosition=dr_browns_bottle_brush_pose_pos[i],baseOrientation=dr_browns_bottle_brush_pose_ori[i])
##########################

############ elmers_washable_no_run_school_glue ###############
# the shape of the object
elmers_washable_no_run_school_glue_c = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="/mesh/elmers_washable_no_run_school_glue/elmers_washable_no_run_school_glue.obj", meshScale=[2, 2, 2])
# the true pose of the object
elmers_washable_no_run_school_glue_TruePosition = [-0.4, 0.7, 0.14]
elmers_washable_no_run_school_glue_EulerAngle_x = math.pi / 2
elmers_washable_no_run_school_glue_EulerAngle_y = 0.0
elmers_washable_no_run_school_glue_EulerAngle_z = -math.pi / 2
elmers_washable_no_run_school_glue_TrueOrientation = utils.euler_to_quaternion(elmers_washable_no_run_school_glue_EulerAngle_z, elmers_washable_no_run_school_glue_EulerAngle_y, elmers_washable_no_run_school_glue_EulerAngle_x)
# Uncertainty #
# For elmers_washable_no_run_school_glue, the uncertainty lies in location (x,y) and orientation around z axis
# the location are close
# the orientation varies a lot
elmers_washable_no_run_school_glue_px_variance = 0.04
elmers_washable_no_run_school_glue_py_variance = 0.13
elmers_washable_no_run_school_glue_oz_variance = 25*math.pi/180
# Belief distribution for the elmers_washable_no_run_school_glue Have some distinction, relatively high condifident
elmers_washable_no_run_school_glue_belief_variance = 8
elmers_washable_no_run_school_glue_belief_space = utils.belief_space_estimator(elmers_washable_no_run_school_glue_belief_variance, nHypotheses)
print "the belief space for the elmers_washable_no_run_school_glue: "
print elmers_washable_no_run_school_glue_belief_space

# generate poses (includes the true pose)
elmers_washable_no_run_school_glue_pose_pos = []
elmers_washable_no_run_school_glue_pose_ori = []
for i in range(0, nHypotheses-1):	
	temp_px = float(np.random.normal(elmers_washable_no_run_school_glue_TruePosition[0], elmers_washable_no_run_school_glue_px_variance, 1))
	temp_py = float(np.random.normal(elmers_washable_no_run_school_glue_TruePosition[1], elmers_washable_no_run_school_glue_py_variance, 1))
	temp_oz = float(np.random.normal(elmers_washable_no_run_school_glue_EulerAngle_z, elmers_washable_no_run_school_glue_oz_variance, 1))
	elmers_washable_no_run_school_glue_pose_pos.append([temp_px, temp_py, elmers_washable_no_run_school_glue_TruePosition[2]])
	elmers_washable_no_run_school_glue_pose_ori.append(utils.euler_to_quaternion(temp_oz, elmers_washable_no_run_school_glue_EulerAngle_y, elmers_washable_no_run_school_glue_EulerAngle_x))
elmers_washable_no_run_school_glue_pose_pos.append(elmers_washable_no_run_school_glue_TruePosition)
elmers_washable_no_run_school_glue_pose_ori.append(elmers_washable_no_run_school_glue_TrueOrientation)

print elmers_washable_no_run_school_glue_pose_pos
print elmers_washable_no_run_school_glue_pose_ori
print "----------------------"

elmers_washable_no_run_school_glue_v = p.createVisualShape(shapeType=p.GEOM_MESH,fileName="/mesh/elmers_washable_no_run_school_glue/elmers_washable_no_run_school_glue.obj", meshScale=[2, 2, 2])
for i in range(0, nHypotheses):
	p.createMultiBody(baseCollisionShapeIndex=elmers_washable_no_run_school_glue_c,baseVisualShapeIndex=elmers_washable_no_run_school_glue_v,basePosition=elmers_washable_no_run_school_glue_pose_pos[i],baseOrientation=elmers_washable_no_run_school_glue_pose_ori[i])
######################

############ folgers_classic_roast_coffee ###############
# the shape of the object
folgers_classic_roast_coffee_c = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="/mesh/folgers_classic_roast_coffee/folgers_classic_roast_coffee.obj", meshScale=[2, 2, 2])
# the true pose of the object
folgers_classic_roast_coffee_TruePosition = [-0.15, 0.45, 0.12]
folgers_classic_roast_coffee_EulerAngle_x = math.pi / 2
folgers_classic_roast_coffee_EulerAngle_y = 0.0
folgers_classic_roast_coffee_EulerAngle_z = -math.pi / 2
folgers_classic_roast_coffee_TrueOrientation = utils.euler_to_quaternion(folgers_classic_roast_coffee_EulerAngle_z, folgers_classic_roast_coffee_EulerAngle_y, folgers_classic_roast_coffee_EulerAngle_x)
# Uncertainty #
# For folgers_classic_roast_coffee, the uncertainty lies in location (x,y) and orientation around z axis
# the location are close
# the orientation does not matter
folgers_classic_roast_coffee_px_variance = 0.07
folgers_classic_roast_coffee_py_variance = 0.07
# Belief distribution for the folgers_classic_roast_coffee: Have similar belief
folgers_classic_roast_coffee_belief_variance = 2
folgers_classic_roast_coffee_belief_space = utils.belief_space_estimator(folgers_classic_roast_coffee_belief_variance, nHypotheses)
print "the belief space for the folgers_classic_roast_coffee: "
print folgers_classic_roast_coffee_belief_space

# generate poses (includes the true pose)
folgers_classic_roast_coffee_pose_pos = []
folgers_classic_roast_coffee_pose_ori = []
for i in range(0, nHypotheses-1):	
	temp_px = float(np.random.normal(folgers_classic_roast_coffee_TruePosition[0], folgers_classic_roast_coffee_px_variance, 1))
	temp_py = float(np.random.normal(folgers_classic_roast_coffee_TruePosition[1], folgers_classic_roast_coffee_py_variance, 1))
	folgers_classic_roast_coffee_pose_pos.append([temp_px, temp_py, folgers_classic_roast_coffee_TruePosition[2]])
folgers_classic_roast_coffee_pose_pos.append(folgers_classic_roast_coffee_TruePosition)

print folgers_classic_roast_coffee_pose_pos
print folgers_classic_roast_coffee_TrueOrientation
print "----------------------"

folgers_classic_roast_coffee_v = p.createVisualShape(shapeType=p.GEOM_MESH,fileName="/mesh/folgers_classic_roast_coffee/folgers_classic_roast_coffee.obj", meshScale=[2, 2, 2])
for i in range(0, nHypotheses):
	p.createMultiBody(baseCollisionShapeIndex=folgers_classic_roast_coffee_c,baseVisualShapeIndex=folgers_classic_roast_coffee_v,basePosition=folgers_classic_roast_coffee_pose_pos[i],baseOrientation=folgers_classic_roast_coffee_TrueOrientation)
##########################

############ kleenex_tissue_box ###############
# the shape of the object
kleenex_tissue_box_c = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", meshScale=[1.5, 1.5, 1.5])
# the true pose of the object
kleenex_tissue_box_TruePosition = [-0.3, -0.3, 0.09]
kleenex_tissue_box_EulerAngle_x = 0.0
kleenex_tissue_box_EulerAngle_y = math.pi / 2
kleenex_tissue_box_EulerAngle_z = -math.pi / 5
kleenex_tissue_box_TrueOrientation = utils.euler_to_quaternion(kleenex_tissue_box_EulerAngle_z, kleenex_tissue_box_EulerAngle_y, kleenex_tissue_box_EulerAngle_x)
# Uncertainty #
# For kleenex_tissue_box, the uncertainty lies in location (x,y) and orientation around z axis
# the location are not that close
# the orientation varies a lot
kleenex_tissue_box_px_variance = 0.1
kleenex_tissue_box_py_variance = 0.14
kleenex_tissue_box_oz_variance = 20*math.pi/180
# Belief distribution for the kleenex_tissue_box Have some distinction, but not very condifident
kleenex_tissue_box_belief_variance = 5
kleenex_tissue_box_belief_space = utils.belief_space_estimator(kleenex_tissue_box_belief_variance, nHypotheses)
print "the belief space for the kleenex_tissue_box: "
print kleenex_tissue_box_belief_space

# generate poses (includes the true pose)
kleenex_tissue_box_pose_pos = []
kleenex_tissue_box_pose_ori = []
for i in range(0, nHypotheses-1):	
	temp_px = float(np.random.normal(kleenex_tissue_box_TruePosition[0], kleenex_tissue_box_px_variance, 1))
	temp_py = float(np.random.normal(kleenex_tissue_box_TruePosition[1], kleenex_tissue_box_py_variance, 1))
	temp_oz = float(np.random.normal(kleenex_tissue_box_EulerAngle_z, kleenex_tissue_box_oz_variance, 1))
	kleenex_tissue_box_pose_pos.append([temp_px, temp_py, kleenex_tissue_box_TruePosition[2]])
	kleenex_tissue_box_pose_ori.append(utils.euler_to_quaternion(temp_oz, kleenex_tissue_box_EulerAngle_y, kleenex_tissue_box_EulerAngle_x))
kleenex_tissue_box_pose_pos.append(kleenex_tissue_box_TruePosition)
kleenex_tissue_box_pose_ori.append(kleenex_tissue_box_TrueOrientation)

print kleenex_tissue_box_pose_pos
print kleenex_tissue_box_pose_ori
print "----------------------"

kleenex_tissue_box_v = p.createVisualShape(shapeType=p.GEOM_MESH,fileName="/mesh/kleenex_tissue_box/kleenex_tissue_box.obj", meshScale=[1.5, 1.5, 1.5])
for i in range(0, nHypotheses):
	p.createMultiBody(baseCollisionShapeIndex=kleenex_tissue_box_c,baseVisualShapeIndex=kleenex_tissue_box_v,basePosition=kleenex_tissue_box_pose_pos[i],baseOrientation=kleenex_tissue_box_pose_ori[i])
####################

############ rawlings_baseball ############
# the shape of the object
rawlings_baseball_c = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="/mesh/rawlings_baseball/rawlings_baseball.obj", meshScale=[1.5, 1.5, 1.5])
# the true pose of the object
rawlings_baseball_TruePosition = [-0.55, -0.6, 0.05]
rawlings_baseball_EulerAngle_x = 0.0
rawlings_baseball_EulerAngle_y = 0.0
rawlings_baseball_EulerAngle_z = 0.0
rawlings_baseball_TrueOrientation = utils.euler_to_quaternion(rawlings_baseball_EulerAngle_z, rawlings_baseball_EulerAngle_y, rawlings_baseball_EulerAngle_x)
# Uncertainty #
# For rawlings_baseball, the uncertainty lies in location (x,y) and orientation around z axis
# the location are not that close
# the orientation varies a lot (does not mateer)
rawlings_baseball_px_variance = 0.06
rawlings_baseball_py_variance = 0.09
rawlings_baseball_oz_variance = 45*math.pi/180
# Belief distribution for the rawlings_baseball: Have some distinction, relatively high confident
rawlings_baseball_belief_variance = 6
rawlings_baseball_belief_space = utils.belief_space_estimator(rawlings_baseball_belief_variance, nHypotheses)
print "the belief space for the rawlings_baseball: "
print rawlings_baseball_belief_space

# generate poses (includes the true pose)
rawlings_baseball_pose_pos = []
rawlings_baseball_pose_ori = []
for i in range(0, nHypotheses-1):	
	temp_px = float(np.random.normal(rawlings_baseball_TruePosition[0], rawlings_baseball_px_variance, 1))
	temp_py = float(np.random.normal(rawlings_baseball_TruePosition[1], rawlings_baseball_py_variance, 1))
	temp_oz = float(np.random.normal(rawlings_baseball_EulerAngle_z, rawlings_baseball_oz_variance, 1))
	rawlings_baseball_pose_pos.append([temp_px, temp_py, rawlings_baseball_TruePosition[2]])
	rawlings_baseball_pose_ori.append(utils.euler_to_quaternion(temp_oz, rawlings_baseball_EulerAngle_y, rawlings_baseball_EulerAngle_x))
rawlings_baseball_pose_pos.append(rawlings_baseball_TruePosition)
rawlings_baseball_pose_ori.append(rawlings_baseball_TrueOrientation)

print rawlings_baseball_pose_pos
print rawlings_baseball_pose_ori
print "----------------------"

rawlings_baseball_v = p.createVisualShape(shapeType=p.GEOM_MESH,fileName="/mesh/rawlings_baseball/rawlings_baseball.obj", meshScale=[1.5, 1.5, 1.5])
for i in range(0, nHypotheses):
	p.createMultiBody(baseCollisionShapeIndex=rawlings_baseball_c,baseVisualShapeIndex=rawlings_baseball_v,basePosition=rawlings_baseball_pose_pos[i],baseOrientation=rawlings_baseball_pose_ori[i])
##########################

################ soft_white_lightbulb #################
# the shape of the object
soft_white_lightbulb_c = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="/mesh/soft_white_lightbulb/soft_white_lightbulb.obj", meshScale=[1.5, 1.5, 1.5])
# the true pose of the object
soft_white_lightbulb_TruePosition = [0, -0.5, 0.05]
soft_white_lightbulb_EulerAngle_x = 0.0
soft_white_lightbulb_EulerAngle_y = 0.0
soft_white_lightbulb_EulerAngle_z = 0.0
soft_white_lightbulb_TrueOrientation = utils.euler_to_quaternion(soft_white_lightbulb_EulerAngle_z, soft_white_lightbulb_EulerAngle_y, soft_white_lightbulb_EulerAngle_x)
# Uncertainty #
# For soft_white_lightbulb, the uncertainty lies in location (x,y) and orientation around z axis
# the location are close
# the orientation varies a lot 
soft_white_lightbulb_px_variance = 0.04
soft_white_lightbulb_py_variance = 0.02
soft_white_lightbulb_oz_variance = 35*math.pi/180
# Belief distribution for the soft_white_lightbulb: Have some distinction, not quite confident
soft_white_lightbulb_belief_variance = 3
soft_white_lightbulb_belief_space = utils.belief_space_estimator(soft_white_lightbulb_belief_variance, nHypotheses)
print "the belief space for the soft_white_lightbulb: "
print soft_white_lightbulb_belief_space

# generate poses (includes the true pose)
soft_white_lightbulb_pose_pos = []
soft_white_lightbulb_pose_ori = []
for i in range(0, nHypotheses-1):	
	temp_px = float(np.random.normal(soft_white_lightbulb_TruePosition[0], soft_white_lightbulb_px_variance, 1))
	temp_py = float(np.random.normal(soft_white_lightbulb_TruePosition[1], soft_white_lightbulb_py_variance, 1))
	temp_oz = float(np.random.normal(soft_white_lightbulb_EulerAngle_z, soft_white_lightbulb_oz_variance, 1))
	soft_white_lightbulb_pose_pos.append([temp_px, temp_py, soft_white_lightbulb_TruePosition[2]])
	soft_white_lightbulb_pose_ori.append(utils.euler_to_quaternion(temp_oz, soft_white_lightbulb_EulerAngle_y, soft_white_lightbulb_EulerAngle_x))
soft_white_lightbulb_pose_pos.append(soft_white_lightbulb_TruePosition)
soft_white_lightbulb_pose_ori.append(soft_white_lightbulb_TrueOrientation)

print soft_white_lightbulb_pose_pos
print soft_white_lightbulb_pose_ori
print "----------------------"

soft_white_lightbulb_v = p.createVisualShape(shapeType=p.GEOM_MESH,fileName="/mesh/soft_white_lightbulb/soft_white_lightbulb.obj", meshScale=[1.5, 1.5, 1.5])
for i in range(0, nHypotheses):
	p.createMultiBody(baseCollisionShapeIndex=soft_white_lightbulb_c,baseVisualShapeIndex=soft_white_lightbulb_v,basePosition=soft_white_lightbulb_pose_pos[i],baseOrientation=soft_white_lightbulb_pose_ori[i])
#########################################

########## ticonderoga_12_pencils ############
# the shape of the object
ticonderoga_12_pencils_c = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="/mesh/ticonderoga_12_pencils/ticonderoga_12_pencils.obj", meshScale=[1.5, 1.5, 1.5])
# the true pose of the object
ticonderoga_12_pencils_TruePosition = [-0.5, -0.02, 0.01]
ticonderoga_12_pencils_EulerAngle_x = 0.0
ticonderoga_12_pencils_EulerAngle_y = math.pi / 2
ticonderoga_12_pencils_EulerAngle_z = math.pi / 4.5
ticonderoga_12_pencils_TrueOrientation = utils.euler_to_quaternion(ticonderoga_12_pencils_EulerAngle_z, ticonderoga_12_pencils_EulerAngle_y, ticonderoga_12_pencils_EulerAngle_x)
# Uncertainty #
# For ticonderoga_12_pencils, the uncertainty lies in location (x,y) and orientation around z axis
# the location are close
# the orientation varies a lot 
ticonderoga_12_pencils_px_variance = 0.03
ticonderoga_12_pencils_py_variance = 0.01
ticonderoga_12_pencils_oz_variance = 55*math.pi/180
# Belief distribution for the ticonderoga_12_pencils: Have some distinction, not quite confident
ticonderoga_12_pencils_belief_variance = 3
ticonderoga_12_pencils_belief_space = utils.belief_space_estimator(ticonderoga_12_pencils_belief_variance, nHypotheses)
print "the belief space for the ticonderoga_12_pencils: "
print ticonderoga_12_pencils_belief_space

# generate poses (includes the true pose)
ticonderoga_12_pencils_pose_pos = []
ticonderoga_12_pencils_pose_ori = []
for i in range(0, nHypotheses-1):	
	temp_px = float(np.random.normal(ticonderoga_12_pencils_TruePosition[0], ticonderoga_12_pencils_px_variance, 1))
	temp_py = float(np.random.normal(ticonderoga_12_pencils_TruePosition[1], ticonderoga_12_pencils_py_variance, 1))
	temp_oz = float(np.random.normal(ticonderoga_12_pencils_EulerAngle_z, ticonderoga_12_pencils_oz_variance, 1))
	ticonderoga_12_pencils_pose_pos.append([temp_px, temp_py, ticonderoga_12_pencils_TruePosition[2]])
	ticonderoga_12_pencils_pose_ori.append(utils.euler_to_quaternion(temp_oz, ticonderoga_12_pencils_EulerAngle_y, ticonderoga_12_pencils_EulerAngle_x))
ticonderoga_12_pencils_pose_pos.append(ticonderoga_12_pencils_TruePosition)
ticonderoga_12_pencils_pose_ori.append(ticonderoga_12_pencils_TrueOrientation)

print ticonderoga_12_pencils_pose_pos
print ticonderoga_12_pencils_pose_ori
print "----------------------"

ticonderoga_12_pencils_v = p.createVisualShape(shapeType=p.GEOM_MESH,fileName="/mesh/ticonderoga_12_pencils/ticonderoga_12_pencils.obj", meshScale=[1.5, 1.5, 1.5])
for i in range(0, nHypotheses):
	p.createMultiBody(baseCollisionShapeIndex=ticonderoga_12_pencils_c,baseVisualShapeIndex=ticonderoga_12_pencils_v,basePosition=ticonderoga_12_pencils_pose_pos[i],baseOrientation=ticonderoga_12_pencils_pose_ori[i])
#########################

######## up_glucose_bottle #############
# the shape of the object
up_glucose_bottle_c = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName="/mesh/up_glucose_bottle/up_glucose_bottle.obj", meshScale=[1.5, 1.5, 1.5])
# the true pose of the object
up_glucose_bottle_TruePosition = [-0.56, 0.42, 0.1]
up_glucose_bottle_EulerAngle_x = -math.pi / 2
up_glucose_bottle_EulerAngle_y = 0.0
up_glucose_bottle_EulerAngle_z = math.pi
up_glucose_bottle_TrueOrientation = utils.euler_to_quaternion(up_glucose_bottle_EulerAngle_z, up_glucose_bottle_EulerAngle_y, up_glucose_bottle_EulerAngle_x)
# Uncertainty #
# For up_glucose_bottle, the uncertainty lies in location (x,y) and orientation around z axis
# the location are not that close
# the orientation varies a little (does not matter) 
up_glucose_bottle_px_variance = 0.02
up_glucose_bottle_py_variance = 0.06
up_glucose_bottle_oz_variance = 25*math.pi/180
# Belief distribution for the up_glucose_bottle: Have some distinction, not quite confident
up_glucose_bottle_belief_variance = 3
up_glucose_bottle_belief_space = utils.belief_space_estimator(up_glucose_bottle_belief_variance, nHypotheses)
print "the belief space for the up_glucose_bottle: "
print up_glucose_bottle_belief_space

# generate poses (includes the true pose)
up_glucose_bottle_pose_pos = []
up_glucose_bottle_pose_ori = []
for i in range(0, nHypotheses-1):	
	temp_px = float(np.random.normal(up_glucose_bottle_TruePosition[0], up_glucose_bottle_px_variance, 1))
	temp_py = float(np.random.normal(up_glucose_bottle_TruePosition[1], up_glucose_bottle_py_variance, 1))
	temp_oz = float(np.random.normal(up_glucose_bottle_EulerAngle_z, up_glucose_bottle_oz_variance, 1))
	up_glucose_bottle_pose_pos.append([temp_px, temp_py, up_glucose_bottle_TruePosition[2]])
	up_glucose_bottle_pose_ori.append(utils.euler_to_quaternion(temp_oz, up_glucose_bottle_EulerAngle_y, up_glucose_bottle_EulerAngle_x))
up_glucose_bottle_pose_pos.append(up_glucose_bottle_TruePosition)
up_glucose_bottle_pose_ori.append(up_glucose_bottle_TrueOrientation)

print up_glucose_bottle_pose_pos
print up_glucose_bottle_pose_ori
print "----------------------"

up_glucose_bottle_v = p.createVisualShape(shapeType=p.GEOM_MESH,fileName="/mesh/up_glucose_bottle/up_glucose_bottle.obj", meshScale=[1.5, 1.5, 1.5])
for i in range(0, nHypotheses):
	p.createMultiBody(baseCollisionShapeIndex=up_glucose_bottle_c,baseVisualShapeIndex=up_glucose_bottle_v,basePosition=up_glucose_bottle_pose_pos[i],baseOrientation=up_glucose_bottle_pose_ori[i])
#############################
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
	ikSolution = p.calculateInverseKinematics(kukaID,kuka_ee_idx,ee_trans[i],ee_quats[i])

	for j in range(1,8):
		result = p.resetJointState(kukaID,j,ikSolution[j-1])

	p.stepSimulation()
	# time.sleep(1)
	contacts = p.getContactPoints(kukaID)
	if len(contacts) == 0:
		nodes.append(ikSolution)

for i in range(0, num_poses):
	# Use NULL space by specifying joint limits
	ikSolution = p.calculateInverseKinematics(kukaID,kuka_ee_idx,ee_trans[i],ee_quats[i], ll, ul, jr, home_configuration)

	for j in range(1,8):
		result = p.resetJointState(kukaID,j,ikSolution[j-1])

	p.stepSimulation()
	# time.sleep(1)
	contacts = p.getContactPoints(kukaID)
	if len(contacts) == 0:
		nodes.append(ikSolution)

print (len(nodes))
time.sleep(5)

# tree = spatial.KDTree(nodes)

# num_neighbors = 5

# init_cfg = nodes[10]
# knn = tree.query(init_cfg, k=num_neighbors, p=2)
# print (knn[1])

# for index in knn[1]:
		
# 	# reset to initial config
# 	for j in range(1,8):
# 		result = p.resetJointState(kukaID,j,init_cfg[j-1])
# 	p.stepSimulation()

# 	final_cfg = nodes[index]

# 	print ('init_cfg: ', init_cfg)
# 	print ('final_cfg: ', final_cfg)

# 	p.setJointMotorControlArray(kukaID, range(1, 8), controlMode=p.POSITION_CONTROL, targetPositions = final_cfg)

# 	for i in range(0, 100):
# 		p.stepSimulation()
# 		time.sleep(1/240.0)

# 	print ('done')

# with open('knn.pickle', 'wb') as handle:
#     pickle.dump(knn, handle)


time.sleep(10000)
#p.disconnect()
