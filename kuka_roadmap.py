import pybullet as p
import time
import numpy as np
import pybullet_data

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

home_position = [0,0,0,-np.pi/2,0,np.pi/2,0]
for i in range(1,8):
	result = p.resetJointState(kukaID,i,home_position[i-1])

# table_c = p.createCollisionShape(shapeType=p.GEOM_BOX,halfExtents=np.array([1.2,1.8,0.2])/2)
# table_v = p.createVisualShape(shapeType=p.GEOM_BOX,halfExtents=np.array([1.2,1.8,0.2])/2)
# p.createMultiBody(baseCollisionShapeIndex=table_c,baseVisualShapeIndex=table_v,basePosition=[0.4,0,-0.3175])

lcamera_c = p.createCollisionShape(shapeType=p.GEOM_BOX,halfExtents=np.array([0.27,0.85,0.9])/2)
lcamera_v = p.createVisualShape(shapeType=p.GEOM_BOX,halfExtents=np.array([0.27,0.85,0.9])/2)
p.createMultiBody(baseCollisionShapeIndex=lcamera_c,baseVisualShapeIndex=lcamera_v,basePosition=[0.685, 0.503, 0.20],baseOrientation=[0, 0, 0.2419219, 0.9702957])

rcamera_c = p.createCollisionShape(shapeType=p.GEOM_BOX,halfExtents=np.array([0.14,0.65,0.9])/2)
rcamera_v = p.createVisualShape(shapeType=p.GEOM_BOX,halfExtents=np.array([0.14,0.65,0.9])/2)
p.createMultiBody(baseCollisionShapeIndex=rcamera_c,baseVisualShapeIndex=rcamera_v,basePosition=[0.685,-0.503,0.20],baseOrientation=[0, 0, -0.2419219, 0.9702957])

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
	ikSolution = p.calculateInverseKinematics(kukaID,kuka_ee_idx,ee_trans[i],ee_quats[i], ll, ul, jr, home_position)

	for j in range(1,8):
		result = p.resetJointState(kukaID,j,ikSolution[j-1])

	p.stepSimulation()
	# time.sleep(1)
	contacts = p.getContactPoints(kukaID)
	if len(contacts) == 0:
		nodes.append(ikSolution)

print (len(nodes))
time.sleep(5)

tree = spatial.KDTree(nodes)

num_neighbors = 5

init_cfg = nodes[10]
knn = tree.query(init_cfg, k=num_neighbors, p=2)
print (knn[1])

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

with open('knn.pickle', 'wb') as handle:
    pickle.dump(knn, handle)


time.sleep(100)
p.disconnect()
