from __future__ import division 
import pybullet as p
import numpy as np
import IPython
import math
import time

class Mesh:
	def __init__(self, m, meshType, objIndx, HypoIndx, pos, quat, prob):
		self.m = m
		self.meshType = meshType
		self.objIndx = objIndx
		self.HypoIndx = HypoIndx
		self.pos = pos
		self.quat = quat
		self.prob = prob

	def setProb(self, prob):
		self.prob = prob

## roll (X), pitch (Y), yaw(Z)
## This function come from online wiki link
## https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = round(cy * cp * cr + sy * sp * sr, 4)
    qx = round(cy * cp * sr - sy * sp * cr, 4)
    qy = round(sy * cp * sr + cy * sp * cr, 4)
    qz = round(sy * cp * cr - cy * sp * sr, 4)

    return [qw, qx, qy, qz]

def comp_prob(pos, temp_pos, angles, temp_zangle):
	## paramter tunning for mimicing sensors
	alpha = 15.0
	beta = 15.0
	gamma = 0.003
	return math.exp(-alpha*math.fabs(pos[0]-temp_pos[0]) \
				- beta*math.fabs(pos[1]-temp_pos[1]) \
				- gamma*math.fabs(angles[2]-temp_zangle))

# In our scene, the uncertainty arises from 3 aspects
# 1. position along x-axis
# 2. position along y-axis
# 3. the orientation around z-axis
# So for each object, we have 3-dimensional uncertainty, each of which is 
# represented as gaussian distribution, where the mean is the true pose and
# the variance is where the uncertainty comes from.
# Each object has different uncertainty level. (But with the same dimension)
def createMesh(objIndx, meshfile, meshType, objectRole, scale, 
								pos, angles, uncertainty, nhypo=3):
	# with the mesh file and scale 
	# create the collision and visual shape of the object
	_c = p.createCollisionShape(shapeType=p.GEOM_MESH, 
			fileName=meshfile, meshScale=[scale, scale, scale])
	_v = p.createVisualShape(shapeType=p.GEOM_MESH,
		fileName=meshfile, meshScale=[scale, scale, scale])

	quat = euler_to_quaternion(angles[2], angles[1], angles[0])

	meshes = []
	# first put the true pose of the objects into the meshes
	_m = p.createMultiBody(baseCollisionShapeIndex=_c,baseVisualShapeIndex=_v,
						basePosition=pos,baseOrientation=quat)
	meshes.append(Mesh(_m, meshType, objIndx, 0, pos, quat, 1))

	# for each hypothesis other than the true pose
	for h in xrange(1, nhypo):
		# generate new pos and new quat according to uncertainty
		temp_px = round(float(np.random.normal(pos[0], uncertainty[0])), 4)
		temp_py = round(float(np.random.normal(pos[1], uncertainty[1])), 4)
		temp_pos = [temp_px, temp_py, pos[2]]
		temp_quat = quat
		temp_zangle = angles[2]

		if len(uncertainty) == 3:
			temp_zangle = float(np.random.normal(angles[2], uncertainty[2]))
			temp_quat = euler_to_quaternion(temp_zangle, angles[1], angles[0])



		# compute the unnormalized probability of this hypothesis
		temp_prob = comp_prob(pos, temp_pos, angles, temp_zangle)
		# create the mesh and save it into meshes
		_m = p.createMultiBody(baseCollisionShapeIndex=_c,baseVisualShapeIndex=_v,
						basePosition=temp_pos,baseOrientation=temp_quat)
		meshes.append(Mesh(_m, meshType, objIndx, h, temp_pos, temp_quat, temp_prob))

	# last step: normalize the probability
	temp_probs = []
	for mesh in meshes:
		temp_probs.append(mesh.prob)
	temp_sum = sum(temp_probs)
	for i in xrange(0, len(meshes)):
		meshes[i].setProb(round(temp_probs[i]/temp_sum, 2))

	# print the meshes for the object please
	printPoses(meshes)


	return meshes

def printPoses(meshes):
	print "----------------------------------\n"
	print "Poses for object " + str(meshes[0].objIndx) + ": " + meshes[0].meshType + "\n"
	for mesh in meshes:
		print str(mesh.pos) + " " +str(mesh.quat) + " " + str(mesh.prob) + "\n"
	print "----------------------------------\n"


def collisionCheck_staticG(kukaID, static_geometries):
	isCollision = False
	# loop through all static geometries
	for g in static_geometries:
		contacts = p.getContactPoints(kukaID, g)
		if len(contacts) != 0:
			isCollision = True
			# print "static collision with " + str(g)
			break
	# print "static collision? " + "\t" + str(isCollision)
	return isCollision

def collisionCheck_objects(kukaID, meshDict):
	isCollision = False
	# loop through meshes of each object
	for i in xrange(len(meshDict)):
		for m in meshDict[i]:
			contacts = p.getContactPoints(kukaID, m.m)
			if len(contacts) != 0:
				isCollision = True
				# print "object collision? " + "\t" + m.meshType
				break
		if isCollision:
			break
	# print "__________________________________________"
	return isCollision

def checkEdgeValidity(n1, n2, kukaID, static_geometries):
	step = 5 * math.pi / 180
	nseg = int(math.ceil(max(math.fabs(n1[0]-n2[0]), math.fabs(n1[1]-n2[1]), 
			math.fabs(n1[2]-n2[2]), math.fabs(n1[3]-n2[3]), math.fabs(n1[4]-n2[4]), 
			math.fabs(n1[5]-n2[5]), math.fabs(n1[6]-n2[6]))) / step)
	isEdgeValid = True
	for i in xrange(1, nseg):
		interm_j1 = n1[0] + (n2[0]-n1[0]) / nseg * i
		interm_j2 = n1[1] + (n2[1]-n1[1]) / nseg * i
		interm_j3 = n1[2] + (n2[2]-n1[2]) / nseg * i
		interm_j4 = n1[3] + (n2[3]-n1[3]) / nseg * i
		interm_j5 = n1[4] + (n2[4]-n1[4]) / nseg * i
		interm_j6 = n1[5] + (n2[5]-n1[5]) / nseg * i
		interm_j7 = n1[6] + (n2[6]-n1[6]) / nseg * i
		intermNode = [interm_j1, interm_j2, interm_j3, interm_j4, 
											interm_j5, interm_j6, interm_j7]
		for j in range(1,8):
			result = p.resetJointState(kukaID,j,intermNode[j-1])
		p.stepSimulation()
		#time.sleep(0.05)
		isCollision = collisionCheck_staticG(kukaID, static_geometries)
		if isCollision == True:
			isEdgeValid = False
			break


	return isEdgeValid

def executeTrajectory(traj_file, kukaID):
	traj = []
	f = open(traj_file)
	time.sleep(10)
	previous_state = None

	for line in f:
		current_state = line.split()
		current_state = map(float, current_state)
		if (previous_state is not None):
			local_move(previous_state, current_state, kukaID)
		previous_state = current_state

def local_move(n1, n2, kukaID):
	step = 5 * math.pi / 180
	nseg = int(math.ceil(max(math.fabs(n1[0]-n2[0]), 
		math.fabs(n1[1]-n2[1]), math.fabs(n1[2]-n2[2]), 
		math.fabs(n1[3]-n2[3]), math.fabs(n1[4]-n2[4]), 
		math.fabs(n1[5]-n2[5]), math.fabs(n1[6]-n2[6]))) / step)
	for i in xrange(1,nseg+1):
		# interm_j1 = min(n1[0], n2[0]) + math.fabs(n1[0]-n2[0])/nseg * i
		# interm_j2 = min(n1[1], n2[1]) + math.fabs(n1[1]-n2[1])/nseg * i
		# interm_j3 = min(n1[2], n2[2]) + math.fabs(n1[2]-n2[2])/nseg * i
		# interm_j4 = min(n1[3], n2[3]) + math.fabs(n1[3]-n2[3])/nseg * i
		# interm_j5 = min(n1[4], n2[4]) + math.fabs(n1[4]-n2[4])/nseg * i
		# interm_j6 = min(n1[5], n2[5]) + math.fabs(n1[5]-n2[5])/nseg * i
		# interm_j7 = min(n1[6], n2[6]) + math.fabs(n1[6]-n2[6])/nseg * i

		interm_j1 = n1[0] + (n2[0]-n1[0]) / nseg * i
		interm_j2 = n1[1] + (n2[1]-n1[1]) / nseg * i
		interm_j3 = n1[2] + (n2[2]-n1[2]) / nseg * i
		interm_j4 = n1[3] + (n2[3]-n1[3]) / nseg * i
		interm_j5 = n1[4] + (n2[4]-n1[4]) / nseg * i
		interm_j6 = n1[5] + (n2[5]-n1[5]) / nseg * i
		interm_j7 = n1[6] + (n2[6]-n1[6]) / nseg * i
		intermNode = [interm_j1, interm_j2, interm_j3, interm_j4, 
											interm_j5, interm_j6, interm_j7]
		if i == 1:
			print "begin state +1: " + str(intermNode) 
		if i == nseg-1:
			print "end state -1: " + str(intermNode)
		if (i == nseg):
			print "end state: " + str(intermNode)
		for j in range(1,8):
			result = p.resetJointState(kukaID,j,intermNode[j-1])
		p.stepSimulation()
		time.sleep(0.05)

	# print "------------------------------------------\n"


		














































# Old functionalities kept for legacy
########################################################################
def belief_space_estimator(v, nHypotheses):
	belief_space = []
	temp_counter = 0
	while (temp_counter < nHypotheses):
		x = float(np.random.normal(20, v, 1))
		if (x <= 0.0):
			continue
		belief_space.append(x)
		temp_counter += 1

	# normalize and sort
	temp_sum = sum(belief_space)
	for i in range(0, nHypotheses):
		belief_space[i] /= temp_sum
	belief_space.sort()
	for i in range(0, nHypotheses):
		belief_space[i] = round(belief_space[i], 2)

	return belief_space
def createMesh1(meshfile, scale, pos, angles):
	_c = p.createCollisionShape(shapeType=p.GEOM_MESH, 
			fileName=meshfile, meshScale=[scale, scale, scale])
	ori = euler_to_quaternion(angles[2], angles[1], angles[0])
	_v = p.createVisualShape(shapeType=p.GEOM_MESH,
		fileName=meshfile, meshScale=[scale, scale, scale])

	_m = p.createMultiBody(baseCollisionShapeIndex=_c,baseVisualShapeIndex=_v,
							basePosition=pos,baseOrientation=ori)





