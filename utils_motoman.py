from __future__ import division 
import pybullet as p
import numpy as np
import pybullet_data
import IPython
import math
import time
import random

from scipy import spatial
import cPickle as pickle

class Mesh:
	def __init__(self, m, objName, objIdx, hypoIdx, pos, quat, prob, objRole):
		self.m = m
		self.objName = objName
		self.objIdx = objIdx
		self.hypoIdx = hypoIdx
		self.pos = pos
		self.quat = quat
		self.prob = prob
		self.objRole = objRole

	def setProb(self, prob):
		self.prob = prob

'''
## yaw(Z), pitch (Y), roll(X) 
## This function come from online wiki link
## https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
def euler_to_quaternion(yaw, pitch, roll):
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

    return [qx, qy, qz, qw]
## Pybullet provides a built-in function getQuaternionFromEuler() which
## follows a new rule [Y, Xnew, Znew] and return the quaternion in format [x,y,z,w]
'''

def comp_prob(pos, angles, temp_pos, temp_angles):
	## parameter tunning for mimicing sensors
	alpha = 15.0
	beta = 15.0
	gamma = 0.003
	temp_prob = math.exp(-alpha*math.fabs(pos[0]-temp_pos[0]) \
				- beta*math.fabs(pos[1]-temp_pos[1]) \
				- gamma*math.fabs(angles[2]-temp_angles[2]))
	return float(format(temp_prob, '.3f'))

def createTrueMesh(hypoIdx, objIdx, meshFile, objName, objRole, scale, pos, angles, mass, clientId):
	### starting hypo index for the current obs
	### with the mesh file and scale
	### create the collision and visual shape of the object
	objMesh = []

	if objRole == "phantom":
		return objMesh
	_c = p.createCollisionShape(shapeType=p.GEOM_MESH, 
								fileName=meshFile, meshScale=[scale, scale, scale], physicsClientId=clientId)
	_v = p.createVisualShape(shapeType=p.GEOM_MESH, 
								fileName=meshFile, meshScale=[scale, scale, scale], physicsClientId=clientId)
	temp_quat = p.getQuaternionFromEuler([angles[0], angles[1], angles[2]])
	quat = []
	for tq in temp_quat:
		quat.append(float(format(tq, '.3f')))
	_m = p.createMultiBody(baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v, 
											basePosition=pos, baseOrientation=quat, physicsClientId=clientId)
	objMesh.append(Mesh(_m, objName, objIdx, hypoIdx, pos, quat, 1.0, objRole))

	return objMesh

def createHypoMesh(currentlabelIdx, objIdx, meshFile, objName, objRole, scale, pos, angles, 
								transErrors, orientErrors, known_geometries, benchmarkType, nHypos, clientId):
	### starting label index for the current obs (currentlabelIdx)
	### with the mesh file and scale
	### create the collision and visual shape of the object
	hypotheses = []

	### first specify the collision and visual geometry
	_c = p.createCollisionShape(shapeType=p.GEOM_MESH, 
			fileName=meshFile, meshScale=[scale, scale, scale], physicsClientId=clientId)
	# _v = p.createVisualShape(shapeType=p.GEOM_MESH,
	# 		fileName=meshFile, meshScale=[scale, scale, scale], physicsClientId=clientId)

	largest_prob = 0.0
	largest_prob_idx = currentlabelIdx
	### for each hypothesis
	h = 0

	while (h < nHypos):
		### generate new pos and new quat according to transErrors and orientErrors
		temp_px = float(format(random.uniform(pos[0]-transErrors, pos[0]+transErrors), '.3f'))
		temp_py = float(format(random.uniform(pos[1]-transErrors, pos[1]+transErrors), '.3f'))
		temp_oz = float(format(random.uniform(angles[2]-orientErrors, angles[2]+orientErrors), '.3f'))
		temp_pos = [temp_px, temp_py, pos[2]] ### so far no uncertainty in height (z axis)
		temp_angles = [angles[0], angles[1], temp_oz] ### so far only uncertainty in the orientation around z axis
		temp_quat = p.getQuaternionFromEuler(temp_angles)
		if objRole != "phantom":
			temp_prob = comp_prob(pos, angles, temp_pos, temp_angles)
		else:
			temp_prob = float(format(random.uniform(0.15, 0.35), '.3f'))
		### create the hypothesis mesh and save it into the hypotheses
		_v = p.createVisualShape(shapeType=p.GEOM_MESH,
			fileName=meshFile, meshScale=[scale, scale, scale], rgbaColor=[1.0, 1.0, 1.0, temp_prob], physicsClientId=clientId)
		_m = p.createMultiBody(baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v, 
										basePosition=temp_pos, baseOrientation=temp_quat, physicsClientId=clientId)
		### Before adding the mesh into the hypotheses, check if it collides with known geometries
		temp_collided = collisionCheck_amongObjects(_m, known_geometries, benchmarkType, clientId)
		if (temp_collided):
			p.removeBody(_m, clientId)
			continue
		# print objName + ":" + str(_m)
		### reached here since it has no collision with known obstacles
		### temporarily assign a probability
		### assign a probability for phantom object between 15%-35%
		# if objRole != "phantom":
		# 	temp_prob = comp_prob(pos, angles, temp_pos, temp_angles)
		# else:
		# 	temp_prob = float(format(random.uniform(0.15, 0.35), '.3f'))
		
		if temp_prob > largest_prob:
			largest_prob = temp_prob
			# largest_prob_idx = currentlabelIdx+h
			largest_prob_idx = int(_m)
		### add this hypothesis
		# hypotheses.append( Mesh(_m, objName, objIdx, currentlabelIdx+h, temp_pos, temp_quat, temp_prob, objRole) )
		hypotheses.append( Mesh(_m, objName, objIdx, int(_m), temp_pos, temp_quat, temp_prob, objRole) )
		h = h + 1

	### last step: normalize the probability
	if objRole == "phantom":
		return hypotheses, largest_prob_idx
	### Otherwise
	temp_probs = []
	for hp in hypotheses:
		temp_probs.append(hp.prob)
	temp_sum = sum(temp_probs)
	### specify the probability that the object is not in the scene (random number between 10%-20%)
	prob_notInScene = float(format(random.uniform(0.1, 0.2), '.3f'))
	for i in xrange(0, len(hypotheses)):
		hypotheses[i].setProb(round(temp_probs[i]*(1-prob_notInScene)/temp_sum, 3))

	return hypotheses, largest_prob_idx


def printPoses(meshes):
	for mesh in meshes:
		print "hypo " + str(mesh.hypoIdx) + " " + str(mesh.pos) + " " + str(mesh.quat) + " " + \
							str(mesh.prob) + "\tfor object " + str(mesh.objIdx) + ": " + mesh.objName
	print "--------------------------------------\n"

def collisionCheck_amongObjects(m, known_geometries, benchmarkType, clientId):
	isCollision = False
	for g in known_geometries:
		if benchmarkType == "table" and str(g) == "1":
			continue
		if benchmarkType == "shelf" and str(g) == "1":
			continue
		if benchmarkType == "shelf" and str(g) == "5":
			continue
		contacts = p.getClosestPoints(m, g, 0.01, physicsClientId=clientId)
		if len(contacts) != 0:
			isCollision = True
			print "collision between " + m.objName + "and " + "known geometries " + str(g)
			break

	return isCollision

def collisionCheck_selfCollision(motomanID, clientId):
	isCollision = False
	contacts = p.getContactPoints(bodyA=motomanID, bodyB=motomanID, physicsClientId=clientId)
	if len(contacts) != 0:
		isCollision = True
		# print "robot self collision occurs!"
		# print contacts
	return isCollision

def collisionCheck_knownObs(motomanID, known_geometries, clientId):
	isCollision = False
	### loop through all known obstacles
	for g in known_geometries:
		if g == 0: ## no need to change robot self-collision again
			continue
		contacts = p.getContactPoints(motomanID, g, physicsClientId=clientId)
		if len(contacts) != 0:
			isCollision = True
			# print "collision with known obstacle " + str(g)
			break
	return isCollision

def collisionCheck_hypos(motomanID, hypotheses, clientId):
	collidedHypos = []
	### loop through all hypotheses
	for hp in hypotheses:
		contacts = p.getContactPoints(motomanID, hp.m, physicsClientId=clientId)
		if len(contacts) != 0:
			### add the index of that hypo
			collidedHypos.append(hp.hypoIdx)

	return collidedHypos


def checkEdgeValidity(n1, n2, motomanID, known_geometries, clientId):
	step = 8 * math.pi / 180
	nseg = int(math.ceil(max(math.fabs(n1[0]-n2[0]), math.fabs(n1[1]-n2[1]), 
			math.fabs(n1[2]-n2[2]), math.fabs(n1[3]-n2[3]), math.fabs(n1[4]-n2[4]), 
			math.fabs(n1[5]-n2[5]), math.fabs(n1[6]-n2[6]))) / step)
	if nseg == 0:
		nseg = 1
	isEdgeValid = True
	for i in xrange(1, nseg):
		interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
		interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
		interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
		interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
		interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
		interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
		interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
		intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6]
		for j in range(1, 8):
			result = p.resetJointState(motomanID, j, intermNode[j-1], physicsClientId=clientId)
		for j in range(11, 18):
			result = p.resetJointState(motomanID, j, 0.0, physicsClientId=clientId)
		p.stepSimulation(clientId)
		isCollisionSelf = collisionCheck_selfCollision(motomanID, clientId)
		isCollisionKnownObs = collisionCheck_knownObs(motomanID, known_geometries, clientId)

		if isCollisionSelf or isCollisionKnownObs:
			isEdgeValid = False
			break

	return isEdgeValid

def label_the_edge(n1, n2, motomanID, hypotheses, clientId):
	temp_labels = []
	labels_status = [False] * (hypotheses[-1].hypoIdx + 1)
	step = 8 * math.pi / 180
	nseg = int(math.ceil(max(math.fabs(n1[0]-n2[0]), math.fabs(n1[1]-n2[1]), 
			math.fabs(n1[2]-n2[2]), math.fabs(n1[3]-n2[3]), math.fabs(n1[4]-n2[4]), 
			math.fabs(n1[5]-n2[5]), math.fabs(n1[6]-n2[6]))) / step)
	if nseg == 0:
		nseg = 1
	for i in xrange(0, nseg+1):
		interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
		interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
		interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
		interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
		interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
		interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
		interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
		intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6]
		for j in range(1, 8):
			result = p.resetJointState(motomanID, j, intermNode[j-1], physicsClientId=clientId)
		for j in range(11, 18):
			result = p.resetJointState(motomanID, j, 0.0, physicsClientId=clientId)
		p.stepSimulation(clientId)
		for hp in hypotheses:
			### Before you do collision checker
			### check if that hypo has been checked before
			if labels_status[hp.hypoIdx] == False:
				contacts = p.getContactPoints(motomanID, hp.m, physicsClientId=clientId)
				if len(contacts) != 0:
					temp_labels.append(hp.hypoIdx)
					labels_status[hp.hypoIdx] = True

	return temp_labels

def collisionCheck_truePoses(motomanID, truePoses, clientId):
	truePosesIdx = []
	### loop through truePoses
	for tp in truePoses:
		contacts = p.getContactPoints(motomanID, tp.m, physicsClientId=clientId)
		if len(contacts) != 0:
			### add the index of that true pose
			truePosesIdx.append(tp.objIdx)
	return set(truePosesIdx)

def trueScene_generation(benchmarkType, scene, Objects, clientId):
	truePoses = []
	nObjectInExecuting = 0
	for i in xrange(len(Objects)):
		if Objects[i][3] != "phantom":
			truePoses += createTrueMesh(-1, Objects[i][0], Objects[i][1], Objects[i][2], Objects[i][3], 
												Objects[i][4], Objects[i][5], Objects[i][6], Objects[i][7], clientId)
			nObjectInExecuting += 1
	print "Number of Objects in the ground truth (execution): " + str(nObjectInExecuting)
	print "-------true poses: " + str(benchmarkType) + ", " + str(scene) + "-------"
	printPoses(truePoses)
	return truePoses, nObjectInExecuting

def planScene_generation(Objects, benchmarkType, known_geometries, nHypos, transErrors, orientErrors, clientId):
	hypotheses = []
	mostPromisingHypoIdxes = []
	currentlabelIdx = int(known_geometries[-1]) + 1
	nObjectInPlanning = 0
	for i in xrange(len(Objects)):
		if Objects[i][3] == "invisible":
			continue
		mm, pp = createHypoMesh(currentlabelIdx, Objects[i][0], Objects[i][1], Objects[i][2], Objects[i][3], Objects[i][4], 
			Objects[i][5], Objects[i][6], transErrors, orientErrors, known_geometries, benchmarkType, nHypos, clientId)
		hypotheses += mm
		mostPromisingHypoIdxes.append(pp)
		nObjectInPlanning += 1
		currentlabelIdx = int(hypotheses[-1].hypoIdx) + 1
	print "Number of Objects in the planning scene: " + str(nObjectInPlanning)
	print "-------all hypotheses: " + str(benchmarkType) + "-------"
	printPoses(hypotheses)
	return hypotheses, mostPromisingHypoIdxes, nObjectInPlanning

def calculateEuclidean(previous_state, current_state):
	tempSquare = 0.0
	for ii in xrange(len(previous_state)):
		tempSquare += math.pow(previous_state[ii]-current_state[ii] ,2)
	tempSquare = math.sqrt(tempSquare)
	return tempSquare

def local_move(n1, n2, motomanID, truePoses, clientId):
	local_poseIdx = set()
	# step = 5 * math.pi / 180
	step = 3.0 * math.pi / 180
	nseg = int(math.ceil(max(math.fabs(n1[0]-n2[0]), 
		math.fabs(n1[1]-n2[1]), math.fabs(n1[2]-n2[2]), 
		math.fabs(n1[3]-n2[3]), math.fabs(n1[4]-n2[4]), 
		math.fabs(n1[5]-n2[5]), math.fabs(n1[6]-n2[6]))) / step)
	if nseg == 0:
		nseg = 1
	for i in xrange(1, nseg):
		interm_j0 = n1[0] + (n2[0]-n1[0]) / nseg * i
		interm_j1 = n1[1] + (n2[1]-n1[1]) / nseg * i
		interm_j2 = n1[2] + (n2[2]-n1[2]) / nseg * i
		interm_j3 = n1[3] + (n2[3]-n1[3]) / nseg * i
		interm_j4 = n1[4] + (n2[4]-n1[4]) / nseg * i
		interm_j5 = n1[5] + (n2[5]-n1[5]) / nseg * i
		interm_j6 = n1[6] + (n2[6]-n1[6]) / nseg * i
		intermNode = [interm_j0, interm_j1, interm_j2, interm_j3, interm_j4, interm_j5, interm_j6]
		for j in range(1, 8):
			result = p.resetJointState(motomanID, j, intermNode[j-1], physicsClientId=clientId)
		for j in range(11, 18):
			result = p.resetJointState(motomanID, j, 0.0, physicsClientId=clientId)
		p.stepSimulation(clientId)
		## add collision checker
		temp_poseIdx = collisionCheck_truePoses(motomanID, truePoses, clientId)
		local_poseIdx = local_poseIdx.union(temp_poseIdx)

		time.sleep(0.05)
	return local_poseIdx

def executeTrajectory(traj_file, motomanID, truePoses, clientId):
	traj = []
	f = open(traj_file)
	previous_state = None
	### Meanwhile compute the cost as well
	trajectoryCost = 0.0
	trajectoryCollision = set()
	isSuccess = 1

	for line in f:
		current_state = line.split()
		current_state = map(float, current_state)
		if (previous_state is not None):
			trajectoryCost += calculateEuclidean(previous_state, current_state)
			trajectoryCollision = trajectoryCollision.union(
											local_move(previous_state, current_state, motomanID, truePoses, clientId))
			time.sleep(0.04)
		previous_state = current_state
	print "collisions: " + str(trajectoryCollision) + ",  total: " + str(len(trajectoryCollision))

	### now work on success evaluation
	if len(trajectoryCollision) != 0:
		isSuccess = 0

	return len(trajectoryCollision), isSuccess, trajectoryCost	

def executeAllTraj_example(home_configuration, motomanID, truePoses, path, clientId):
	statistics_file = path + "/statistics.txt"
	### write in (#obs, success, cost) information for each search method
	f = open(statistics_file, "w")

	raw_input("Press to put Motoman to home configuration")
	### Put the motoman back to its home configuration
	for j in range(1, 8):
		result = p.resetJointState(motomanID, j, home_configuration[j-1], physicsClientId=clientId)
	for j in range(11, 18):
		result = p.resetJointState(motomanID, j, home_configuration[j-4], physicsClientId=clientId)
	raw_input("Press to execute A star trajectory")
	astar_traj_file = path + "/Astartraj.txt"
	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(astar_traj_file, motomanID, truePoses, clientId)
	f.write( str(temp_ncollision) + " " + str(temp_isSuccess) + " " + str(temp_cost) + "\n" )

	raw_input("Press to put Motoman to home configuration")
	### Put the motoman back to its home configuration
	for j in range(1, 8):
		result = p.resetJointState(motomanID, j, home_configuration[j-1], physicsClientId=clientId)
	for j in range(11, 18):
		result = p.resetJointState(motomanID, j, home_configuration[j-4], physicsClientId=clientId)
	raw_input("Press to execute MCR Greedy trajectory")
	mcrg_traj_file = path + "/MCRGtraj.txt"
	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(mcrg_traj_file, motomanID, truePoses, clientId)
	f.write( str(temp_ncollision) + " " + str(temp_isSuccess) + " " + str(temp_cost) + "\n" )

	raw_input("Press to put Motoman to home configuration")
	### Put the motoman back to its home configuration
	for j in range(1, 8):
		result = p.resetJointState(motomanID, j, home_configuration[j-1], physicsClientId=clientId)
	for j in range(11, 18):
		result = p.resetJointState(motomanID, j, home_configuration[j-4], physicsClientId=clientId)
	raw_input("Press to execute MCR exact trajectory")
	mcre_traj_file = path + "/MCREtraj.txt"
	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(mcre_traj_file, motomanID, truePoses, clientId)
	f.write( str(temp_ncollision) + " " + str(temp_isSuccess) + " " + str(temp_cost) + "\n" )

	raw_input("Press to put Motoman to home configuration")
	### Put the motoman back to its home configuration
	for j in range(1, 8):
		result = p.resetJointState(motomanID, j, home_configuration[j-1], physicsClientId=clientId)
	for j in range(11, 18):
		result = p.resetJointState(motomanID, j, home_configuration[j-4], physicsClientId=clientId)
	raw_input("Press to execute MaxSuccess greedy trajectory")
	msg_traj_file = path + "/MSGtraj.txt"
	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(msg_traj_file, motomanID, truePoses, clientId)
	f.write( str(temp_ncollision) + " " + str(temp_isSuccess) + " " + str(temp_cost) + "\n" )

	raw_input("Press to put Motoman to home configuration")
	### Put the motoman back to its home configuration
	for j in range(1, 8):
		result = p.resetJointState(motomanID, j, home_configuration[j-1], physicsClientId=clientId)
	for j in range(11, 18):
		result = p.resetJointState(motomanID, j, home_configuration[j-4], physicsClientId=clientId)
	raw_input("Press to execute MaxSuccess exact trajectory")
	mse_traj_file = path + "/MSEtraj.txt"
	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(mse_traj_file, motomanID, truePoses, clientId)
	f.write( str(temp_ncollision) + " " + str(temp_isSuccess) + " " + str(temp_cost) + "\n" )	

	raw_input("Press to put Motoman to home configuration")
	### Put the motoman back to its home configuration
	for j in range(1, 8):
		result = p.resetJointState(motomanID, j, home_configuration[j-1], physicsClientId=clientId)
	for j in range(11, 18):
		result = p.resetJointState(motomanID, j, home_configuration[j-4], physicsClientId=clientId)
	raw_input("Press to execute MCR-MLC trajectory")
	mcrmcg_traj_file = path + "/MCRMCGtraj.txt"
	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(mcrmcg_traj_file, motomanID, truePoses, clientId)
	f.write( str(temp_ncollision) + " " + str(temp_isSuccess) + " " + str(temp_cost) + "\n" )

	f.close()
	print "trajectories all executed and time record."