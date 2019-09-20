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
	def __init__(self, m, meshType, objIndx, hypoIndx, pos, quat, prob, objectRole):
		self.m = m
		self.meshType = meshType
		self.objIndx = objIndx
		self.hypoIndx = hypoIndx
		self.pos = pos
		self.quat = quat
		self.prob = prob
		self.objectRole = objectRole

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
def createHypoMesh(f, labelIdx, objIndx, meshfile, meshType, objectRole, scale, 
						pos, angles, uncertainty, clientId, static_geometries, benchmarkType, nhypo):
	# starting label index for the current obs
	# with the mesh file and scale 
	# create the collision and visual shape of the object
	meshes = []

	if objectRole == "invisible":
		return meshes

	_c = p.createCollisionShape(shapeType=p.GEOM_MESH, 
			fileName=meshfile, meshScale=[scale, scale, scale], physicsClientId=clientId)
	_v = p.createVisualShape(shapeType=p.GEOM_MESH,
		fileName=meshfile, meshScale=[scale, scale, scale], physicsClientId=clientId)

	# quat = euler_to_quaternion(angles[2], angles[1], angles[0])
	quat = p.getQuaternionFromEuler([angles[0], angles[1], angles[2]])

	## first put the true pose of the objects into the meshes
	_m = p.createMultiBody(baseCollisionShapeIndex=_c,baseVisualShapeIndex=_v,
						basePosition=pos,baseOrientation=quat, physicsClientId=clientId)
	meshes.append(Mesh(_m, meshType, objIndx, labelIdx, pos, quat, 1, objectRole))

	largest_prob = 1.0
	largest_prob_idx = labelIdx
	## for each hypothesis other than the true pose
	h = 1
	failureTime = 0
	while (h < nhypo):
		## generate new pos and new quat according to uncertainty
		temp_px = round(float(np.random.normal(pos[0], uncertainty[0])), 4)
		temp_py = round(float(np.random.normal(pos[1], uncertainty[1])), 4)
		temp_pos = [temp_px, temp_py, pos[2]]
		temp_quat = quat
		temp_zangle = angles[2]

		if len(uncertainty) == 3:
			temp_zangle = float(np.random.normal(angles[2], uncertainty[2]))
			# temp_quat = euler_to_quaternion(temp_zangle, angles[1], angles[0])
			temp_quat = p.getQuaternionFromEuler([angles[0], angles[1], temp_zangle])

		## compute the unnormalized probability of this hypothesis
		temp_prob = comp_prob(pos, temp_pos, angles, temp_zangle)
		if temp_prob > largest_prob:
			largest_prob = temp_prob
			largest_prob_idx = labelIdx+h
		## create the mesh and save it into meshes
		_m = p.createMultiBody(baseCollisionShapeIndex=_c,baseVisualShapeIndex=_v,
						basePosition=temp_pos,baseOrientation=temp_quat, physicsClientId=clientId)
		# Before adding the mesh obj into the meshes, check if it collides with static geometries
		temp_collided = collisionCheck_specificG(_m, static_geometries, benchmarkType, clientId)
		if (temp_collided):
			p.removeBody(_m, clientId)
			continue
		meshes.append(Mesh(_m, meshType, objIndx, labelIdx+h, temp_pos, temp_quat, temp_prob, objectRole))
		h = h + 1


	## last step: normalize the probability
	temp_probs = []
	for mesh in meshes:
		temp_probs.append(mesh.prob)
	temp_sum = sum(temp_probs)
	for i in xrange(0, len(meshes)):
		meshes[i].setProb(round(temp_probs[i]*0.96/temp_sum, 3))
		f.write( str(labelIdx) + " " + str(meshes[i].objIndx) + " " + str(meshes[i].prob) + "\n" )
		labelIdx += 1

	return meshes, largest_prob_idx

def createTrueMesh(labelIdx, objIndx, meshfile, meshType, objectRole, scale, pos, angles, clientId):
	# starting label index for the current obs
	# with the mesh file and scale 
	# create the collision and visual shape of the object
	meshes = []

	if objectRole == "phantom":
		return meshes
	_c = p.createCollisionShape(shapeType=p.GEOM_MESH, 
			fileName=meshfile, meshScale=[scale, scale, scale], physicsClientId=clientId)
	_v = p.createVisualShape(shapeType=p.GEOM_MESH,
		fileName=meshfile, meshScale=[scale, scale, scale], physicsClientId=clientId)

	# quat = euler_to_quaternion(angles[2], angles[1], angles[0])
	quat = p.getQuaternionFromEuler([angles[0], angles[1], angles[2]])

	## first put the true pose of the objects into the meshes
	if meshType != "baseball":
		_m = p.createMultiBody(baseMass=3.5, baseCollisionShapeIndex=_c,baseVisualShapeIndex=_v,
				basePosition=pos,baseOrientation=quat, physicsClientId=clientId)
	else:
		_m = p.createMultiBody(baseCollisionShapeIndex=_c,baseVisualShapeIndex=_v,
				basePosition=pos,baseOrientation=quat, physicsClientId=clientId)		
	meshes.append(Mesh(_m, meshType, objIndx, labelIdx, pos, quat, 1, objectRole))

	return meshes

def printPoses(meshes):
	for mesh in meshes:
		print "Label " + str(mesh.hypoIndx) + " " + str(mesh.pos) + \
							" " + str(mesh.quat) + " " + str(mesh.prob) + \
							"\tfor object " + str(mesh.objIndx) + ": " + mesh.meshType
	print "----------------------------\n"

def collisionCheck_specificG(m, static_geometries, mode, clientId):
	isCollision = False;
	for g in static_geometries:
		if mode == "table" and str(g) == "1":
			continue
		if mode == "shelf" and str(g) == "2":
			continue
		if mode == "shelf" and str(g) == "6":
			continue
		contacts = p.getClosestPoints(m, g, 0.01, physicsClientId=clientId)
		if len(contacts) != 0:
			isCollision = True
			# print "static collision with " + str(g)
			break

	return isCollision



def collisionCheck_staticG(kukaID, static_geometries, clientId):
	isCollision = False
	## loop through all static geometries
	for g in static_geometries:
		if g != 0:
			contacts = p.getContactPoints(kukaID, g, physicsClientId=clientId)
			if len(contacts) != 0:
				isCollision = True
				# if counter == 1:
				# 	print "Self Collision with Kuka!!"
				# print "static collision with " + str(g)
				break
		# print "static collision? " + "\t" + str(isCollision)
	return isCollision


def truePosesCollision(kukaID, truePoses, clientId):
	truePoseIdx = []
	## loop through truePoses
	for m in truePoses:
		contacts = p.getContactPoints(kukaID, m.m, physicsClientId=clientId)
		if len(contacts) != 0:
			## add the index of that true pose
			truePoseIdx.append(m.objIndx)
	return set(truePoseIdx)	


def collision_with_hypos(kukaID, meshSet, clientId):
	collidedHypos = []
	## loop through meshes of each hypothesis
	for m in meshSet:
		if m.objectRole != "invisible":
			contacts = p.getContactPoints(kukaID, m.m, physicsClientId=clientId)
			if len(contacts) != 0:
				## add the index of that hypo
				collidedHypos.append(m.hypoIndx)

	return collidedHypos


def checkEdgeValidity(n1, n2, kukaID, static_geometries, clientId):
	step = 8 * math.pi / 180
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
			result = p.resetJointState(kukaID,j,intermNode[j-1], physicsClientId=clientId)
		p.stepSimulation(clientId)

		isCollision = collisionCheck_staticG(kukaID, static_geometries, clientId)

		if isCollision == True:
			isEdgeValid = False
			break 

	return isEdgeValid

def label_the_edge(n1, n2, kukaID, meshSet, clientId):
	temp_labels = []
	labels_status = [False] * len(meshSet)
	step = 8 * math.pi / 180
	nseg = int(math.ceil(max(math.fabs(n1[0]-n2[0]), math.fabs(n1[1]-n2[1]), 
			math.fabs(n1[2]-n2[2]), math.fabs(n1[3]-n2[3]), math.fabs(n1[4]-n2[4]), 
			math.fabs(n1[5]-n2[5]), math.fabs(n1[6]-n2[6]))) / step)
	for i in xrange(0, nseg+1):
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
			result = p.resetJointState(kukaID,j,intermNode[j-1], physicsClientId=clientId)
		p.stepSimulation(clientId)
		for m in meshSet:
			if m.objectRole != "invisible":
				## Before you do collision checker
				## check if that hypo has been checked before
				if labels_status[m.hypoIndx] == False:
					contacts = p.getContactPoints(kukaID, m.m, physicsClientId=clientId)
					if len(contacts) != 0:
						# print "Collision with " + str(m.meshType) + " on hypo " + str(m.hypoIndx)
						temp_labels.append(m.hypoIndx)
						labels_status[m.hypoIndx] = True

	return temp_labels

def deletePlanScene(meshSet, clientId):
	for meshobj in meshSet:
		p.removeBody(meshobj.m, clientId)

def deleteTruePoses(truePoses, clientId):
	for trueobj in truePoses:
		p.removeBody(trueobj.m, clientId)

def deleteStaticGeometries(static_geometries, clientId):
	for g in static_geometries:
		p.removeBody(g, clientId)

def executeAllTraj_example(home_configuration, kukaID_e, truePoses, folderFile, clientId):
	statistics_file = folderFile + "/statistics.txt"
	## write in (#obs, success, cost) information for each search method
	f = open(statistics_file, "w")

	raw_input("Press to put kuka to home configuration")
	## Put the kuka back to its home configuration
	for i in range(1,8):
		result = p.resetJointState(kukaID_e, i, home_configuration[i-1], physicsClientId=clientId)
	raw_input("Press to execute A star trajectory")
	astar_traj_file = folderFile + "/Astartraj.txt"
	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(astar_traj_file, kukaID_e, truePoses, clientId)
	f.write( str(temp_ncollision) + " " + str(temp_isSuccess) + " " + str(temp_cost) + "\n" )

	raw_input("Press to put kuka to home configuration")
	## Put the kuka back to its home configuration
	for i in range(1,8):
		result = p.resetJointState(kukaID_e, i, home_configuration[i-1], physicsClientId=clientId)
	raw_input("Press to execute MCR Greedy trajectory")
	mcrg_traj_file = folderFile + "/MCRGtraj.txt"
	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(mcrg_traj_file, kukaID_e, truePoses, clientId)
	f.write( str(temp_ncollision) + " " + str(temp_isSuccess) + " " + str(temp_cost) + "\n" )

	raw_input("Press to put kuka to home configuration")
	## Put the kuka back to its home configuration
	for i in range(1,8):
		result = p.resetJointState(kukaID_e, i, home_configuration[i-1], physicsClientId=clientId)
	raw_input("Press to execute MCR exact trajectory")
	mcre_traj_file = folderFile + "/MCREtraj.txt"
	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(mcre_traj_file, kukaID_e, truePoses, clientId)
	f.write( str(temp_ncollision) + " " + str(temp_isSuccess) + " " + str(temp_cost) + "\n" )

	raw_input("Press to put kuka to home configuration")
	## Put the kuka back to its home configuration
	for i in range(1,8):
		result = p.resetJointState(kukaID_e, i, home_configuration[i-1], physicsClientId=clientId)
	raw_input("Press to execute MaxSuccess greedy trajectory")
	msg_traj_file = folderFile + "/MSGtraj.txt"
	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(msg_traj_file, kukaID_e, truePoses, clientId)
	f.write( str(temp_ncollision) + " " + str(temp_isSuccess) + " " + str(temp_cost) + "\n" )

	raw_input("Press to put kuka to home configuration")
	## Put the kuka back to its home configuration
	for i in range(1,8):
		result = p.resetJointState(kukaID_e, i, home_configuration[i-1], physicsClientId=clientId)
	raw_input("Press to execute MaxSuccess exact trajectory")
	mse_traj_file = folderFile + "/MSEtraj.txt"
	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(mse_traj_file, kukaID_e, truePoses, clientId)
	f.write( str(temp_ncollision) + " " + str(temp_isSuccess) + " " + str(temp_cost) + "\n" )

	raw_input("Press to put kuka to home configuration")
	## Put the kuka back to its home configuration
	for i in range(1,8):
		result = p.resetJointState(kukaID_e, i, home_configuration[i-1], physicsClientId=clientId)
	raw_input("Press to execute MCR-MLC trajectory")
	mcrmcg_traj_file = folderFile + "/MCRMCGtraj.txt"
	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(mcrmcg_traj_file, kukaID_e, truePoses, clientId)
	f.write( str(temp_ncollision) + " " + str(temp_isSuccess) + " " + str(temp_cost) + "\n" )		

	f.close()

	print "trajectories all executed and time record."

def executeTraj(kukaID_e, keyword, truePoses, folderFile, cc, nn, clientId):

	# statistics_file = folderFile + "/statistics_" + str(cc) + "_" + str(nn) + ".txt"
	## write in (#obs, success, cost) information for each search method
	# f = open(statistics_file, "w")

	raw_input("Press to execute" + keyword + "trajectory")
	traj_file = folderFile + keyword + str(cc) + "_" + str(nn) + ".txt"
	temp_ncollision, temp_isSuccess, temp_cost = executeTrajectory(traj_file, kukaID_e, truePoses, clientId)
	# f.write( str(temp_ncollision) + " " + str(temp_isSuccess) + " " + str(temp_cost) + "\n" )


	# f.close()



def executeTrajectory(traj_file, kukaID, truePoses, clientId):
	traj = []
	f = open(traj_file)
	previous_state = None

	## Meanwhile compute the cost as well
	trajectoryCost = 0.0
	trajectoryCollision = set()
	isSuccess = 1

	for line in f:
		current_state = line.split()
		current_state = map(float, current_state)
		if (previous_state is not None):
			trajectoryCost += calculateEuclidean(previous_state, current_state)
			trajectoryCollision = trajectoryCollision.union(local_move(previous_state, current_state, kukaID, truePoses, clientId))
			time.sleep(0.04)
		previous_state = current_state

	print "collisions: " + str(trajectoryCollision) + ",  total: " + str(len(trajectoryCollision))

	## now work on success evaluation
	if len(trajectoryCollision) != 0:
		isSuccess = 0

	return len(trajectoryCollision), isSuccess, trajectoryCost

def local_move(n1, n2, kukaID, truePoses, clientId):
	local_poseIdx = set()
	# step = 5 * math.pi / 180
	step = 3.0 * math.pi / 180
	nseg = int(math.ceil(max(math.fabs(n1[0]-n2[0]), 
		math.fabs(n1[1]-n2[1]), math.fabs(n1[2]-n2[2]), 
		math.fabs(n1[3]-n2[3]), math.fabs(n1[4]-n2[4]), 
		math.fabs(n1[5]-n2[5]), math.fabs(n1[6]-n2[6]))) / step)
	for i in xrange(1,nseg+1):
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
			result = p.resetJointState(kukaID,j,intermNode[j-1], physicsClientId=clientId)
		p.stepSimulation(clientId)
		## add collision checker
		temp_poseIdx = truePosesCollision(kukaID, truePoses, clientId)
		local_poseIdx = local_poseIdx.union(temp_poseIdx)

		time.sleep(0.05)
	return local_poseIdx

def calculateEuclidean(previous_state, current_state):
	tempSquare = 0.0
	for ii in xrange(len(previous_state)):
		tempSquare += math.pow(previous_state[ii]-current_state[ii] ,2)
	tempSquare = math.sqrt(tempSquare)
	return tempSquare

def trueScene_generation(benchmarkType, scene, Objects, clientId):
	truePoses = []
	nObjectInExecuting = 0
	for i in xrange(len(Objects)):
		if Objects[i][3] != "phantom":	
			truePoses += createTrueMesh(-1, Objects[i][0], Objects[i][1], 
					Objects[i][2], Objects[i][3], Objects[i][4], Objects[i][5], Objects[i][6], clientId)
			nObjectInExecuting += 1
	print "Number of Objects in the ground truth (execution): " + str(nObjectInExecuting)
	print "-------true poses: " + str(benchmarkType) + ", " + str(scene) + "-------"
	printPoses(truePoses)
	return truePoses, nObjectInExecuting

def planScene_generation(Objects, benchmarkType, static_geometries, 
														path, nhypo, noise, clientId):
	meshSet = []
	mostPromisingLabelIdx = []
	labelIdx = 0
	nObjectInPlanning = 0
	currentLabelWeightsFile = path + "/labelWeights.txt"
	f_label = open(currentLabelWeightsFile, "w")
	for i in xrange(len(Objects)):
		if Objects[i][3] != "invisible":
			mm, pp = createHypoMesh(f_label, labelIdx, Objects[i][0], Objects[i][1], Objects[i][2],
				Objects[i][3], Objects[i][4], Objects[i][5], Objects[i][6],
				[ Objects[i][7][0]*noise, Objects[i][7][1]*noise, 
					Objects[i][7][2]*noise ], clientId, static_geometries, benchmarkType, nhypo)
			meshSet += mm
			mostPromisingLabelIdx.append(pp)
			nObjectInPlanning += 1
		labelIdx = len(meshSet)
	print "Number of Objects in the planning scene: " + str(nObjectInPlanning)
	print "----------------all hypotheses----------------"
	printPoses(meshSet)
	# print "most promising labels: " + str(mostPromisingLabelIdx)

	currentMostPromisingLabelsFile = path + "/mostPromisingLabels.txt"
	f_mostPromisingLabels = open(currentMostPromisingLabelsFile, "w")
	for lll in mostPromisingLabelIdx:
		f_mostPromisingLabels.write(str(lll) + " ")
	f_mostPromisingLabels.close()	
	f_label.close()

	return meshSet, nObjectInPlanning

# def roadmap_generation(home_configuration, kukaID_p, kuka_ee_idx, ll, ul, jr,
# 											 static_geometries, meshSet, 
# 											 			goalPos_offset, goalEuler, x_ll, x_ul, y_ll, y_ul, z_ll, z_ul,
# 											 				nhypo, nObjectInPlanning, cc, nn,
# 																nsamples, folderFile, clientId):
# 	startTime = time.clock()
# 	############### specify q_start and set of q_goal first #################
# 	## q_start ##
# 	q_start = home_configuration

# 	## set home configuration
# 	for i in range(1,8):
# 		result = p.resetJointState(kukaID_p, i, home_configuration[i-1], physicsClientId=clientId)

# 	goalSet = []
# 	goalHypo = []
# 	goalSurvivalThreshold = 0.5
# 	MaxGoalsPerPose = 5
# 	MaxTrialsPerPose = 7
# 	MaxTrialsPerEE = 7

# 	## for each target hypothesis ##
# 	for hp in xrange(nhypo):
# 		# print "***********For Hypo " + str(hp) + "***************"
# 		temp_trials_pose = 0
# 		temp_ngoals = 0
# 		while temp_ngoals < MaxGoalsPerPose and temp_trials_pose < MaxTrialsPerPose:
# 			# print "-----A new ee pose-----"
# 			for i in range(1,8):
# 				result = p.resetJointState(kukaID_p, i, home_configuration[i-1], 
# 																		physicsClientId=clientId)
# 			## specify the position and quaternion of the goal pose based on the target hypothesis ##
# 			goal_pose_pos = []
# 			for i in xrange(len(goalPos_offset)):
# 				goal_pose_pos.append(meshSet[hp].pos[i] + goalPos_offset[i])
# 			temp_goal_pose_quat = p.getQuaternionFromEuler([goalEuler[0], goalEuler[1], 
# 														goalEuler[2]+random.uniform(-math.pi, math.pi)])
# 			goal_pose_quat = [temp_goal_pose_quat[0], temp_goal_pose_quat[1], 
# 														temp_goal_pose_quat[3], temp_goal_pose_quat[2]]
# 			temp_trials_ee = 0
# 			while temp_ngoals < MaxGoalsPerPose and temp_trials_ee < MaxTrialsPerEE:
# 				q_goal = p.calculateInverseKinematics(kukaID_p, kuka_ee_idx, goal_pose_pos, 
# 											goal_pose_quat, ll, ul, jr, physicsClientId=clientId)		
# 				for j in range(1,8):
# 					result = p.resetJointState(kukaID_p, j, q_goal[j-1], physicsClientId=clientId)
# 				p.stepSimulation(clientId)
# 				## check collision for static geometry
# 				isCollision1 = collisionCheck_staticG(kukaID_p, 
# 														static_geometries, clientId)
# 				if isCollision1:
# 					# print "static collision for the current IK..."
# 					# raw_input("Press Enter to Continue")
# 					temp_trials_ee += 2
# 					continue
# 				else:
# 					## check collision condition with all hypos of objects
# 					## (The opposite of the collision probability)
# 					collidedHypos = collision_with_hypos(kukaID_p, meshSet, clientId)
# 					# print "Collide with Hypos: " + str(collidedHypos)
# 					## compute the survivability
# 					if hp in collidedHypos:
# 						# the end effector collides with the pose it deems as the target pose
# 						temp_survival = 0.0
# 					else:
# 						temp_survival = 1.0
# 						collisionPerObj = [0.0] * nObjectInPlanning
# 						for ch in collidedHypos:
# 							if ch not in range(0, nhypo):
# 								collisionPerObj[meshSet[ch].objIndx] += meshSet[ch].prob
# 						for cpobs in collisionPerObj:
# 							temp_survival *= (1 - cpobs)
# 					# print "Survival: " + str(temp_survival)
# 					# raw_input("Press Enter to Continue")

# 					if temp_survival >= goalSurvivalThreshold:
# 						## accept this goal
# 						goalSet.append(q_goal)
# 						goalHypo.append(hp)
# 						temp_ngoals += 1
					
# 					temp_trials_ee += 1
			
# 			temp_trials_pose += 1
# 	#print goalHypo
# 	####################################################################################################

# 	##############start sampling##################
# 	currentSamplesFile = folderFile + "/samples_" + str(cc) + "_" + str(nn) + ".txt"
# 	f_samples = open(currentSamplesFile, "w")
# 	nodes = []
# 	temp_counter = 0

# 	while temp_counter < nsamples:
# 		# sample a cartesian ee pose and calculate the IK solution
# 		temp_x = float(format(random.uniform(x_ll, x_ul), '.2f'))
# 		temp_y = float(format(random.uniform(y_ll, y_ul), '.2f'))
# 		temp_z = float(format(random.uniform(z_ll, z_ul), '.2f'))
# 		ikSolution = p.calculateInverseKinematics(kukaID_p, kuka_ee_idx, 
# 						[temp_x, temp_y, temp_z], ll, ul, jr, physicsClientId=clientId)
# 		for j in range(1,8):
# 			result = p.resetJointState(kukaID_p, j, ikSolution[j-1], physicsClientId=clientId)
# 		p.stepSimulation(clientId)
# 		isCollision = collisionCheck_staticG(kukaID_p, static_geometries, clientId)
# 		if isCollision == False:
# 			nodes.append(ikSolution)
# 			## write it into a roadmap file
# 			f_samples.write(str(temp_counter) + " " + str(ikSolution[0]) + " " + str(ikSolution[1]) + " " \
# 				+ str(ikSolution[2]) + " " + str(ikSolution[3]) + " " + str(ikSolution[4]) + " " \
# 				+ str(ikSolution[5]) + " " + str(ikSolution[6]) + "\n")
# 			temp_counter += 1
# 	#####################################################################################################

# 	# startTime = time.clock()
# 	############ connect neighbors to build roadmaps #############
# 	connectivity = np.zeros((nsamples, nsamples))
# 	tree = spatial.KDTree(nodes)
# 	neighbors_const = 1.5 * math.e * (1 + 1/len(home_configuration))
# 	num_neighbors = int(neighbors_const * math.log(nsamples))
# 	if num_neighbors >= nsamples:
# 		num_neighbors = nsamples - 1
# 	# print "num_neighbors: " + str(num_neighbors)

# 	currentRoadmapFile = folderFile + "/roadmap_" + str(cc) + "_" + str(nn) + ".txt"
# 	f_roadmap = open(currentRoadmapFile, "w")
# 	# for each node
# 	for i in xrange(len(nodes)):
# 		queryNode = nodes[i]
# 		knn = tree.query(queryNode, k=num_neighbors, p=2)
# 		# for each neighbor
# 		for j in xrange(len(knn[1])):
# 			if knn[1][j] == i or connectivity[i][knn[1][j]] == 1:
# 				# if the neighbor is the query node itself
# 				# or the connectivity has been checked before
# 				# then skip the edge checking procedure
# 				continue
# 			# Otherwise, check the edge validity (in terms of collision with static geometry)
# 			# between the query node and the the current neighbor
# 			neighbor = nodes[knn[1][j]]
# 			isEdgeValid = checkEdgeValidity(queryNode, neighbor, kukaID_p, 
# 																static_geometries, clientId)
# 			if isEdgeValid:
# 				# write this edge information with their cost and labels into the txt file
# 				# It is a valid edge in terms of static geometry
# 				# Let's check the collision status for each hypothesis for the purpose of labeling
# 				temp_labels = label_the_edge(queryNode, neighbor, kukaID_p, meshSet, clientId)
# 				f_roadmap.write(str(i) + " " + str(knn[1][j]) + " " + format(knn[0][j], '.4f') + " ")
# 				for tl in temp_labels:
# 					f_roadmap.write(str(tl) + " ")
# 				f_roadmap.write("\n")
# 				# update connectivity information
# 				connectivity[i][knn[1][j]] = 1
# 				connectivity[knn[1][j]][i] = 1

# 		# if i % 100 == 99:
# 		# 	print "finish labeling and connecting neighbors for node " + str(i)

# 	# print "finish all the neighbors"
# 	# print "Time elapsed: ", time.clock() - startTime
# 	##########################################################################################################

# 	###### add the start node and goal nodes to the roadmap######
# 	nodes.append(q_start)
# 	f_samples.write(str(temp_counter) + " " + str(q_start[0]) + " " + str(q_start[1]) + " " \
# 		+ str(q_start[2]) + " " + str(q_start[3]) + " " + str(q_start[4]) + " " + \
# 		str(q_start[5]) + " " + str(q_start[6]) + "\n")
# 	temp_counter += 1
# 	## connect the start to the roadmap
# 	tree = spatial.KDTree(nodes)
# 	queryNode = nodes[temp_counter-1]
# 	knn = tree.query(queryNode, k=num_neighbors*2, p=2)
# 	# for each neighbor
# 	highestSurvival_neighborIdx = -1;
# 	highestSurvivalSofar = -1.0
# 	highestSurvival_labels = []
# 	highestSurvival_cost = -1.0
# 	for j in xrange(len(knn[1])):
# 		if knn[1][j] == (temp_counter-1):
# 			continue
# 		else:
# 			# check collision
# 			neighbor = nodes[knn[1][j]]
# 			isEdgeValid = checkEdgeValidity(queryNode, neighbor, kukaID_p, 
# 														static_geometries, clientId)
# 			if isEdgeValid:
# 				## examine the survivability of the edge
# 				## first get the labels
# 				temp_labels = label_the_edge(queryNode, neighbor, kukaID_p, meshSet, clientId)
# 				## compute the survivability
# 				temp_survival = 1.0
# 				collisionPerObj = [0.0] * nObjectInPlanning
# 				for tl in temp_labels:
# 					collisionPerObj[meshSet[tl].objIndx] += meshSet[tl].prob
# 				for cpobs in collisionPerObj:
# 					temp_survival *= (1 - cpobs)
# 				# print "Survival: " + str(temp_survival)
# 				# raw_input("Press Enter to Continue")
# 				if temp_survival > highestSurvivalSofar:
# 					highestSurvivalSofar = temp_survival
# 					highestSurvival_neighborIdx = knn[1][j]
# 					highestSurvival_labels = temp_labels
# 					highestSurvival_cost = knn[0][j]
# 	## finish all the survival computation for all k nearest neighbors
# 	## Now connect the start to the one neighbor with the largest survival (lower cost for tie breaker)
# 	if (highestSurvival_neighborIdx != -1):
# 		f_roadmap.write(str(temp_counter-1) + " " + str(highestSurvival_neighborIdx) + " " \
# 											+ format(highestSurvival_cost, '.4f') + " ")
# 		for hl in highestSurvival_labels:
# 			f_roadmap.write(str(hl) + " ")
# 		f_roadmap.write("\n")
# 		# print "successfully connect the start to the roadmap\n"
	
# 	## Loop through goalSet
# 	for q_goal, hypo in zip(goalSet, goalHypo):
# 		# print "For goal: " + str(temp_counter)
# 		nodes.append(q_goal)
# 		f_samples.write(str(temp_counter) + " " + str(q_goal[0]) + " " + str(q_goal[1]) + " " \
# 			+ str(q_goal[2]) + " " + str(q_goal[3]) + " " + str(q_goal[4]) + " " + \
# 			str(q_goal[5]) + " " + str(q_goal[6]) + " " + str(hypo) + "\n")
# 		temp_counter += 1
# 		## connect the goal to the roadmap
# 		tree = spatial.KDTree(nodes)
# 		queryNode = nodes[temp_counter-1]
# 		knn = tree.query(queryNode, k=num_neighbors*2, p=2)
# 		# for each neighbor
# 		# connectTimes = 0
# 		highestSurvival_neighborIdx = -1;
# 		highestSurvivalSofar = -1.0
# 		highestSurvival_labels = []
# 		highestSurvival_cost = -1.0
# 		for j in xrange(len(knn[1])):
# 			if knn[1][j] == (temp_counter - 1):
# 				continue
# 			else:
# 				# check collision
# 				neighbor = nodes[knn[1][j]]
# 				isEdgeValid = checkEdgeValidity(queryNode, neighbor, kukaID_p, 
# 															static_geometries, clientId)
# 				if isEdgeValid:
# 					## examine the survivability of the edge
# 					## first get the labels
# 					temp_labels = label_the_edge(queryNode, neighbor, kukaID_p, meshSet, clientId)
# 					## compute the survivability
# 					temp_survival = 1.0
# 					collisionPerObj = [0.0] * nObjectInPlanning
# 					for tl in temp_labels:
# 						collisionPerObj[meshSet[tl].objIndx] += meshSet[tl].prob
# 					for cpobs in collisionPerObj:
# 						temp_survival *= (1 - cpobs)
# 					# print "Survival: " + str(temp_survival)
# 					# raw_input("Press Enter to Continue")
# 					if temp_survival > highestSurvivalSofar:
# 						highestSurvivalSofar = temp_survival
# 						highestSurvival_neighborIdx = knn[1][j]
# 						highestSurvival_labels = temp_labels
# 						highestSurvival_cost = knn[0][j]

# 		## finish all the survival computation for all k nearest neighbors for a single specific goal
# 		## Now connect that specific goal to the one neighbor with the largest survival (lower cost for tie breaker)
# 		if (highestSurvival_neighborIdx != -1):
# 			f_roadmap.write(str(temp_counter-1) + " " + str(highestSurvival_neighborIdx) + " " \
# 												+ format(highestSurvival_cost, '.4f') + " ")
# 			for hl in highestSurvival_labels:
# 				f_roadmap.write(str(hl) + " ")
# 			f_roadmap.write("\n")
# 			# print "successfully connect the goal " + str(temp_counter-1) + " to the roadmap\n"

# 	f_samples.close()
# 	f_roadmap.close()

# 	print "roadmap generated with " + str(len(nodes)) + " nodes in " + str(time.clock() - startTime) + " second."