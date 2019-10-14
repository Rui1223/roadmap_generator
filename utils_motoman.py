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
	quat = p.getQuaternionFromEuler([angles[0], angles[1], angles[2]])
	print "type quat:" + str(type(quat))
	for i in xrange(len(quat)):
		quat[i] = float(format(quat[i], '.3f'))
	_m = p.createMultiBody(baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v, 
											basePosition=pos, baseOrientation=quat, physicsClientId=clientId)
	objMesh.append(Mesh(_m, objName, objIdx, hypoIdx, pos, quat, 1.0, objRole))

	return objMesh

def createHypoMesh(currentlabelIdx, objIdx, meshFile, objName, objRole, scale, pos, angles, 
								transError, orientError, clientId, static_geometries, benchmarkType, nHypos):
	### starting label index for the current obs (currentlabelIdx)
	### with the mesh file and scale
	### create the collision and visual shape of the object
	hypotheses = []

	### first specify the collision and visual geometry
	_c = p.createCollisionShape(shapeType=p.GEOM_MESH, 
			fileName=meshFile, meshScale=[scale, scale, scale], physicsClientId=clientId)
	_v = p.createVisualShape(shapeType=p.GEOM_MESH,
			fileName=meshFile, meshScale=[scale, scale, scale], physicsClientId=clientId)

	largest_prob = 0.0
	largest_prob_idx = labelIdx
	### for each hypothesis
	h = 0

	while (h < nHypos):
		### generate new pos and new quat according to transError and orientError
		temp_px = float(format(random.uniform(pos[0]-transError, pos[0]+transError), '.3f'))
		temp_py = float(format(random.uniform(pos[1]-transError, pos[1]+transError), '.3f'))
		temp_oz = float(format(random.uniform(angles[2]-orientError, angles[2]+orientError), '.3f'))
		temp_pos = [temp_px, temp_py, pos[2]] ### so far no uncertainty in height (z axis)
		temp_angles = [angles[0], angles[1], temp_oz] ### so far only uncertainty in the orientation around z axis
		temp_quat = p.getQuaternionFromEuler(temp_angles)
		### create the hypothesis mesh and save it into the hypotheses
		_m = p.createMultiBody(baseCollisionShapeIndex=_c, baseVisualShapeIndex=_v, 
										basePosition=temp_pos, baseOrientation=temp_quat, physicsClientId=clientId)
		### Before adding the mesh into the hypotheses, check if it collides with static geometries
		temp_collided = collisionCheck_specificG(_m, static_geometries, benchmarkType, clientId)
		if (temp_collided):
			p.removeBody(_m, clientId)
			continue
		hypotheses.append( Mesh(_m, objName) )
		# def __init__(self, m, objName, objIdx, hypoIdx, pos, quat, prob, objRole)

		### compute the unnormalized probability of this hypothesis
		temp_prob = comp_prob(pos, angles, temp_pos, temp_angles)
		if temp_prob > largest_prob:
			largest_prob = temp_prob
			largest_prob_idx = labelIdx+h



def printPoses(meshes):
	for mesh in meshes:
		print "hypo " + str(mesh.hypoIdx) + " " + str(mesh.pos) + " " + str(mesh.quat) + " " + \
							str(mesh.prob) + "\tfor object " + str(mesh.objIdx) + ": " + mesh.objName
	print "--------------------------------------\n"

def collisionCheck_specificG(m, static_geometries, benchmarkType, clientId):
	isCollision = False
	for g in static_geometries:
		if benchmarkType == "table" and str(g) == "1":
			continue
		if benchmarkType == "shelf" and str(g) == "2":
			continue
		if benchmarkType == "shelf" and str(g) == "6":
			continue
		contacts = p.getClosestPoints(m, g, 0.01, physicsClientId=clientId)
		if len(contacts) != 0:
			isCollision = True
			# print "static collision with " + str(g) + "when deploying the hypothesis"
			break

	return isCollision


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

def plannScene_generation(Objects, benchmarkType, static_geometries, nHypos, transError, orientError, clientId):
	hypotheses = []
	mostPromisingHypoIdx = []
	currentlabelIdx = 0
	nObjectInPlanning = 0
	for i in xrange(len(Objects)):
		if Object[i][3] == "invisible":
			continue;
		mm, pp = createHypoMesh(currentlabelIdx, Objects[i][0], Objects[i][1], Objects[i][2], Objects[i][3], Objects[i][4], 
			Objects[i][5], Objects[i][6], transError, orientError, clientId, static_geometries, benchmarkType, nHypos) 

