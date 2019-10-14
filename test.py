from __future__ import division

import math
import random
import time
import numpy as np

import sys
import os
import subprocess

from scipy import spatial
import cPickle as pickle

import IPython

def test1(Object):
	Object[4] = 3

def test2(element):
	element = 3 




if __name__ == '__main__':
	Objects = dict()
	Objects[0] = [0, "/mesh/rawlings_baseball/rawlings_baseball.obj", "baseball", 
		"target", 2, [0.20, 0.0, 0.07], [math.pi/5.8, math.pi/0.4, math.pi/2.3], 3.2, 3]

	test1(Objects[0])
	# test2(Objects[0][4])
	print "scale: " + str(Objects[0][4]) 

