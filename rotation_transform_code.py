#!/usr/bin/env python
# coding: utf-8

#####################################################################################
# general information                                                               #
# all "..._transformations" vectors have the following layout:                      #
# [translation_x, translation_y, translation_z, rotation_p, rotation_r, rotation_y] #
# all "..._size" vectors have the following layout:                                 #
# [size_x, size_y, size_z]                                                          #
#####################################################################################

__author__ = "Daniel Jacobsson, Zixiao Ren, Veit Wörner"
__credits__ = ["Daniel Jacobsson", "Zixiao Ren", "Veit Wörner"]
__license__ = "GPL"
__version__ = "0.9b"
__maintainer__ = "Veit Wörner"
__email__ = "veit@student.chalmers.se"
__status__ = "Production"

import math
import numpy as np
from tf.transformations import *

def rpy_to_rotation_vector(rpy):
	"""
	This is the transform from the euler angle (RPY) representation to the rotation vector representation.
	For more information please have a look at the following webpage:
	https://www.zacobria.com/universal-robots-knowledge-base-tech-support-forum-hints-tips/python-code-example-of-converting-rpyeuler-angles-to-rotation-vectorangle-axis-for-universal-robots/
	"""

	r, p, y = rpy
	# Standart rotation matrices
	roll_rotation_matrix = np.matrix([[1, 0          , 0           ],
					[0, math.cos(r), -math.sin(r)],
					[0, math.sin(r),  math.cos(r)]])
	pitch_rotation_matrix = np.matrix([[ math.cos(p), 0, math.sin(p)],
					[0           , 1, 0          ],
					[-math.sin(p), 0, math.cos(p)]])
	yaw_rotation_matrix = np.matrix([[math.cos(y), -math.sin(y), 0],
					[math.sin(y),  math.cos(y), 0],
					[0          , 0           , 1]])
	
	rotation_matrix = yaw_rotation_matrix*pitch_rotation_matrix*roll_rotation_matrix
	theta = math.acos(((rotation_matrix[0, 0]+rotation_matrix[1, 1]+rotation_matrix[2, 2])-1)/2)
	
	# If all three rotations are zero, we would divide by zero. Therefore, we need to handle that case extra
	singular = theta < 1e-6
	if singular:
		rx = ry = rz = 0
	else: 
		multi = 1/(2*math.sin(theta))
		rx = multi*(rotation_matrix[2, 1]-rotation_matrix[1, 2])*theta
		ry = multi*(rotation_matrix[0, 2]-rotation_matrix[2, 0])*theta
		rz = multi*(rotation_matrix[1, 0]-rotation_matrix[0, 1])*theta

	return rx, ry, rz

def rotation_matrix_to_ypy(rotation_matrix):
	"""
	This is the transform from the rotation matrix representation to the euler angle (YPY) representation.
	"""
	ypy = euler_from_matrix(rotation_matrix, axes='szyz')
	return ypy

def ypy_to_rpy(ypy):
	"""
	This is the transform from the euler angle (YPY) representation to the euler angle (RPY) representation.
	"""
	ry1, rp, ry2 = ypy 
	quaternion = quaternion_from_euler(ry1, rp, ry2, axes='szyz')
	rpy = euler_from_quaternion(quaternion, axes='sxyz')
	return rpy
