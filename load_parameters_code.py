#!/usr/bin/env python
# coding: utf-8

#####################################################################################
# general information                                                               #
# all "..._transformations" vectors have the following layout:                      #
# [translation_x, translation_y, translation_z, rotation_r, rotation_p, rotation_y] #
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

import numpy as np

def load_parameters():

	"""
	In order to achieve a behaviour of the algorithm that is safe both for the robot with the tool attached to the endeffector as well as for its environment, the output variables must change depending on the input quantities.
	For this purpose, this loading file is used which contains all parameters and variables such as the dimensions of various components of the test setup.
	These are needed to plan all possible and necessary transformations of a defined TCP in a defined base.
	Furthermore, these quantities determine the movement of the robot and tool between the TCP transformations.
	
	Inputs: _
	
	Outputs: parameters - library with all elements listed below
			  			  	host
						 	port
						  	robot absolute transformations base
						  	workpiece size
		      	  		  		box size outside
						  	box size inside
							box absolute transformation outside
							box absolute transformation inside
							scanning area relative transformation
							scanning area size
	"""
	
	# Robot ip address
	#host="localhost"
	host="192.168.20.42"
	
	# To send commands, port 30002 is used
	# The state listener uses port 30003
	port=30002

	# This is the absolute transformation of the robot base 
	robot_absolute_transformation_base = [0,0,0,0,0,0]

	# This is the size of the bounding box of the workpiece
	workpiece_size = [0.12, 0.24, 0] # m

	# This is the size of the bounding box of the pallet
	pallet_size = [1.200,0.800,0.144] # m
	# This is the size of the outer bounding box of one collar
	collar_size_outside = [1.200,0.800,0.2] # m
	# This is the size of the inner bounding box of one collar
	collar_size_inside = [1.160,0.760,0.2] # m
	# This is the size of the bounding box of the lid
	lid_size = [1.200,0.800,0.006] # m

	# The number of collars is defined generic and can be changed here.
	number_collars = 4

	# This is the size of the outer bounding box of the box, depending on the number of collars
	box_size_outside = [pallet_size[0],pallet_size[1],pallet_size[2]+lid_size[2]+number_collars*collar_size_outside[2]]
	# This is the size of the inner bounding box of the box, depending on the number of collars
	box_size_inside = [collar_size_inside[0],collar_size_inside[1],number_collars*collar_size_inside[2]]

	# This is the absolute transformation of the outer coordinate system of the box.
	# The outer coordinate system is located at the center on the bottom of the pallet.
	box_absolute_transformation_outside = [0,-0.59,-0.962,0,0,0] # m, rad
	# This is the absolute transformation of the inner coordinate system of the box
	# The inner coordinate system is located at the center on the top of the lid.
	box_absolute_transformation_inside = [box_absolute_transformation_outside[0],\
					      				  box_absolute_transformation_outside[1],\
					      				  box_absolute_transformation_outside[2]+pallet_size[2]+lid_size[2],\
					      				  box_absolute_transformation_outside[3],\
                                          box_absolute_transformation_outside[4],\
                                          box_absolute_transformation_outside[5]]

	# This is the transformation of the scanning area coordinate system relative to the tool center point coordinate system
	scanningArea_relative_transformation = np.array([-0.2716,0.0023,0.6487,0,np.deg2rad(-15),0])-np.array([0,0,0.0856,0,0,0]) # m
	# This is a pessimistic approximation of the size of the scanning area.
	#scanningArea_size = [0.237,0.343,0.136] # m
	scanningArea_size = [0.343,0.343,0.136] # m
	maximal_scanning_angle = np.pi/10
	scanning_angle_steps = 2

	plcfFile = "bygel_5s_50p"
	timeout = 10000
	locate_port = 9081
	scan_port = 9080
	maxNbrOfResults = 6
	overlap = 25.0

	speed = 0.2

	# The parameters are returned as a library with the following keys
	parameters = {"host": host,\
			"port": port,\
			"robot absolute transformations base": robot_absolute_transformation_base,\
			"workpiece size": workpiece_size,\
		      	"box size outside": box_size_outside,\
		      	"box size inside": box_size_inside,\
		      	"box absolute transformation outside": box_absolute_transformation_outside,\
		      	"box absolute transformation inside": box_absolute_transformation_inside,\
		      	"scanning area relative transformation": scanningArea_relative_transformation,\
		      	"scanning area size": scanningArea_size,\
			"maximal scanning angle": maximal_scanning_angle,\
			"scanning angle steps": scanning_angle_steps,\
			"scan_port": scan_port,\
			"locate_port" : locate_port,\
			"plcfFile": plcfFile,\
			"timeout": timeout,\
			"maxNbrOfResults": maxNbrOfResults,\
			"overlap": overlap,\
			"speed": speed}
	return parameters
