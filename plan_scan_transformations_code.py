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
__email__ = "veit@student.chalmers.se, danjako@student.chalmers.se"
__status__ = "Production"

import time
import numpy as np

from load_parameters_code import load_parameters
from scanner_handler import scan,localize
from rotation_transform_code import *

def plan_scan_transformations(move_robot):
	"""
	In a first step, the entire ground plane of the box is divided in tiles and scanned in a systematic pattern in order to
	examine it for workpieces and thus identify potential points of interest.
	"""

	# Loading some parameters 
	parameters = load_parameters()
	box_size_inside = parameters["box size inside"]
	scanningArea_size = parameters["scanning area size"]
	workpiece_size = parameters["workpiece size"]

	# In order to perform a safe movement, first the scan translations are calculated and depending on them the rotations.  
	scan_translations = calculate_scan_translations(box_size_inside, scanningArea_size, scanningArea_size[0:2])
	# Afterwards, both translations and rotations are combined to the transformations.
	scan_transformations = calculate_scan_transformations(box_size_inside, scanningArea_size, scan_translations, workpiece_size)
	#print(np.array(scan_transformations).shape)
	#print(scan_transformations)

	initialize_optimization(scan_transformations,move_robot,parameters)

def calculate_scan_translations(box_size_inside, scanningArea_size, offsets):
	"""
	The algorithm prefers moving the TCP away from or towards to the robot over moving it radially towards the robot,
	since the function of moving the robot safely then changes more often from forehand to backhand or vice versa than
	from right-handed to left-handed and vice versa.
	This is better because the first movement is faster as well as more gentle on the cables attached to the robot arm than the second one.
	Here, the outer loop that does the movements of the TCP radially towards the robot is defined.
	"""

	scan_translations = []
	# We need this variable to switch between movements of the TCP away from and towards to the robot
	move_positive_y = False
	# The first scan translation is defined such that the scanning area exactly fits one corner of the box
	x = -box_size_inside[0]/2+scanningArea_size[0]/2  
	# The scanning area 
	while x < box_size_inside[0]/2-scanningArea_size[0]/2:
		# Calculate the y coordinates of the translation for every x coordinate with the sign of the y direction defined by move_positive_y 
		scan_translations += calculate_scan_translations_y(x, move_positive_y, box_size_inside, scanningArea_size, offsets)
		# Offset the x coordinate by the defined value before iterating
		x += offsets[0]
		# Switch the sign of the y direction defined by move_positive_y 
		move_positive_y = not move_positive_y
	# Add the newly defined scan translations to the end of the scan translations array
	scan_translations += calculate_scan_translations_y(box_size_inside[0]/2-scanningArea_size[0]/2, move_positive_y, box_size_inside, scanningArea_size, offsets)
	return scan_translations

def calculate_scan_translations_y(x, move_positive_y, box_size_inside, scanningArea_size, offsets):
	"""
	The overall context of this function is described in the calculate_scan_translations function.
	Here, the inner loop that does the movements of the TCP away from or towards to the robot is defined.
	"""

	scan_translations = []
	# Calculate the y coordinates of the translation for every x coordinate with the sign of the y direction defined by move_positive_y 
	if move_positive_y:
		# The first scan translation is defined such that the scanning area exactly fits one edge of the box
		y = -box_size_inside[1]/2+scanningArea_size[1]/2
		# As long as the opposite edge is not reached, we offset the y coordinate by the defined value
		while y < box_size_inside[1]/2-scanningArea_size[1]/2:
			# Add the newly defined scan translation to the end of the scan translations array
			scan_translations += [[x, y]]
			# Offset the y coordinate by the defined value before iterating
			y += offsets[1]
		# Add the last scan translation that is defined such that the scanning area exactly fits the opposite edge of the box to the scan translations array
		scan_translations += [[x, box_size_inside[1]/2-scanningArea_size[1]/2]]
	else:
		# The first scan translation is defined such that the scanning area exactly fits one edge of the box
		y = box_size_inside[1]/2-scanningArea_size[1]/2
		# As long as the opposite edge is not reached, we offset the y coordinate by the defined value
		while y > -box_size_inside[1]/2+scanningArea_size[1]/2:
			# Add the newly defined scan translation to the end of the scan translations array
			scan_translations += [[x, y]]
			# Offset the y coordinate by the defined value before iterating
			y -= offsets[1]
		# Add the last scan translation that is defined such that the scanning area exactly fits the opposite edge of the box to the scan translations array
		scan_translations += [[x, -box_size_inside[1]/2+scanningArea_size[1]/2]]
	return scan_translations

def calculate_scan_transformations(box_size_inside, scanningArea_size, scan_translations, workpiece_size, safety_threshold_walls = 3e-1, safety_threshold_numerical = 1e-3):
	"""
	Not only scans in the top view position of every tile in the ground plane are made, but also from different angles.
	The vector between the endeffector and the coordinate origin of the scanning area describes a cylinder volume,
	as long as the origin is not near a wall of the box.
	The cylinder volume needs to be restricted near the walls of the box such that collisions of the robot or the tool
	attached to the endeffector with the wall are prevented.
	The safety_threshold_walls is necessary to suppress some initial scan positions in order to do not bump into the walls.
	The safety_threshold_numerical is necessary to avoid bumping into the walls due to numerical errors.
	"""
	scan_transformations = []
	# For every single scan translation we define respective initial scan rotations such that the robot cannot bump into the walls of the box.
	for scan_translation in scan_translations:
		rotations = []
		# The following booleans tell us if the distance of the tcp to one wall of the box is smaller than a defined threshold 
		negative_x_wall = abs(scan_translation[0]+box_size_inside[0]/2-scanningArea_size[0]/2) < safety_threshold_walls
		positive_x_wall = abs(scan_translation[0]-box_size_inside[0]/2+scanningArea_size[0]/2) < safety_threshold_walls
		negative_y_wall = abs(scan_translation[1]+box_size_inside[1]/2-scanningArea_size[1]/2) < safety_threshold_walls
		positive_y_wall = abs(scan_translation[1]-box_size_inside[1]/2+scanningArea_size[1]/2) < safety_threshold_walls

		# The initial scan rotations are hardcoded by now for all four edges and all four corners
		parameters = load_parameters()
		maximal_scanning_angle = parameters["maximal scanning angle"]
		scanning_angle_steps = parameters["scanning angle steps"]
		rotations = []

		# corners
		# northwest
		if negative_x_wall and positive_y_wall:
			rotations = [[0, 0, -safety_threshold_numerical],
					[0, maximal_scanning_angle/2, -safety_threshold_numerical],
					[0, maximal_scanning_angle/2, -np.pi/2+safety_threshold_numerical],
				     	[0, maximal_scanning_angle, -np.pi/2+safety_threshold_numerical],
				     	[0, maximal_scanning_angle, -np.pi/4],
					[0, maximal_scanning_angle, -safety_threshold_numerical]]
		# northeast
		elif positive_x_wall and positive_y_wall:
			rotations = [[np.pi, 0, safety_threshold_numerical],
					[np.pi, -maximal_scanning_angle/2, safety_threshold_numerical],
					[np.pi, -maximal_scanning_angle/2, np.pi/2-safety_threshold_numerical],
					[np.pi, -maximal_scanning_angle, np.pi/2-safety_threshold_numerical],
				     	[np.pi, -maximal_scanning_angle, np.pi/4],
					[np.pi, -maximal_scanning_angle, safety_threshold_numerical]]
		# southeast		
		elif positive_x_wall and negative_y_wall:
			rotations = [[np.pi, 0, -safety_threshold_numerical],
					[np.pi, -maximal_scanning_angle/2, -safety_threshold_numerical],
					[np.pi, -maximal_scanning_angle/2, -np.pi/2+safety_threshold_numerical],
				     	[np.pi, -maximal_scanning_angle, -np.pi/2+safety_threshold_numerical],
				     	[np.pi, -maximal_scanning_angle, -np.pi/4],
					[np.pi, -maximal_scanning_angle, -safety_threshold_numerical]]
		# southwest		
		elif negative_x_wall and negative_y_wall:
			rotations = [[0, 0, safety_threshold_numerical],
					[0, maximal_scanning_angle/2, safety_threshold_numerical],
					[0, maximal_scanning_angle/2, np.pi/2-safety_threshold_numerical],
				     	[0, maximal_scanning_angle, np.pi/2-safety_threshold_numerical],
				     	[0, maximal_scanning_angle, np.pi/4],
					[0, maximal_scanning_angle, safety_threshold_numerical]]
		# edges
		# north
		elif not negative_x_wall and not positive_x_wall and positive_y_wall:
			rotations = [[0, 0, -safety_threshold_numerical],
					[0, maximal_scanning_angle/2, -safety_threshold_numerical],
					[0, maximal_scanning_angle/2, -np.pi/2],
					[0, maximal_scanning_angle/2, -np.pi+safety_threshold_numerical],
					[0, maximal_scanning_angle, -np.pi+safety_threshold_numerical],
					[0, maximal_scanning_angle, -np.pi*3/4],
				     	[0, maximal_scanning_angle, -np.pi*1/2],
				     	[0, maximal_scanning_angle, -np.pi*1/4],
					[0, maximal_scanning_angle, -safety_threshold_numerical]]
		# east		
		elif positive_x_wall and not positive_y_wall and not negative_y_wall:
			rotations = [[np.pi, 0, np.pi/2-safety_threshold_numerical],
					[np.pi, maximal_scanning_angle/2, np.pi/2-safety_threshold_numerical],
					[np.pi, maximal_scanning_angle/2, 0],
					[np.pi, maximal_scanning_angle/2, -np.pi/2+safety_threshold_numerical],
					[np.pi, maximal_scanning_angle, -np.pi/2+safety_threshold_numerical],
					[np.pi, maximal_scanning_angle, -np.pi/4],
					[np.pi, maximal_scanning_angle, 0],
					[np.pi, maximal_scanning_angle, np.pi/4],
				     	[np.pi, maximal_scanning_angle, np.pi/2-safety_threshold_numerical]]
		# south	
		elif not positive_x_wall and not negative_x_wall and negative_y_wall:
			rotations = [[0, 0, safety_threshold_numerical],
					[0, maximal_scanning_angle/2, safety_threshold_numerical],
					[0, maximal_scanning_angle/2, np.pi/2],
					[0, maximal_scanning_angle/2, np.pi-safety_threshold_numerical],
					[0, maximal_scanning_angle, np.pi-safety_threshold_numerical],
					[0, maximal_scanning_angle, np.pi*3/4],
				     	[0, maximal_scanning_angle, np.pi*1/2],
				     	[0, maximal_scanning_angle, np.pi*1/4],
					[0, maximal_scanning_angle, safety_threshold_numerical]]
		# west		
		elif negative_x_wall and not negative_y_wall and not positive_y_wall: 
			rotations = [[0, 0, np.pi/2-safety_threshold_numerical],
					[0, maximal_scanning_angle/2, np.pi/2-safety_threshold_numerical],
					[0, maximal_scanning_angle/2, 0],
					[0, maximal_scanning_angle/2, -np.pi/2+safety_threshold_numerical],
					[0, maximal_scanning_angle, -np.pi/2+safety_threshold_numerical],
					[0, maximal_scanning_angle, -np.pi/4],
					[0, maximal_scanning_angle, 0],
					[0, maximal_scanning_angle, np.pi/4],
				     	[0, maximal_scanning_angle, np.pi/2-safety_threshold_numerical]]
		# equator		
		else:
			rotations = [[0, 0, np.pi],
					[0, maximal_scanning_angle/2, np.pi],
					[0, maximal_scanning_angle/2, np.pi/2],
					[0, maximal_scanning_angle/2, 0],
					[0, maximal_scanning_angle/2, -np.pi/2],
					[0, maximal_scanning_angle, -np.pi*3/4],
					[0, maximal_scanning_angle, -np.pi/2],
					[0, maximal_scanning_angle, -np.pi/4],
					[0, maximal_scanning_angle, 0],
					[0, maximal_scanning_angle, np.pi/4],
				     	[0, maximal_scanning_angle, np.pi/2],
				     	[0, maximal_scanning_angle, np.pi/4],
					[0, maximal_scanning_angle, np.pi]]
		# Both scan translations and scan rotations are combined to the scan transformations
		poses = []
		for rotation in rotations:
			poses += [[scan_translation[0], scan_translation[1], workpiece_size[2]-scanningArea_size[2]/2, rotation[0], rotation[1], rotation[2]]]
		scan_transformations.append(poses) #Restructured the data based on each translation point in the box, and then a list of rotations within
	return scan_transformations

def initialize_optimization(scan_transformations,move_robot,parameters):
	# This loop starts all the movements to the scan_transformations contained in the scan_transformations array
	for scan_translation in scan_transformations:
		successful_scans = []
		for scan_transformation in scan_translation:
			# In order to be able to describe movements around the coordinate origin of the scanning area, the TCP is placed to this point.
			scan_translation = [scan_transformation[0],\
						scan_transformation[1],\
						scan_transformation[2]]
			scan_rotation_ypy = [scan_transformation[3],\
						scan_transformation[4],\
						scan_transformation[5]]
			scan_rotation_rpy = ypy_to_rpy(scan_rotation_ypy)
			scan_transformation_rpy = scan_translation+[scan_rotation_rpy[0], scan_rotation_rpy[1], scan_rotation_rpy[2]]
			move_robot.absolute_tcp_transformation(scan_transformation_rpy, tcp = "scanning area", base = "box", move = "l")
			scan_id = scan(parameters["scan_port"])
			try:
				output = localize(scan_id, parameters["locate_port"], parameters["plcfFile"], parameters["timeout"], parameters["overlap"], parameters["maxNbrOfResults"])
				for index, _ in enumerate(range(len(output["Result"]))):
					rotation, translation, overlap, succesful_scan, _ = extract_scan_output(output, index)
			
					successful_scans += [{
						"scan_transformation" : scan_transformation,
						"rotation" : rotation,
						"translation" : translation,
						"overlap" : overlap
						}]
			except:
				# This time is defined to simulate the scanning duration
				time.sleep(5)
		print(successful_scans)
		optimize_angles(successful_scans, move_robot, parameters)

def extract_scan_output(output,index):
	"""
	This function extracts the following data out of the output of the scanner:
	rotation, Translation, overlap, successful_scan, time
	"""
	rotation = 0
	translation = 0
	overlap = 0
	time = 0
	succesful_scan = False
	try:
		transform = output['Result'][index]['Transform']
		overlap = float(output['Result'][index]['VisibleOverlap'])
		time = output['Result'][index]['Time']
		rotation_matrix = np.zeros((3,3))
		translation = np.zeros(3)
		for i in range(3):
			for j in range(3):
				rotation_matrix[i][j] = transform[i][j]
			translation[i] = transform[i][-1]
		r, p, y = rotation_matrix_to_rpy(rotation_matrix)
		rotation = np.array[r, p, y]
		succesful_scan = True
	except:
		pass

	return rotation,translation,overlap,succesful_scan,time

def optimize_angles(successful_scans,move_robot,parameters):
	"""
	The angle to the points of interest are optimized iteratively with each scan.
	"""
	if(len(successful_scans) > 2):
		sorted_scans = sorted(successful_scans, key = lambda x: x['overlap'])
		top_three_scans = [sorted_scans[-1], sorted_scans[-2], sorted_scans[-3]]
		while(True):			
			print("Top three scans:", top_three_scans)
			new_transformation = []
			for i in range(6):			
				new_transformation.append((top_three_scans[0]['scan_transformation'][i]*top_three_scans[0]['overlap']+\
							top_three_scans[1]['scan_transformation'][i]*top_three_scans[1]['overlap']+\
							top_three_scans[2]['scan_transformation'][i]*top_three_scans[2]['overlap'])/(\
							top_three_scans[0]['overlap']+\
							top_three_scans[1]['overlap']+\
							top_three_scans[2]['overlap']))
			new_translation = [new_transformation[0],\
						new_transformation[1],\
						new_transformation[2]]
			new_rotation_ypy = [new_transformation[3],\
						new_transformation[4],\
						new_transformation[5]]
			new_rotation_rpy = ypy_to_rpy(new_rotation_ypy)
			new_transformation_rpy = new_translation+[new_rotation_rpy[0], new_rotation_rpy[1], new_rotation_rpy[2]]
			move_robot.absolute_tcp_transformation(new_transformation_rpy, tcp = "scanning area", base = "box", move = "l")
			better_scan = False
			scan_id = scan(parameters["scan_port"])
			try:
				output = localize(scan_id,parameters["locate_port"], parameters["plcfFile"], parameters["timeout"], parameters["overlap"], parameters["maxNbrOfResults"])
				for index, _ in enumerate(range(len(output["Result"]))):
					new_rotation, new_translation, new_overlap, succesful_scan, _ = extract_scan_output(output, index)
					print(new_overlap)				
					if new_overlap > top_three_scans[0]['overlap']:
						top_three_scans[2] = top_three_scans[1]
						top_three_scans[1] = top_three_scans[0] # Want to keep the best scan on the 0 position to keep track of it
						top_three_scans[0]['scan_transformation'] = new_transformation
						top_three_scans[0]['rotation'] = new_rotation
						top_three_scans[0]['translation'] = new_translation
						top_three_scans[0]['overlap'] = new_overlap
						better_scan = True
					elif(new_overlap > top_three_scans[1]['overlap']):
						top_three_scans[2] = top_three_scans[1] # Want to keep the best scan on the 0 position to keep track of it
						top_three_scans[1]['scan_transformation'] = new_transformation
						top_three_scans[1]['rotation'] = new_rotation
						top_three_scans[1]['translation'] = new_translation
						top_three_scans[1]['overlap'] = new_overlap					
						better_scan = True
					elif(new_overlap > top_three_scans[2]['overlap']):
						top_three_scans[2]['scan_transformation'] = new_transformation
						top_three_scans[2]['rotation'] = new_rotation
						top_three_scans[2]['translation'] = new_translation
						top_three_scans[2]['overlap'] = new_overlap
						better_scan = True
			except:
				# This time is defined to simulate the scanning duration
				time.sleep(5)
			if not better_scan:
				break
		print("Output :" + str(top_three_scans[0]['scan_transformation']))
		output_file = open(r"output.txt","a")
		output_file.write(str(top_three_scans[0]['scan_transformation'])+"\n")
		output_file.close()

	elif(len(successful_scans) == 2):
		sorted_scans = sorted(successful_scans, key = lambda x: x['overlap'])
		top_two_scans = [sorted_scans[-1],sorted_scans[-2]]
		while(True):			
			print("Top two scans:", top_two_scans)
			new_transformation = []
			for i in range(6):			
				new_transformation.append((top_two_scans[0]['scan_transformation'][i]*top_two_scans[0]['overlap']  +\
							top_two_scans[1]['scan_transformation'][i]*top_two_scans[1]['overlap'])/(\
							top_two_scans[0]['overlap']+\
							top_two_scans[1]['overlap']))
			new_translation = [new_transformation[0],\
						new_transformation[1],\
						new_transformation[2]]
			new_rotation_ypy = [new_transformation[3],\
						new_transformation[4],\
						new_transformation[5]]
			new_rotation_rpy = ypy_to_rpy(new_rotation_ypy)
			new_transformation_rpy = new_translation+[new_rotation_rpy[0], new_rotation_rpy[1], new_rotation_rpy[2]]
			move_robot.absolute_tcp_transformation(new_transformation_rpy, tcp = "scanning area", base = "box", move = "l")
			better_scan = False
			scan_id = scan(parameters["scan_port"])		
			try:
				output = localize(scan_id,parameters["scan_port"], parameters["plcfFile"], parameters["timeout"], parameters["overlap"], parameters["maxNbrOfResults"])
				for index, _ in enumerate(range(len(output["Result"]))):
					new_rotation, new_translation, new_overlap, succesful_scan, _ = extract_scan_output(output, index)
					print(new_overlap)
					if new_overlap > top_two_scans[0]['overlap']:
						top_two_scans[1] = top_two_scans[0] # Want to keep the best scan on the 0 position to keep track of it
						top_two_scans[0]['scan_transformation'] = new_transformation
						top_two_scans[0]['rotation'] = new_rotation
						top_two_scans[0]['translation'] = new_translation
						top_two_scans[0]['overlap'] = new_overlap
						better_scan = True
					elif(new_overlap > top_two_scans[1]['overlap']):
						top_two_scans[1]['scan_transformation'] = new_transformation
						top_two_scans[1]['rotation'] = new_rotation
						top_two_scans[1]['translation'] = new_translation
						top_two_scans[1]['overlap'] = new_overlap
						better_scan = True
			except:
				# This time is defined to simulate the scanning duration
				time.sleep(5)
			if not better_scan:
				break
		output_file = open(r"output.txt","a")
		output_file.write(str(top_two_scans[0]['scan_transformation'])+"\n")
		output_file.close()
		print("Output :" + str(top_two_scans[0]['scan_transformation']))

	elif(len(successful_scans) == 1):
		output_file = open(r"output.txt","a")
		output_file.write(str(successful_scans[0]['scan_transformation'])+"\n")
		output_file.close()
