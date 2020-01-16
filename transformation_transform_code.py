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

import numpy as np
from load_parameters_code import load_parameters

def coordinate_transform(old_coordinates, old_base = "box", new_base = "robot base"):
	"""
	In order to be able to specify the position at which the TCP is moved relative to the coordinate origin of the box,
	the possibility to transform the base is necessary.
	"""

	# Loading some parameters 
	parameters = load_parameters()
	box_absolute_transformation_inside = parameters["box absolute transformation inside"]

	# The transformation in both directions is supported
	if old_base == "robot base" and new_base == "box":
		box_absolute_transformation_inside = parameters["box absolute transformation inside"]	
		new_coordinates = np.array(old_coordinates)-np.array(box_absolute_transformation_inside)
	if old_base == "box" and new_base == "robot base":
		box_absolute_transformation_inside = parameters["box absolute transformation inside"]	
		new_coordinates = np.array(old_coordinates)+np.array(box_absolute_transformation_inside)
	return new_coordinates
