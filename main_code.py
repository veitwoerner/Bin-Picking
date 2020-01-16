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

import time
import numpy as np

from plan_scan_transformations_code import plan_scan_transformations
from load_parameters_code import load_parameters
from move_robot_code import *

parameters = load_parameters()
# Here, the move_robot class is instanziated.
# Since we want to have just one socket connection, we will reuse this class in all functions.
move_robot = Move_robot(host=parameters["host"],port=parameters["port"])

def main():
    """
    This is the main entry point.
    Please find the description of the respective functions where they are defined
    """
    # This moves the robot to a initial position. Attention! This movement is NOT safe! 
    move_robot.absolute_tcp_transformation([0,0,1,0,0,0], safe = False)
    # The whole definition of the initial scanning pattern is done in the path planning method.
    plan_scan_transformations(move_robot)
    # This moves the robot back to the initial position. This movement is safe.
    move_robot.absolute_tcp_transformation([0,0,1,0,0,0])

if __name__ == '__main__':
    main()
