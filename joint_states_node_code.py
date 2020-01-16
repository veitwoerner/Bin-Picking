#!/usr/bin/env python
# coding: utf-8

__author__ = "Daniel Jacobsson, Zixiao Ren, Veit Wörner"
__credits__ = ["Daniel Jacobsson", "Zixiao Ren", "Veit Wörner"]
__license__ = "GPL"
__version__ = "0.9b"
__maintainer__ = "Veit Wörner"
__email__ = "veit@student.chalmers.se"
__status__ = "Production"

import rospy
from sensor_msgs.msg import JointState
import time

def callback(data, joint_transformation_queue):
	# A queue of size 2 is used, since this is the best compromise between the delay and the reader-writer-problem.
	# This means, that the returned state always is delayed by one cycle.
	# If there already are 2 elements in the queue, we put the old one out and the new one in
	if joint_transformation_queue.qsize() == 2:
		joint_transformation_queue.get()
    	joint_transformation_queue.put(data.position)

def joint_states_node(joint_transformation_queue):
	"""
	The algorithm does not need the joint states continuously, but only at certain times.
	This node therefore serves as subscriber for the endless published states and returns on request the current value of the states.
	"""
	rospy.init_node('joint_states_node', anonymous = True)
    	rospy.Subscriber('joint_states', JointState, callback, joint_transformation_queue)
    	rospy.spin()
