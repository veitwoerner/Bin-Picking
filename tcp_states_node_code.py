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
from project.msg import TcpState
import time

def callback(data, tcp_transformation_queue):
	# A queue of size 2 is used, since this is the best compromise between a delay and the reader-writer-problem.
	# This means, that the returned state is the one of newest finished cycle.
	# If there already are 2 elements in the queue, we put the old one out and the new one in
	if tcp_transformation_queue.qsize() == 2:
		tcp_transformation_queue.get()
    	tcp_transformation_queue.put(data.pose)

def tcp_states_node(tcp_transformation_queue):
	"""
	The algorithm does not need the TCP states continuously, but only at certain times.
	This node therefore serves as subscriber for the endless published states and returns on request the current value of the states.
	"""
	rospy.init_node('tcp_states_node', anonymous = True)
    	rospy.Subscriber('tcp_states', TcpState, callback, tcp_transformation_queue)
    	rospy.spin()
