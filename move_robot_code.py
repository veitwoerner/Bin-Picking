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

import rospy
import socket
import numpy as np
import time
import math

from transformation_transform_code import coordinate_transform
from tcp_states_node_code import tcp_states_node
from joint_states_node_code import joint_states_node
from multiprocessing import Process, Queue
from numpy.linalg import norm
from load_parameters_code import load_parameters
from rotation_transform_code import rpy_to_rotation_vector

class Move_robot():
	def __init__(self,host="localhost",port = 30002):
		"""
		The core of the algorithm must be able to send commands to the robot and the 3D scanner and react directly or indirectly to their feedback.
		Four types of robot movements are supported:
		Both the movement of the TCP to a given absolute transformation or around by a given relative transformation
		and the movement of the joints to a given absolute transformation or by a given relative transformation.
		"""
		# Here, the move_robot class is initialized.
		# Since we want to have just one socket connection, we will reuse the instantiation of this class in all functions.
		self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.s.connect((host,port))
		# One needs to delay python after sending a command because it can work faster than URSim or UR Polyscope.
		# If one does not delay python and it therefore sends another command too early, the first one is ignored.
		time.sleep(0.1) 

		# Here, the queues for the state nodes are defined. The queue size of two is explained in the respective codes.
		self.tcp_transformation_queue = Queue(2)
		self.joint_transformation_queue = Queue(2)

		# The nodes serve as subscriber for the endless published states and therefore need to run as background processes.
		self.tcp_transformation_process = Process(target = tcp_states_node, args = ((self.tcp_transformation_queue),))
		self.joint_transformation_process = Process(target = joint_states_node, args = ((self.joint_transformation_queue),))
		self.tcp_transformation_process.daemon = True
		self.joint_transformation_process.daemon = True
		self.tcp_transformation_process.start()
		self.joint_transformation_process.start()

		# Initialize the parameters and variables
		self.initialization = True
		absolute_transformation = self.get_tcp_transformation()
		self.qnear = [None]*6
		self.tool_center_transformation = [0,0,0,np.pi,0,0]
		self.corner_in_direction = ""
		self.initialize_safe_movement(absolute_transformation,None)
		self.initialization = False
		self.tcp = "robot endeffector"
		self.parameters = load_parameters()

		# Initialize URSim or UR Polyscope
		self.s.send("set_gravity([0.0, 0.0, 9.81])"+"\n"+
				"set_payload(2.0)"+"\n"+
				"set_standard_analog_input_domain(0,1)"+"\n"+
				"set_standard_analog_input_domain(1,1)"+"\n"+
				"set_tool_analog_input_domain(0,1)"+"\n"+
				"set_tool_analog_input_domain(1,1)"+"\n"+
				"set_analog_outputdomain(0,0)"+"\n"+
				"set_analog_outputdomain(1,0)"+"\n"+
				"set_input_actions_to_default()"+"\n"+
				"set_tool_voltage(0)"+"\n"+
				"set speed %s" %self.parameters["speed"]+"\n")
		time.sleep(0.1)
###############################################################################################################################################################
	def absolute_tcp_transformation(self, absolute_transformation, base = "box", move = "j", threshold = 1e-3, safe = True, tcp = "robot endeffector"):
		"""
		Movement of the TCP to a given absolute transformation.
		One can define the TCP and the base of the robot for every movement.
		The way of movement and the threshold between the ideal robot position and the real one is choosable.
		In this context, a movement is defined as safe if the robot moves in such a way, that the arm should not bump into the wall of the box.  
		"""
		self.tcp = tcp
		old_transformation = self.get_tcp_transformation()
		if base == "robot base":
			[tx,ty,tz,r,p,y] = absolute_transformation
		else:
			[tx,ty,tz,r,p,y] = coordinate_transform(absolute_transformation, old_base = base, new_base = "robot base")
		if safe:
			move = self.initialize_safe_movement([tx,ty,tz,r,p,y],move)
		self.move_robot_tcp_transformation(old_transformation, [tx,ty,tz,r,p,y], move, threshold)			

	def relative_tcp_transformation(self, relative_transformation, move = "j", threshold = 1e-3, safe = True):
		"""
		Movement of the TCP around a given relative transformation.
		The way of movement and the threshold between the ideal robot position and the real one is choosable.
		In this context, a movement is defined as safe if the robot moves in such a way, that the arm should not bump into the wall of the box.  
		"""
		old_transformation = self.get_tcp_transformation()
		[tx,ty,tz,r,p,y] = np.array(old_transformation)+np.array(relative_transformation)
		if safe:
			move = self.initialize_safe_movement([tx,ty,tz,r,p,y],move)
		self.move_robot_tcp_transformation(old_transformation, [tx,ty,tz,r,p,y], move, threshold)

	def absolute_joint_transformation(self, absolute_transformation, move = "j", threshold = 1e-3):
		"""
		Movement of the joints to a given absolute transformation.
		The way of movement and the threshold between the ideal robot position and the real one is choosable.
		"""
		old_transformation = self.get_joint_transformation()
		[r0,r1,r2,r3,r4,r5] = absolute_transformation
		self.move_robot_joint_transformation(old_transformation, [r0,r1,r2,r3,r4,r5], move, threshold)

	def relative_joint_transformation(self, relative_transformation, move = "j", threshold = 1e-3):
		"""
		Movement of the joints by a given absolute transformation.
		The way of movement and the threshold between the ideal robot position and the real one is choosable.
		"""
		old_transformation = self.get_joint_transformation()
		[r0,r1,r2,r3,r4,r5] = np.array(old_transformation)+np.array(relative_transformation)
		self.move_robot_joint_transformation(old_transformation, [r0,r1,r2,r3,r4,r5], move, threshold)

###############################################################################################################################################################

	def move_robot_tcp_transformation(self, old_transformation, new_transformation, move, threshold):
		"""
		Commands sent via socket are executed directly without waiting for the previously sent commands to be terminated
		and the duration of the execution of a movement cannot be intuitively predicted.
		Therefore, a feedback loop of the TCP states is required.
		The next command may only be sent as soon as the error of the current state compared to the target state falls below a certain value.
		"""

		self.set_tcp()
		
		[old_tx,old_ty,old_tz,old_rx,old_ry,old_rz] = old_transformation
		[new_tx,new_ty,new_tz,new_r,new_p,new_y] = new_transformation
		new_rx, new_ry, new_rz = rpy_to_rotation_vector([new_r, new_p, new_y])
		command = "move"+move+"(get_inverse_kin(p["+str(new_tx)+", "+\
													str(new_ty)+", "+\
													str(new_tz)+", "+\
													str(new_rx)+", "+\
													str(new_ry)+", "+\
													str(new_rz)+"], "+\
													"qnear = "+str(self.qnear)+"))"+"\n"
		self.s.send(command)
		# One needs to delay python after sending a command because it can work faster than URSim or UR Polyscope.
		# If one does not delay python and it therefore sends another command too early, the first one is ignored.
		time.sleep(0.1)
		while True:
			"""print(abs(old_tx-new_tx)+\
				    	abs(old_ty-new_ty)+\
				   		abs(old_tz-new_tz)+\
				    	self.compare_angles(old_rx,new_rx)+\
				    	self.compare_angles(old_ry,new_ry)+\
				    	self.compare_angles(old_rz,new_rz))"""
			criterion = abs(old_tx-new_tx)+\
				    	abs(old_ty-new_ty)+\
				   		abs(old_tz-new_tz)+\
				    	self.compare_angles(old_rx,new_rx)+\
				    	self.compare_angles(old_ry,new_ry)+\
				    	self.compare_angles(old_rz,new_rz) < threshold
			if criterion:
				break
			[old_tx,old_ty,old_tz,old_rx,old_ry,old_rz] = self.get_tcp_transformation()
		time.sleep(0.1)

	def move_robot_joint_transformation(self, old_transformation, new_transformation, move, threshold):
		"""
		Commands sent via socket are executed directly without waiting for the previously sent commands to be terminated
		and the duration of the execution of a movement cannot be intuitively predicted.
		Therefore, a feedback loop of the joint states is required.
		The next command may only be sent as soon as the error of the current state compared to the target state falls below a certain value.
		"""

		self.s.send("set_tcp(p"+str(self.tool_center_transformation)+")"+"\n")
		# One needs to delay python after sending a command because it can work faster than URSim or UR Polyscope.
		# If one does not delay python and it therefore sends another command too early, the first one is ignored.
		time.sleep(0.1)
		[old_r0,old_r1,old_r2,old_r3,old_r4,old_r5] = old_transformation		
		[new_r0,new_r1,new_r2,new_r3,new_r4,new_r5] = new_transformation
		command = "move"+move+"(["+str(new_r0)+", "+\
		            		   		str(new_r1)+", "+\
					   				str(new_r2)+", "+\
					   				str(new_r3)+", "+\
			              	   		str(new_r4)+", "+\
					   				str(new_r5)+"])"+"\n"	
		self.s.send(command)
		time.sleep(0.1)
		while True:
			"""print(self.compare_angles(old_r0,new_r0)+\
				    self.compare_angles(old_r1,new_r1)+\
				    self.compare_angles(old_r2,new_r2)+\
				    self.compare_angles(old_r3,new_r3)+\
				    self.compare_angles(old_r4,new_r4)+\
				    self.compare_angles(old_r5,new_r5))"""
			criterion = self.compare_angles(old_r0,new_r0)+\
				    self.compare_angles(old_r1,new_r1)+\
				    self.compare_angles(old_r2,new_r2)+\
				    self.compare_angles(old_r3,new_r3)+\
				    self.compare_angles(old_r4,new_r4)+\
				    self.compare_angles(old_r5,new_r5) < threshold
			if criterion:
				break
			[old_r0,old_r1,old_r2,old_r3,old_r4,old_r5] = self.get_joint_transformation()
		time.sleep(0.1)

###############################################################################################################################################################

	def get_tcp_transformation(self):
		"""
		Returns the TCP states of the newest finished cycle
		"""

		while True:
			if self.tcp_transformation_queue:
				tcp_transformation = self.tcp_transformation_queue.get()
				# Attention!
				# The following query is very unpleasant but unavoidable due to the reader-writer problem.
				# This means that tcp positions very close to p[0,0,0,0,0,0] make the code get stuck!	
				if norm(tcp_transformation) > 0.1:
					break
			time.sleep(0.01)
		return tcp_transformation

	def get_joint_transformation(self):
		"""
		Returns the joint states of the newest finished cycle
		"""

		while True:
			if self.joint_transformation_queue:
				joint_transformation = self.joint_transformation_queue.get()
				# Attention!
				# The following query is very unpleasant but unavoidable due to the reader-writer problem.
				# This means that joint positions very close to [0,0,0,0,0,0] make the code get stuck!			
				if norm(joint_transformation) > 0.1:
					break
			time.sleep(0.01)
		return joint_transformation

	def initialize_safe_movement(self,absolute_transformation,move):
		"""
		The corners of the box are only reached safely if the robot makes a corresponding and also safe change from left-handed to right-handed and from backhand to forhand.
		Since the more the TCP is distant from the wall of the box, the safer the change is, it will be made as soon as the quadrant of the translation of the TCP in the box changes.
		The implemented algorithm automatically handles this functionality, therefore this does not have to be taken into account when planning the scan transformations.
		"""

		old_corner_in_direction = self.corner_in_direction
		old_qnear = self.qnear

		tx,ty,_,_,_,_ = coordinate_transform(absolute_transformation, old_base = "robot base", new_base = "box")
		
		# southeast
		# corner in positive x and y direction
		if tx > 0 and ty > 0:
			self.corner_in_direction = "positive x and y"
			self.qnear = [math.pi/2,-math.pi/2,math.pi/2,math.pi/2,math.pi/2,math.pi]
		
		# northeast
		# corner in positive x and negative y direction
		elif tx > 0 and ty <= 0:
			self.corner_in_direction = "positive x and negative y"
			self.qnear = [math.pi/2,-math.pi/2,math.pi/2,-math.pi/2,-math.pi/2,0]
		
		# southwest
		# corner in negative x and y direction
		elif tx <= 0 and ty <= 0:
			self.corner_in_direction = "negative x and y direction"
			self.qnear = [-math.pi/2,-math.pi/2,-math.pi/2,-math.pi/2,math.pi/2,math.pi]

		# northwest
		# corner in negative x and positive y direction
		else:	
			self.corner_in_direction = "negative x and positive y"
			self.qnear = [-math.pi/2,-math.pi/2,-math.pi/2,math.pi/2,-math.pi/2,0]

		if self.corner_in_direction != old_corner_in_direction and self.initialization == False:
			self.move_robot_joint_transformation(self.get_tcp_transformation(),old_qnear,"l",1e-3)
			self.move_robot_joint_transformation(self.get_tcp_transformation(),self.qnear,"j",1e-3)
			return "l"
		else: 	
			return move
	
	def compare_angles(self,angle1,angle2):
		"""
		Because the inverse kinematics equation converges to the nearest solution, the consideration that the angle of the TCP and the joints are not uniquely defined is important
		"""
		if abs(math.atan(math.tan(((angle1+math.pi)/2)))-math.atan(math.tan(((angle2+math.pi)/2))))*2 < abs(math.atan(math.tan(angle1/2))-math.atan(math.tan(angle2/2)))*2:
			return abs(math.atan(math.tan(((angle1+math.pi)/2)))-math.atan(math.tan(((angle2+math.pi)/2))))*2
		else:	
			return abs(math.atan(math.tan(angle1/2))-math.atan(math.tan(angle2/2)))*2

	def set_tcp(self):
		"""
		To allow the scan area to rotate around a point of interest, it must be possible to set the TCP to a point defined relative to the endeffector.
		"""
		if self.tcp == "scanning area":
			[tx,ty,tz,r,p,y] = self.tool_center_transformation+self.parameters["scanning area relative transformation"]
			rx, ry, rz = rpy_to_rotation_vector([r, p, y])
			self.s.send("set_tcp(p["+str(tx)+", "+\
						 str(ty)+", "+\
						 str(tz)+", "+\
						 str(rx)+", "+\
						 str(ry)+", "+\
						 str(rz)+"])"+"\n")
			# One needs to delay python after sending a command because it can work faster than URSim or UR Polyscope.
			# If one does not delay python and it therefore sends another command too early, the first one is ignored.
			time.sleep(0.1)
		else:
			tx,ty,tz,r,p,y = self.tool_center_transformation
			rx, ry, rz = rpy_to_rotation_vector([r, p, y])
			self.s.send("set_tcp(p"+str([tx,ty,tz,rx,ry,rz])+")"+"\n")
			time.sleep(0.1)

	def close_socket(self):
		"""
    	This function cleanly exits the socket connection
    	"""
		self.s.close()
