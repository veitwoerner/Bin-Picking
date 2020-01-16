#!/usr/bin/env python2

"""
Bridge between robot real time socket and ROS TF for robot joint state.
This script publishes joint states on topic "joint_states" and TCP states on topic "tcp_states"

These joint states are translated to tf by running:
rosrun robot_state_publisher robot_state_publisher
"""

import argparse
import threading
import time
import rospy
import sensor_msgs.msg
from realtime_socket import UrSocketClient
from project.msg import TcpState

class RosRobotListener(UrSocketClient):

    def __init__(self):
        ## SETUP ---------------------------------------------------
        ## args
        parser = argparse.ArgumentParser(description="Bridge between robot rt socket and ROS TF for robot joint state")

        ## Init ros node
        rospy.init_node("robot_listener")

        ## Ros topics
        self._jointStatesPub = rospy.Publisher("joint_states", sensor_msgs.msg.JointState, queue_size=1)
	self._tcpStatesPub = rospy.Publisher("tcp_states", TcpState, queue_size=1)

        ## Setup rtsocket
        self._urSocketClient = UrSocketClient
        self._urSocketClient.__init__(self, parser)

        ## Get arguments
        args = parser.parse_args()
	rospy.loginfo(args)

        ## Connect rtsocket
        self.connectSocket(args)

        ## Variables
        # To store what has been polled from socket
        self._robotData = {}

        threading.Thread(target=self._readingRtSocket).start()
        threading.Thread(target=self._publishingRobotState).start()

        rospy.on_shutdown(self._onShutdown)

        rospy.spin()

    def _readingRtSocket(self):
        """Reads socket until stopped"""
        while not rospy.is_shutdown():
            self._robotData = self.getParams()

    def _publishingRobotState(self):
        while not rospy.is_shutdown():
            # Publish actual robot joint state
            joint_state = sensor_msgs.msg.JointState()
            joint_state.name = [
                'shoulder_pan_joint'
                ,'shoulder_lift_joint'
                ,'elbow_joint'
                ,'wrist_1_joint'
                ,'wrist_2_joint'
                ,'wrist_3_joint'
                ]
            joint_state.header.stamp = rospy.Time.now()
            joint_state.position = self._robotData.get('jointPosition', [0]*6)
            joint_state.velocity = self._robotData.get('jointVelocity', [0]*6)
            joint_state.effort = self._robotData.get('jointCurrent', [0]*6)

	    # Publish actual robot tcp state
            tcp_state = TcpState()
            tcp_state.header.stamp = rospy.Time.now()
            tcp_state.pose = self._robotData.get('tcpPose', [0]*6)
	    tcp_state.velocity = self._robotData.get('tcpVelocity', [0]*6)
            tcp_state.wrench = self._robotData.get('tcpWrench', [0]*6)
            
            try:
                self._jointStatesPub.publish(joint_state)
		self._tcpStatesPub.publish(tcp_state)
            except:
                pass
            # Wait
            rospy.sleep(1/20.0)

    def _onShutdown(self):
        try:
            self._rtSocket.close()
        except:
            pass

# Here is the main entry point
if __name__ == '__main__':
    RosRobotListener()
