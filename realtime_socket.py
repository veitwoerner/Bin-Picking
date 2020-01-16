#!/usr/bin/env python2

"""
Methods to read UR real time socket.
Wrapper should execute method 'getParams' with 125 Hz.
"""

import select
import socket
import struct
import time

from load_parameters_code import load_parameters

class UrSocketClient():

	def __init__(self, parser):

		parameters = load_parameters()

		## Arguments related to real time socket
		host = parameters["host"]
		parser.add_argument("--ip", type=str, default=host, help="Robot ip address. Default defined in load_parameters_code")

		## Mr non-blocking socket
		self._rtSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self._rtSocket.setblocking(0)

	def connectSocket(self, args):
		"""
		Real time socket is on port 30003
		Connect to this socket
		"""
		print("Socket connection status", self._rtSocket.connect_ex((args.ip, 30003)))
		time.sleep(0.1)

	def _recv8Bytes(self):
		"""
		Reads and decode 8 bytes from socket.
		:return: 8 bytes decoded.
		:rtype: <str>, <int>, <float>, ...
		"""
		try:
			packet = self._rtSocket.recv(8)
			return struct.unpack('!d', packet)[0]
		except:
			return None

	def _getMsgSize(self):
		"""
		Retruns msg size
		:return <int>: msg size
		"""
		try:
			return int(self._rtSocket.recv(4).hex(),16)
		except:
			return None

	def _getVector(self):
		"""
		Reads 6*8 bytes from socket.
		Used for poses and joint configurations.
		:return: the vector with the 6 values
		:rtype: <str>, <int>, <float>, ...
		"""
		try:
			return [self._recv8Bytes() for x in [0]*6]
		except:
			return None

	def _getArrayBits(self):
		"""
		Reads 8 bytes from socket and translates into array of bits.
		Used for digital inputs/outputs.
		:return [<int>]: an array of 8 bits, one byte.
		"""
		try:
			return [int(x) for x in bin(int(self._recv8Bytes()))[2:].zfill(8)]
		except:
			return None

	def _getRobotMode(self):
		"""
		Gets current robot mode.
		:return <str>: the current robot mode.
		"""
		modeDict ={
		0: "ROBOT_MODE_DISCONNECTED",
		1: "ROBOT_MODE_CONFIRM_SAFETY",
		2: "ROBOT_MODE_BOOTING",
		3: "ROBOT_MODE_POWER_OFF",
		4: "ROBOT_MODE_POWER_ON",
		5: "ROBOT_MODE_IDLE",
		6: "ROBOT_MODE_BACKDRIVE",
		7: "ROBOT_MODE_RUNNING",
		8: "ROBOT_MODE_UPDATING_FIRMWARE"
		}
		try:
			return modeDict.get(int(self._recv8Bytes()))
		except:
			return None

	def _getSafetyMode(self):
		"""
		Gets current safety mode of robot.
		:return <str>: the current safety mode.
		"""
		modeDict ={
		1: "SAFETY_MODE_NORMAL",
		2: "SAFETY_MODE_REDUCED",
		3: "SAFETY_MODE_PROTECTIVE_STOP",
		4: "SAFETY_MODE_RECOVERY",
		5: "SAFETY_MODE_SAFEGUARD_STOP",
		6: "SAFETY_MODE_SYSTEM_EMERGENCY_STOP",
		7: "SAFETY_MODE_ROBOT_EMERGENCY_STOP",
		8: "SAFETY_MODE_VIOLATION",
		9: "SAFETY_MODE_FAULT"
		}
		try:
			return modeDict.get(int(self._recv8Bytes()))
		except:
			return None

	def _getBusyState(self):
		"""
		Returns a boolen to tell if the robot is busy or not.
		:return <bool>: True if robot is busy else False
		"""
		try:
			return not int(self._recv8Bytes()) == 1
		except:
			return None

	def _ignore(self, bytes):
		"""
		Reads 'bytes' bytes from socket.
		:param <int> bytes: bytes to read.
		"""
		try:
			self._rtSocket.recv(bytes)
		except:
			return None

	def getParams(self, timeout = .1):
		"""
		Reads socket.
		1108 bytes. UR control system version/Zacobria >3.5
		Some of the parameters are put into dictionary. Rest are ignored.
		:return <dict>: interesting parameters
		See https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/remote-control-via-tcpip-16496/
		"""

		robotData = {}

		try:
			ready = select.select([self._rtSocket], [], [], timeout)
			if ready[0]:
				robotData['msgSize'] = self._getMsgSize()
				robotData['timeStamp'] = self._recv8Bytes()
				self._ignore(48*5)
				robotData['jointPosition'] = self._getVector()
				robotData['jointVelocity'] = self._getVector()
				robotData['jointCurrent'] = self._getVector()
				self._ignore(48)
				robotData['tcpPose'] = self._getVector()
				robotData['tcpVelocity'] = self._getVector()
				robotData['tcpWrench'] = self._getVector()
				self._ignore(48*2)
				robotData['digitalInputs'] = self._getArrayBits()
				self._ignore(48+8+8)
				robotData['robotMode'] = self._getRobotMode()
				self._ignore(48)
				robotData['safetyMode'] = self._getSafetyMode()
				self._ignore(48+24+48+8*7+48)
				robotData['digitalOutputs'] = self._getArrayBits()
				robotData['busyState'] = self._getBusyState()
				self._ignore(24+24)
		except:
			pass

		return robotData

	def send(self, script):
		"""
		Sends a script on socket.
		:script <str> or <bytes>: script to send
		:return <bool>: True if ok else False
		'script' of type <bytes> needs to end with new line ('\n')

		"""
		try:
		# make sure new line and byte encoding
			if isinstance(script, str):
				if not script.endswith('\n'):
					script = script + '\n'
					script = str.encode(script)
					# send
					#            print("script", script)
					self._rtSocket.send(script)
					return True
		except Exception as e:
			print("rt socket send exception", e)
			return False

	def __del__(self):
		#        print("__del__ rt socket")
		try:
			#            self.send("stopj(1)") # halt robot
			self._rtSocket.close()
		except:
			pass
