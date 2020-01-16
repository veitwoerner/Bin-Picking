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
__maintainer__ = "Daniel Jakobson"
__email__ = "danjako@student.chalmers.se"
__status__ = "Production"

import requests
import time

def scan(port,ip = 'localhost'):
	"""
	The 3D scanner software performs a scan on request, saves the point cloud and returns the scan ID.
	"""
	try:
		r = requests.post("http://"+ip+":" + str(port) + "/", json={"format": ["praw"]})
	
		scan_id = str(r.json()['id'][0]).split('.')[0]
		return scan_id
	except:
		return None

def localize(scan_id, port, plcfFile, timeout, overlap, maxNbrOfResults,ip = 'localhost'):
	"""
	Using this point cloud, a request can be sent to the localization software, which returns some results.
	"""
	try:
		l = requests.post("http://"+ip+":" + str(port) + "/locate/",json={"scanId": scan_id, "plcfFile": plcfFile,"timeout" : timeout,"overlap" : overlap,"maxNbrOfResults" : maxNbrOfResults})
		return l.json()
	except:
		return None


