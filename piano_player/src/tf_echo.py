#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

import tf2_ros
import rospy
from geometry_msgs.msg import TransformStamped
import numpy as np

def translation_to_vector(translation):
  #returns a 2D ARRAY from a translation, intended for using to hstack 
  return np.array([[translation.x], [translation.y], [translation.z]])

def quaternion_to_r(quaternion):
  #tested and works! but having trouble indexing into other stuff, need to figure out if we want to use concatenate instead of hstack 

  
  w = quaternion.w 
  x = quaternion.x
  y = quaternion.y
  z = quaternion.z
  vector = np.array([x, y, z, w])
  norm = np.linalg.norm(vector)
  w /= norm 
  x /= norm
  y /= norm
  z /= norm 


  col1 = np.array([[(1-2*(y**2 + z**2))], [(2*(x*y+z*w))], [(2*(x*z-y*w))]])
  col2 = np.array([[(2*(x*y-z*w))], [(1-2*(x**2 + z**2))], [(2*(y*z+x*w))]])
  col3 = np.array([[(2*(x*z+y*w))], [(2*(y*z-x*w))], [(1-2*(x**2 + y**2))]])

  r = np.hstack((col1, col2))
  r = np.hstack((r, col3))
  return r 

def listener(target = "base", source = "ar_marker_14"):
	# frame = [x for x in raw_input('target (base),source (AR): ').split(',')]

	# rospy.init_node('tf2_listener')
	# tfBuffer = tf2_ros.Buffer()
	# tfListener = tf2_ros.TransformListener(tfBuffer)
	# boole = False
	# while not boole:

	# 	try:
	# 		trans = tfBuffer.lookup_transform(target,source,rospy.Time())
	# 		boole = True

	# 	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
	# 		continue
	# gx = trans.transform.translation.x
	# gy = trans.transform.translation.y
	# gz = trans.transform.translation.z

	# print(gx, gy, gz)

	rospy.init_node('tf2_listener')
	tfBuffer = tf2_ros.Buffer()
	tfListener = tf2_ros.TransformListener(tfBuffer)
	boole = False
	while not boole:

		try:
			trans = tfBuffer.lookup_transform(target,source,rospy.Time())
			boole = True

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			continue

	rot = quaternion_to_r(trans.transform.rotation)
	translation = translation_to_vector(trans.transform.translation)
	Gwc = np.hstack((rot, translation))
	bottom_row = np.array([0, 0, 0, 1])
	Gwc = np.vstack((Gwc, bottom_row))

	print(Gwc)



if __name__ == '__main__':
    listener()
