#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped, TransformStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import scipy as sp 
from scipy import sparse
import tf2_ros
from path_planner import PathPlanner
from moveit_msgs.msg import OrientationConstraint
from subprocess import call
import intera_interface
import argparse
import cv2
import imutils
from pyimagesearch.shapedetector import ShapeDetector
from pyimagesearch.colorlabeler import ColorLabeler
import argparse

def quaternion_to_r(quaternion, vector=False):
    #tested and works! but having trouble indexing into other stuff, need to figure out if we want to use concatenate instead of hstack
    if vector:
        w=quaternion[3]
        x=quaternion[0]
        y=quaternion[1]
        z=quaternion[2]
    else:
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

def g_to_quaternion(g):
    #Works when tr != 0 (when w!= 0)
    tr = g[0][0]+g[1][1]+g[2][2] + 1

    if tr > 0:
        s = 0.5/np.sqrt(tr)
    else: 
        s = max(g[0][0], g[1][1], g[2][2])
    w = 0.25 / s
    x = (g[2][1] - g[1][2]) * s
    y = (g[0][2] - g[2][0]) * s
    z = (g[1][0] - g[0][1]) * s
    return [x, y, z, w]

def make_g_from_message(message):
    #Not tested yet
    rot = quaternion_to_r(message.transform.rotation)
    print(r)
    translation = translation_to_vector(message.transform.translation)
    Gwc = np.hstack((rot, translation))
    #Gwc = np.concatenate((rot, translation), axis = 1)
    bottom_row = np.array([0, 0, 0, 1])
    Gwc = np.vstack((Gwc, bottom_row))

def make_g_from_components(quaternion, translation):
    #should work now 

    rot = quaternion_to_r(quaternion)

    trans = translation_to_vector(translation)
    
    G = np.array([[rot[0][0], rot[0][1],rot[0][2],trans[0][0]], [rot[1][0], rot[1][1],rot[1][2],trans[1][0]], [rot[2][0], rot[2][1],rot[2][2],trans[2][0]], [0,0,0,1] ])
    print(G)
    # #trans = trans.reshape(trans, (3,1))
    # G = sp.sparse.hstack((rot, trans))
    # #G = np.hstack((rot, translation))

    # bottom_row = np.array([0, 0, 0, 1])
    # G = np.vstack((G, bottom_row))
    return G

def g_to_translation(g):
    return [g[0][3],g[1][3],g[2][3]]

def matrix_to_power(matrix, power):
    #tested and works 
    m = np.eye(3) 
    for i in range(power):
        m = np.dot(matrix, m)
    return m 

def translation_to_vector(translation):
    #returns a 2D ARRAY from a translation message, intended for using to hstack 
    return [[translation.x], [translation.y], [translation.z]]

#DO NOT DELETE, used for testing 
# class quaternion:
# #has to be normalized, w cannot be 0
#   x = np.sqrt(2)/2
#   y = np.sqrt(2)/2
#   z = 0
#   w = 0.002
# class translation: 
#   x = 0
#   y = 0.015
#   z = 0

trans_dictionary = {"C":np.array([.11,0,.0335]), "D":np.array([0.11,-.022, 0.0335]), "E":np.array([0.11,-.044,0.0335]), "F":np.array([0.11,-0.066,0.0335]), "G":np.array([0.11,-0.088,0.0335]), "A":np.array([0.11,-0.11,0.0335]), "B":np.array([0.11,-0.132,0.0335]), "C2":np.array([0.11,-0.154,0.0335])}
# quat = quaternion()
# trans = translation()

# rot = quaternion_to_r(quat)
# translation = translation_to_vector(trans)
# GwcCheck = make_g()
# Gwc = np.hstack((rot, translation))
# bottom_row = np.array([0, 0, 0, 1])
# Gwc = np.vstack((Gwc, bottom_row))
# print(Gwc)


# print(g_to_quaternion(Gwc))
# print(g_to_translation(Gwc))


def get_gwar(target = "base",source = "ar_marker_15"):
#gets gwar from tf
    #rospy.init_node('tf2_listener')
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    boole = False
    while not boole:

        try:
            trans = tfBuffer.lookup_transform(target,source,rospy.Time())
            boole = True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

    # rot = quaternion_to_r(trans.transform.rotation)
    # translation = translation_to_vector(trans.transform.translation)
    # gwar = np.hstack((rot, translation))
    # bottom_row = np.array([0, 0, 0, 1])
    # gwar = np.vstack((gwar, bottom_row))
    # SHOULD BE ABLE TO REPLACE ALL THIS WITH:
    gwar = make_g_from_components(trans.transform.rotation, trans.transform.translation)

    return gwar

def make_tranrot(gwar,garn):
    #returns [[trans.x,trans.trans.z],[rot.x,rot.y,rot.z,rot.w]]
    gwn = np.dot(gwar,garn)
    translation = g_to_translation(gwn)
    rotation = g_to_quaternion(gwn)
    # rotation = [0,0,0,1]

    return [translation,rotation]

def perp_quat(rotation):
    ninety_degree_quat = [np.sqrt(2)/2, -np.sqrt(2)/2,0.0,0.0]
    ninety_degree_g = quaternion_to_r(ninety_degree_quat, vector=True)    
    rotation_g = quaternion_to_r(rotation, vector=True)
    orientation_rotation = np.dot(ninety_degree_g, rotation_g)
    orientation_rotation = np.dot(rotation_g, ninety_degree_g) #not commutative
    return g_to_quaternion(orientation_rotation)


# Four possible moves to make: (1) Moving to surface of new key (2) Pressing key (3) High release of key

def first_move_to_key(translation, rotation): #translation, rotation
    #rospy.init_node('play', anonymous=True)
    #ROTATION WORKS BABYYYY
    #path planning w/obstacle works, so H&J think :/

    # rotation = [np.sqrt(2)/2,-np.sqrt(2)/2,0,0]
    new_rotation = perp_quat(rotation)

    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;
    # rotation = rotate_quat(rotation)
    #translation = [0.696,-0.399,-.22]
    planner = PathPlanner("right_arm")
    planner.remove_obstacle('lower_piano')
    planner.remove_obstacle('upper_piano')
    planner.remove_obstacle('table')
    planner.remove_obstacle('lower_piano_middle')
    planner.remove_obstacle('lower_piano_left')
    planner.remove_obstacle('lower_piano_right')
    planner.remove_obstacle('wall')

    piano_lower = PoseStamped()
    piano_lower.pose.position.x = translation[0] + .015
    piano_lower.pose.position.y = translation[1] + 0.0 
    piano_lower.pose.position.z = translation[2] -.0275 #-0.055

    #print(new_rot_quaternion)
    piano_lower.pose.orientation.x = rotation[0]
    piano_lower.pose.orientation.y = rotation[1]
    piano_lower.pose.orientation.z = rotation[2]
    piano_lower.pose.orientation.w = rotation[3]
    lower_piano_size = np.array([0.06,.5,.055])
    piano_lower.header.frame_id = "base"
    planner.add_box_obstacle(lower_piano_size, "lower_piano", piano_lower)

    #1.5, 10 
    piano_upper = PoseStamped()
    piano_upper.pose.position.x = translation[0] + .1
    piano_upper.pose.position.y = translation[1] + 0.0
    piano_upper.pose.position.z = translation[2] + .01 #0.01 -.02
    piano_upper.pose.orientation.x = rotation[0]
    piano_upper.pose.orientation.y = rotation[1]
    piano_upper.pose.orientation.z = rotation[2]
    piano_upper.pose.orientation.w = rotation[3]
    upper_piano_size = np.array([.14, .5, .03])
    piano_upper.header.frame_id = "base"

    planner.add_box_obstacle(upper_piano_size, "upper_piano", piano_upper)

    wall = PoseStamped()
    wall.pose.position.x = -.7
    wall.pose.position.y = 0.0
    wall.pose.position.z = 0.5
    wall.pose.orientation.x = 0.0
    wall.pose.orientation.y = 0.0
    wall.pose.orientation.z = 0.0
    wall.pose.orientation.w = 1.0
    wall_size = np.array([.01, 2, 1.4])
    wall.header.frame_id = "base"

    # planner.add_box_obstacle(wall_size, "wall", wall)

    table = PoseStamped()
    table.pose.position.x = translation[0] - 0.1
    table.pose.position.y = translation[1] + 0.0
    table.pose.position.z = translation[2] - 0.04
    table.pose.orientation.x = rotation[0]
    table.pose.orientation.y = rotation[1]
    table.pose.orientation.z = rotation[2]
    table.pose.orientation.w = rotation[3]
    table_size = np.array([.3, .7, 0.02])
    table.header.frame_id = "base"

    planner.add_box_obstacle(table_size, "table", table)


    while not rospy.is_shutdown():
        try:
            goal = PoseStamped()
            goal.header.frame_id = "base"

            #x, y, and z position
            goal.pose.position.x = translation[0]
            goal.pose.position.y = translation[1]
            goal.pose.position.z = translation[2] + 0.011 # did + 0.01 because it thinks gripper tip is lower than it actually is

            #Orientation as a quaternion
            goal.pose.orientation.x = new_rotation[0]
            goal.pose.orientation.y = new_rotation[1]
            goal.pose.orientation.z = new_rotation[2]
            goal.pose.orientation.w = new_rotation[3]
            print(new_rotation)

            plan = planner.plan_to_pose(goal, list())

            raw_input("Press <Enter> to move the right arm to move to key: ")
            if not planner.execute_plan(plan):
                raise Exception("Execution failed")
        except Exception as e:
            print e
        else:
            break

def move_to_key(translation, rotation): #translation, rotation
    #rospy.init_node('play', anonymous=True)
    #ROTATION WORKS BABYYYY
    #path planning w/obstacle works, so H&J think :/

    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;

    # rotation = [np.sqrt(2)/2,-np.sqrt(2)/2,0,0]
    new_rotation = perp_quat(rotation)
    # rotation = rotate_quat(rotation)
    #translation = [0.696,-0.399,-.22]
    planner = PathPlanner("right_arm")
    planner.remove_obstacle('lower_piano')
    planner.remove_obstacle('lower_piano_middle')
    planner.remove_obstacle('lower_piano_left')
    planner.remove_obstacle('lower_piano_right')

    piano_lower = PoseStamped()
    piano_lower.pose.position.x = translation[0] + .015
    piano_lower.pose.position.y = translation[1] + 0.0 
    piano_lower.pose.position.z = translation[2] -.0275 #-0.055

    #print(new_rot_quaternion)
    piano_lower.pose.orientation.x = rotation[0]
    piano_lower.pose.orientation.y = rotation[1]
    piano_lower.pose.orientation.z = rotation[2]
    piano_lower.pose.orientation.w = rotation[3]
    lower_piano_size = np.array([0.06,.5,.055])
    piano_lower.header.frame_id = "base"
    planner.add_box_obstacle(lower_piano_size, "lower_piano", piano_lower)

    #1.5, 10 
    piano_upper = PoseStamped()
    piano_upper.pose.position.x = translation[0] + .1
    piano_upper.pose.position.y = translation[1] + 0.0
    piano_upper.pose.position.z = translation[2] + .01 #0.01 -.02
    piano_upper.pose.orientation.x = rotation[0]
    piano_upper.pose.orientation.y = rotation[1]
    piano_upper.pose.orientation.z = rotation[2]
    piano_upper.pose.orientation.w = rotation[3]
    upper_piano_size = np.array([.14, .5, .03])
    piano_upper.header.frame_id = "base"

    planner.add_box_obstacle(upper_piano_size, "upper_piano", piano_upper)

    wall = PoseStamped()
    wall.pose.position.x = -.7
    wall.pose.position.y = 0.0
    wall.pose.position.z = 0.5
    wall.pose.orientation.x = 0.0
    wall.pose.orientation.y = 0.0
    wall.pose.orientation.z = 0.0
    wall.pose.orientation.w = 1.0
    wall_size = np.array([.01, 2, 1.4])
    wall.header.frame_id = "base"

    # planner.add_box_obstacle(wall_size, "wall", wall)

    table = PoseStamped()
    table.pose.position.x = translation[0] - 0.1
    table.pose.position.y = translation[1] + 0.0
    table.pose.position.z = translation[2] - 0.04
    table.pose.orientation.x = rotation[0]
    table.pose.orientation.y = rotation[1]
    table.pose.orientation.z = rotation[2]
    table.pose.orientation.w = rotation[3]
    table_size = np.array([.3, .7, 0.02])
    table.header.frame_id = "base"

    planner.add_box_obstacle(table_size, "table", table)


    while not rospy.is_shutdown():
        try:
            goal = PoseStamped()
            goal.header.frame_id = "base"

            #x, y, and z position
            goal.pose.position.x = translation[0]
            goal.pose.position.y = translation[1]
            goal.pose.position.z = translation[2] + 0.011 # did + 0.01 because it thinks gripper tip is lower than it actually is

            #Orientation as a quaternion
            goal.pose.orientation.x = new_rotation[0]
            goal.pose.orientation.y = new_rotation[1]
            goal.pose.orientation.z = new_rotation[2]
            goal.pose.orientation.w = new_rotation[3]

            plan = planner.plan_to_pose(goal, [orien_const])

            #raw_input("Press <Enter> to move the right arm to move to key: ")
            if not planner.execute_plan(plan):
                raise Exception("Execution failed")
        except Exception as e:
            print e
        else:
            break

def press_key(translation, rotation):
    #rospy.init_node('play', anonymous=True)

    new_rotation = perp_quat(rotation)

    planner = PathPlanner("right_arm")
    planner.remove_obstacle('lower_piano')
    # planner.remove_obstacle('upper_piano')
    # planner.remove_obstacle('table')

    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;

    piano_middle = PoseStamped()
    piano_middle.pose.position.x = translation[0] + .015
    piano_middle.pose.position.y = translation[1] + 0.0
    piano_middle.pose.position.z = translation[2] + -.03
    piano_middle.pose.orientation.x = rotation[0]
    piano_middle.pose.orientation.y = rotation[1]
    piano_middle.pose.orientation.z = rotation[2]
    piano_middle.pose.orientation.w = rotation[3]
    lower_piano_middle_size = np.array([0.06,.04,.02])
    piano_middle.header.frame_id = "base"

    planner.add_box_obstacle(lower_piano_middle_size, "lower_piano_middle", piano_middle)

    piano_left = PoseStamped()
    piano_left.pose.position.x = translation[0] + .015
    piano_left.pose.position.y = translation[1] - 0.03
    piano_left.pose.position.z = translation[2] + -.0275
    piano_left.pose.orientation.x = rotation[0]
    piano_left.pose.orientation.y = rotation[1]
    piano_left.pose.orientation.z = rotation[2]
    piano_left.pose.orientation.w = rotation[3]
    lower_piano_left_size = np.array([0.06,.02,.07]) #0.055
    piano_left.header.frame_id = "base"

    planner.add_box_obstacle(lower_piano_left_size, "lower_piano_left", piano_left)

    piano_right = PoseStamped()
    piano_right.pose.position.x = translation[0] + .015
    piano_right.pose.position.y = translation[1] + 0.03 # 0.015
    piano_right.pose.position.z = translation[2] + -.0275
    piano_right.pose.orientation.x = rotation[0]
    piano_right.pose.orientation.y = rotation[1]
    piano_right.pose.orientation.z = rotation[2]
    piano_right.pose.orientation.w = rotation[3]
    lower_piano_right_size = np.array([0.06,.02,.07]) #0.11
    piano_right.header.frame_id = "base"

    planner.add_box_obstacle(lower_piano_right_size, "lower_piano_right", piano_right) 

    # NOT NECESSARY TO HAVE UPPER PIANO, TABLE????????????

    # 1.5, 10 control
    piano_upper = PoseStamped()
    piano_upper.pose.position.x = translation[0] + .1
    piano_upper.pose.position.y = translation[1] + 0.0
    piano_upper.pose.position.z = translation[2] + .01
    piano_upper.pose.orientation.x = rotation[0]
    piano_upper.pose.orientation.y = rotation[1]
    piano_upper.pose.orientation.z = rotation[2]
    piano_upper.pose.orientation.w = rotation[3]
    upper_piano_size = np.array([.14, .5, .02])
    piano_upper.header.frame_id = "base"

    # planner.add_box_obstacle(upper_piano_size, "upper_piano", piano_upper)

    table = PoseStamped()
    table.pose.position.x = translation[0] + 0.0
    table.pose.position.y = translation[1] + 0.0
    table.pose.position.z = translation[2] + -.105
    table.pose.orientation.x = rotation[0]
    table.pose.orientation.y = rotation[1]
    table.pose.orientation.z = rotation[2]
    table.pose.orientation.w = rotation[3]
    table_size = np.array([.5, .5, .1])
    table.header.frame_id = "base"

    # planner.add_box_obstacle(table_size, "table", table)


    while not rospy.is_shutdown():
        try:
            goal = PoseStamped()
            goal.header.frame_id = "base"

            #x, y, and z position
            goal.pose.position.x = translation[0]
            goal.pose.position.y = translation[1]
            goal.pose.position.z = translation[2] -0.01

            #Orientation as a quaternion
            goal.pose.orientation.x = new_rotation[0]
            goal.pose.orientation.y = new_rotation[1]
            goal.pose.orientation.z = new_rotation[2]
            goal.pose.orientation.w = new_rotation[3]

            plan = planner.plan_to_pose(goal, [orien_const])

            #raw_input("Press <Enter> to move the right arm to press key: ")
            if not planner.execute_plan(plan):
                raise Exception("Execution failed")
        except Exception as e:
            print e
        else:
            break   


def release_key(translation, rotation):
    #rospy.init_node('play', anonymous=True)
    #CHANGE CONSTRAINTS HERE
    #rotation = [np.sqrt(2)/2,-np.sqrt(2)/2,0,0]
    # translation = [0.696,-0.399,-.14]

    planner = PathPlanner("right_arm")
    # planner.remove_obstacle('lower_piano')
    # planner.remove_obstacle('upper_piano')
    # planner.remove_obstacle('table')
    planner.remove_obstacle('lower_piano_middle')
    # planner.remove_obstacle('lower_piano_left')
    # planner.remove_obstacle('lower_piano_right')
    new_rotation = perp_quat(rotation)

    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;

    piano_middle = PoseStamped()
    piano_middle.pose.position.x = translation[0] + .015
    piano_middle.pose.position.y = translation[1] + 0.0
    piano_middle.pose.position.z = translation[2] + -.025
    piano_middle.pose.orientation.x = rotation[0]
    piano_middle.pose.orientation.y = rotation[1]
    piano_middle.pose.orientation.z = rotation[2]
    piano_middle.pose.orientation.w = rotation[3]
    lower_piano_middle_size = np.array([0.04,.02,.02])
    piano_middle.header.frame_id = "base"

    # planner.add_box_obstacle(lower_piano_middle_size, "lower_piano_middle", piano_middle)

    piano_left = PoseStamped()
    piano_left.pose.position.x = translation[0] + .015
    piano_left.pose.position.y = translation[1] - 0.02
    piano_left.pose.position.z = translation[2] + -.055
    piano_left.pose.orientation.x = rotation[0]
    piano_left.pose.orientation.y = rotation[1]
    piano_left.pose.orientation.z = rotation[2]
    piano_left.pose.orientation.w = rotation[3]
    lower_piano_left_size = np.array([0.04,.02,.13]) #0.11
    piano_left.header.frame_id = "base"

    # planner.add_box_obstacle(lower_piano_left_size, "lower_piano_left", piano_left)

    piano_right = PoseStamped()
    piano_right.pose.position.x = translation[0] + .015
    piano_right.pose.position.y = translation[1] + 0.02
    piano_right.pose.position.z = translation[2] + -.055
    piano_right.pose.orientation.x = rotation[0]
    piano_right.pose.orientation.y = rotation[1]
    piano_right.pose.orientation.z = rotation[2]
    piano_right.pose.orientation.w = rotation[3]
    lower_piano_right_size = np.array([0.04,.02,.13]) #0.11
    piano_right.header.frame_id = "base"

    # planner.add_box_obstacle(lower_piano_right_size, "lower_piano_right", piano_right) 
# e - success 1 time??
        # key = "E"
        # translation = trans_dictionary[key]
        # print("made it to getting gwar")
        # garn = np.array([[1, 0,0,translation[0]],[0, 1, 0, translation[1]], [0, 0, 1, translation[2]], [0,0,0,1]])
        # print(garn)
        # tranrot = make_tranrot(gwar,garn)
        # print(tranrot)
        # move_to_key(tranrot[0],tranrot[1])
        # press_key(tranrot[0],tranrot[1])
        # release_key(tranrot[0],tranrot[1])


        # #onto 3rd note
        # key = "C"
        # translation = trans_dictionary[key]
        # print("made it to getting gwar")
        # garn = np.array([[1, 0,0,translation[0]],[0, 1, 0, translation[1]], [0, 0, 1, translation[2]], [0,0,0,1]])
        # print(garn)
        # tranrot = make_tranrot(gwar,garn)
        # print(tranrot)
        # move_to_key(tranrot[0],tranrot[1])
        # press_key(tranrot[0],tranrot[1])
        # release_key(tran)
    # NOT NECESSARY TO HAVE UPPER PIANO, TABLE????????????

    # 1.5, 10 control
    piano_upper = PoseStamped()
    piano_upper.pose.position.x = translation[0] + .1
    piano_upper.pose.position.y = translation[1] + 0.0
    piano_upper.pose.position.z = translation[2] + .01
    piano_upper.pose.orientation.x = rotation[0]
    piano_upper.pose.orientation.y = rotation[1]
    piano_upper.pose.orientation.z = rotation[2]
    piano_upper.pose.orientation.w = rotation[3]
    upper_piano_size = np.array([.14, .1, .02])
    piano_upper.header.frame_id = "base"

    # planner.add_box_obstacle(upper_piano_size, "upper_piano", piano_upper)

    table = PoseStamped()
    table.pose.position.x = translation[0] + 0.0
    table.pose.position.y = translation[1] + 0.0
    table.pose.position.z = translation[2] + -.105
    table.pose.orientation.x = rotation[0]
    table.pose.orientation.y = rotation[1]
    table.pose.orientation.z = rotation[2]
    table.pose.orientation.w = rotation[3]
    table_size = np.array([.5, .1, .1])
    table.header.frame_id = "base"

    # planner.add_box_obstacle(table_size, "table", table)


    while not rospy.is_shutdown():
        try:
            goal = PoseStamped()
            goal.header.frame_id = "base"

            #x, y, and z position
            goal.pose.position.x = translation[0]
            goal.pose.position.y = translation[1]
            goal.pose.position.z = translation[2] + 0.01

            #Orientation as a quaternion
            goal.pose.orientation.x = new_rotation[0]
            goal.pose.orientation.y = new_rotation[1]
            goal.pose.orientation.z = new_rotation[2]
            goal.pose.orientation.w = new_rotation[3]

            plan = planner.plan_to_pose(goal, [orien_const])

            # raw_input("Press <Enter> to move the right arm to release key: ")
            if not planner.execute_plan(plan):
                raise Exception("Execution failed")
        except Exception as e:
            print e
        else:
            break


# Call this function to play the key
def play_key(key, prev_key, gwar):
    translation = trans_dictionary[key]
    print(key)
    garn = np.array([[1, 0,0,translation[0]],[0, 1, 0, translation[1]], [0, 0, 1, translation[2]], [0,0,0,1]])
    tranrot = make_tranrot(gwar,garn)
    # terminal commands, echo prints to terminal, rosrun shows image
    # head_display.display_image(key + ".png")
    call(["echo", "Key: " + key])
    if key != prev_key:
        # play_key("D", gwar)
        move_to_key(tranrot[0],tranrot[1])
    call("rosrun intera_examples head_display_image.py -f " + key + ".png", shell=True)
    press_key(tranrot[0],tranrot[1])
    release_key(tranrot[0],tranrot[1])

def play_first_key(key, gwar):
    translation = trans_dictionary[key]
    print(key)
    garn = np.array([[1, 0,0,translation[0]],[0, 1, 0, translation[1]], [0, 0, 1, translation[2]], [0,0,0,1]])
    tranrot = make_tranrot(gwar,garn)
    cmd = 'echo ' + key
    # head_display.display_image("/ros_workspaces/bachxter/src/piano_player/src/" + key + ".png")
    call(["echo", "Key: " + key])
    first_move_to_key(tranrot[0], tranrot[1])
    call("rosrun intera_examples head_display_image.py -f " + key + ".png", shell=True)
    press_key(tranrot[0],tranrot[1])
    release_key(tranrot[0],tranrot[1])

#sorting function to sort shapes on sheet music from top left to bottom right
def greater(a):
    momA = cv2.moments(a)
    print(momA)
    if momA['m00'] != 0: 
        (xa,ya) = int(momA['m10']/momA['m00']) * ratio, int(momA['m01']/momA['m00']) * ratio
        return xa + 10*ya
    return 100000000


# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method
    notes = []
    note_dictionary = {"square":"C", "triangle":"D", "rectangle":"E", "pentagon":"F", "hexagon":"G", "septagon":"A", "octagon":"B"}
    cam = cv2.VideoCapture(0)

    cv2.namedWindow("test")

    img_counter = 0
    img_name = ""

    while True:
        ret, frame = cam.read()
        cv2.imshow("test", frame)
        if not ret:
            break
        k = cv2.waitKey(1)

        if k%256 == 27:
            # ESC pressed
            print("Escape hit, closing...")
            break
        elif k%256 == 32:
            # SPACE pressed
            img_name = "opencv_frame_{}.png".format(img_counter)
            cv2.imwrite(img_name, frame)
            print("{} written!".format(img_name))
            img_counter += 1

    cam.release()

    cv2.destroyAllWindows()

    # load the image and resize it to a smaller factor so that
    # the shapes can be approximated better
    image = cv2.imread(img_name)
    print(img_name)
    resized = imutils.resize(image, width=400)
    ratio = image.shape[0] / float(resized.shape[0])

    # blur the resized image slightly, then convert it to both
    # grayscale and the L*a*b* color spaces
    blurred = cv2.GaussianBlur(resized, (5, 5), 0)
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
    thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY)[1]
    cv2.imshow("Thresh", thresh)

    # find contours in the thresholded image
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    cnts.sort(key=greater)
    #cnts_sorted = sorted(cnts, key=lambda item: centroid_calc(item))
    #print(cnts_sorted)

    # initialize the shape detector and color labeler
    sd = ShapeDetector()
    cl = ColorLabeler()

    # loop over the contours
    for c in cnts:
        # compute the center of the contour
        M = cv2.moments(c)
        cX = int((M["m10"] / M["m00"]) * ratio)
        cY = int((M["m01"] / M["m00"]) * ratio)
        #printCountours(c)

        # detect the shape of the contour and label the color
        shape = sd.detect(c)
        color = cl.label(lab, c)

        # multiply the contour (x, y)-coordinates by the resize ratio,
        # then draw the contours and the name of the shape and labeled
        # color on the image
        c = c.astype("float")
        c *= ratio
        c = c.astype("int")
        # text = "{} {}".format(color, shape)
        text = shape
        cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
        cv2.putText(image, text, (cX, cY),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        # print(color + " " + shape)
        print(shape)
        notes.append(shape)

        # show the output image
        cv2.imshow("Image", image)
        cv2.waitKey(1000)
        # k = cv2.waitKey(1)

        # if k%256 == 27:
        #     # ESC pressed
        #     print("Escape hit, closing...")
        #     break

    rospy.init_node('play')
    print("made it here")
    print(notes)
    music = [note_dictionary[note] for note in notes]
    print(music)
    # head_display = intera_interface.HeadDisplay()
    try:
        # key = "C"
        gwar = get_gwar()
        call("rosrun intera_examples head_display_image.py -f bachbig.jpg", shell=True)
        first_key = note_dictionary[notes[0]]
        play_first_key(first_key, gwar)
        # prev_key = first_key
        # if len(notes) > 1:
        #     for note in notes[1:]:
        #         key = note_dictionary[note]
        #         play_key(key, prev_key, gwar)
        #         prev_key = key
        call("rosrun intera_examples head_display_image.py -f bachbig.jpg", shell=True)

        # music = raw_input().split()
        # gwar = get_gwar()
        # call("rosrun intera_examples head_display_image.py -f bachbig.jpg", shell=True)
        # play_first_key(music[0], gwar)
        # if len(music) > 1:
        #     for note in music[1:]:
        #         play_key(note, gwar)
        # call("rosrun intera_examples head_display_image.py -f bachbig.jpg", shell=True)



    except rospy.ROSInterruptException: 
        print("ROSInterruptException")
        pass


