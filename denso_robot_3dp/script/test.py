#!/usr/bin/env python

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray

from pilz_robot_programming import *
import math
import rospy
import numpy as np
import os
import sys
import moveit_commander
import tf
from time import sleep
from printrun.printcore import printcore
from printrun import gcoder
import time
from threading import Thread

def callback(data):
    #print("i got my data", data)
    x = 0
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/tcp', rospy.Time(0))
            ext_position = data.poses[x].position

            ext_position_arr = np.array([ext_position.x, ext_position.y ,  ext_position.z])

            rounded_trans_arr = np.round(list(trans), decimals=6)

            #print("trans:",rounded_trans_arr)
            #print("ext_tupl:",ext_position_arr)
            if (np.allclose(ext_position_arr, rounded_trans_arr, atol=0.0001,rtol=0)):
                print(x)
                x = x + 1
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        

if __name__ == '__main__':
    rospy.init_node('extruder_node')

    listener = tf.TransformListener()

    rospy.Subscriber("move_ext_poses", PoseArray, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()