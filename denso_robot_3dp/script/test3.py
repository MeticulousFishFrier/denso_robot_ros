#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Point
from pilz_robot_programming import *
import math
import rospy
import numpy as np
import os
import sys
import moveit_commander
import tf
from time import sleep
__REQUIRED_API_VERSION__ = "1"
_GRIPPER_CLOSED = 0.0015 # includes brackets
_GRIPPER_OPEN = 0.02
if __name__ == '__main__':
    rospy.init_node('robot_program_node')

    # begins the robot controller
    r = Robot(__REQUIRED_API_VERSION__)


    z_pos = np.linspace(.4,.2,2)
    z_pos = np.round(z_pos,decimals=3)
    sequence = Sequence()
    for z in z_pos:
        sequence.append(Lin(goal=Pose(position=Point(0.4, 0.1, z)), vel_scale=1, acc_scale = .2 ))

    sequence.append(Extruder(goal=.1))
    r.move(sequence)

    # r.move(Extruder(goal=.1) )