#!/usr/bin/env python
from geometry_msgs.msg import Point
from pilz_robot_programming import *
import math
import rospy
import numpy as np
import sys

__REQUIRED_API_VERSION__ = "1"
Q_SIZE = 100

def start_program():
    print("Executing " + __file__)

    r = Robot(__REQUIRED_API_VERSION__)

    # Simple ptp movement
    #r.move(Ptp(goal=[-.4, 0, 1.66, 3.45, -1, -3.71], vel_scale=1, acc_scale = .1))
    r.move(Ptp(goal=Pose(position=Point(0.2, 0.1, 0.20), orientation = Quaternion( 0, 1, 0, 0 )), vel_scale=.1, acc_scale = .1 ))
    

    #r.move(Lin(goal=Pose(position=Point(0.390638, 0.009637, 0.20025)), vel_scale=1, acc_scale = .2 ))
    # z_pos = np.linspace(.4,.2,100)
    # z_pos = np.round(z_pos,decimals=3)


    # for z in z_pos:
    #     sequence.append(Lin(goal=Pose(position=Point(0.4, 0.1, z)), vel_scale=1, acc_scale = .2 ))


    # r.move(sequence)

    # sequence_action_goal = sequence._get_sequence_request(r)

    # _planning_options = PlanningOptions()
    # _planning_options.planning_scene_diff.robot_state.is_diff = True

    # sequence_action_goal.planning_options = _planning_options

    #state machine indicates MOTION_RESUMED is the integer 5
    #4 is RESUME_REQUESTED



    # # automatic transition from STOP_REQUESTED to NO_REQUEST when move is called
    # if r._move_ctrl_sm.state == 2:
    #     r._move_ctrl_sm.switch(4)

    # # automatic transition from RESUME_REQUESTED to NO_REQUEST when move is called
    # if r._move_ctrl_sm.state == 4:
    #     r._move_ctrl_sm.switch(5)

    # try:
    #     r._move_execution_loop(cmd)
    # finally:
    #     r._move_lock.release()

    # r._sequence_client.send_goal(sequence_action_goal)
    # r._move_lock.release()

if __name__ == "__main__":
    # filename = sys.argv[1]
    
    # if filename is None:
    #     raise TypeError

    rospy.init_node('robot_program_node')

    start_program()