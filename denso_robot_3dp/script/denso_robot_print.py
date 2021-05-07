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


__REQUIRED_API_VERSION__ = "1"

PLANAR_ORIENTATION = Quaternion(0, 1, 0, 0)

# NOTE: the bottom left corner of the build plate is X0Y0


# TODO:put this in a class later
# extract xyz from GCode and then transform it to robot coordinates
# TODO: switch the x and y and add offest to coordinates so matches up with the slicer coord

# error coordinate mismatch current_gcode_pose is in the wrong coordinate

# Extracts the XYZ of the G-Code line and converts them into a PoseStamped msg
def get_transformed_coords(line, gcode_pose_stamped,listener):
    # extracts the XYZE coordinates of gcode line
    x_start = line.find('X')
    x_end = line.find(' ', x_start)
    x = float(line[x_start+1:x_end]) / \
        1000 if x_start != -1 else gcode_pose_stamped.pose.position.x
    y_start = line.find('Y')
    y_end = line.find(' ', y_start)
    y = float(line[y_start+1:y_end]) / \
        1000 if y_start != -1 else gcode_pose_stamped.pose.position.y

    z_start = line.find('Z')
    z_end = line.find(' ', z_start)
    z = float(line[z_start+1:z_end]) / \
        1000 if z_start != -1 else gcode_pose_stamped.pose.position.z

    # E_start = line.find('E')
    # E_end = line.find(' ', e_start)
    # E = line[E_start+1:E_end] if E_start != -1 else None

    # set up the point as a PoseStamped msg so tf can transform it
    goal_pose_stamped = PoseStamped()
    goal_pose_stamped.header.frame_id = "slicer_frame"
    goal_pose_stamped.pose = Pose(position=Point(x, y, z))
    gcode_pose_stamped = goal_pose_stamped
                              
    # transform the coordiantes from slicer_frame to world frame
    goal_pose_stamped = listener.transformPose("world", goal_pose_stamped)

    # add orientation to make extruder point downwards
    # need to be changed if nc code is used instead of gcode
    goal_pose_stamped.pose.orientation = PLANAR_ORIENTATION

    # round the to the 6 decimal place, which is 0.001mm or 1e-6m
    position = goal_pose_stamped.pose.position
    position.x, position.y, position.z = list(
        map(lambda a: round(a, 6), [position.x, position.y, position.z]))
    goal_pose_stamped.pose.position = position

    # NOTE xyz in m, extrusion in mm, goal_pose_stamped is in world frame, gcode_pose_stamped is in slicer frame
    return gcode_pose_stamped, goal_pose_stamped


def get_cmd_3dp_controller(line):
    # - finds the last space in the command, which means skipping all the XYZ and getting the E and F's
    # - will need to store the values of the F commands later b/c it indicates the motor speed?
    # and that will determine the maximum velocity of the xyz movements as well

    idx = line.find('E')
    if idx == -1:
        idx = F_idx = line.find('F')

    return "G1 " + line[idx:]


def is_move_ext_pose(line):
    a_string = "A string is more than its parts!"
    matches = ['X', 'Y','E']

    if all(x in line for x in matches):
        return True

    return False

def get_sequence(gcode_file_path, debug_file_path):
    # takes current xyz to be used as previoius coordinates when Gcodes doesnt specify
    # and convert from m to mm

    gcode_pose_stamped = PoseStamped()

    # recording the poses before extrusion so that when they are reached
    # extrusion and move to next point can occur concurrently
    before_ext_pose = PoseStamped()
    move_ext_poses = PoseArray()
    move_ext_poses.header.frame_id = r.get_planning_frame()

    for_loop_seq = []

    sequence = Sequence()
    

    scene = moveit_commander.PlanningSceneInterface()

    # needed to spawn the box representing the build plate
    rospy.sleep(1)

    build_plate = PoseStamped()
    build_plate.header.frame_id = r.get_planning_frame()
    build_plate.pose.position.x = .3
    build_plate.pose.position.y = 0.
    build_plate.pose.position.z = 0.2

    #build_plate.pose.position.z = 0.003

    #scene.add_box("build_plate", build_plate, (0.214, 0.214, 0.003))

    slicer_frame = build_plate
    slicer_frame.pose.position.x = build_plate.pose.position.x - .1
    slicer_frame.pose.position.y = build_plate.pose.position.y + .1
    # sets the orientation of slicer frame to be rotated -90 degree about z axis of world frame
    # rotation about
    q = tf.transformations.quaternion_from_euler(
        0, 0, math.radians(-90), 'ryxz').tolist()
    slicer_frame.pose.orientation = Quaternion(*q)

    #scene.add_sphere("slicer_origin", slicer_frame, radius = .005)

    # makes sure the slicer_frame msg is published
    while not rospy.is_shutdown():
        if slicer_frame_pub.get_num_connections() > 0:
            slicer_frame_pub.publish(slicer_frame)
            rospy.loginfo('Published')
            rospy.sleep(1)
            break


    # make listener for transform the PoseStamp msg
    listener = tf.TransformListener()
    listener.waitForTransform('slicer_frame', 'world',
                              rospy.Time(), rospy.Duration(4))

    #gcode_pose_stamped.pose.position.z = .003

    # takes coord from Gcode and plugs them into sequence object
    # extrude first, then move
    with open(gcode_file_path, "r") as gcode_file, open(debug_file_path, 'w+') as debug_file, open(debug_file_path[:-4]+"2.gcode", 'w+') as debug_file2:
        for line in gcode_file:
            # checks if the line is an empty line or a comment
            if line:
                if line[0] == ";" or line[0] == "\n":
                    continue

                else:
                    cmd_end = line.find(' ')
                    cmd = line[:cmd_end]

                    if cmd == "G1" or cmd == "G0":
                        # only execute when encountering G1 commands

                        # TODO: sync up the extrusion and the movement and the change in feed rate
                        # need to check if the 3d printer controller can handle empty xyz axis

                        gcode_pose_stamped, goal_pose_stamped = get_transformed_coords(
                            line, gcode_pose_stamped, listener)

                        cmd_3dp_controller = get_cmd_3dp_controller(line)

                        # debug
                        # print(''.join(map(str,[x,y,z,e])))
                        pos = goal_pose_stamped.pose.position
                        debug_file.write("LIN: ")
                        debug_file.write(
                            ', '.join(map(str, [pos.x, pos.y, pos.z]))+'\n')

                        debug_file.write("EXT: " + cmd_3dp_controller)
                        debug_file2.write(cmd_3dp_controller)

                        for_loop_seq.append(Lin(goal=goal_pose_stamped.pose, vel_scale=1, acc_scale=.2))
                        sequence.append(Lin(goal=goal_pose_stamped.pose, vel_scale=0.01, acc_scale=.2))


                        if (is_move_ext_pose(line)):
                            move_ext_poses.poses.append(before_ext_pose.pose)
                        
                        #update the pose before next extrusion happened
                        before_ext_pose = goal_pose_stamped


                    elif cmd == "G28":
                        # G28 home (X0Y0Z0)
                        gcode_pose_stamped, goal_pose_stamped = get_transformed_coords(
                            "G1 X0 Y0 Z0\n", gcode_pose_stamped,listener)

                    else:
                        extruder_cmd = line
                        debug_file.write("EXT: " + line)
                        debug_file2.write(line)

    return for_loop_seq,sequence, move_ext_poses

if __name__ == "__main__":
    rospy.init_node('robot_program_node')

    # begins the robot controller
    r = Robot(__REQUIRED_API_VERSION__)

    # begins the slicer_frame publisher
    slicer_frame_pub = rospy.Publisher(
        'slicer_frame', PoseStamped, queue_size=10)
    
    #begin 

    move_ext_poses_pub = rospy.Publisher(
        'move_ext_poses', PoseArray, queue_size=10)
   

    # gets Gcode relative file path, file should be under /resources
    rel_path = sys.argv[1]
    script_dir = os.path.dirname(__file__)
    gcode_file_path = os.path.join(script_dir, rel_path)
    debug_file_path = os.path.join(script_dir, "resources/debug.txt")

    for_loop_seq, sequence, move_ext_poses = get_sequence(gcode_file_path, debug_file_path)
    #print(move_ext_poses.poses)

    move_ext_poses_pub.publish(move_ext_poses)

    r.move(sequence)



    # before_ext_pose = PoseStamped()
    # before_ext_pose.pose.position.x = 0.3
    # before_ext_pose.pose.position.y = 0.009637
    # before_ext_pose.pose.position.z = 0.20025

    # move_ext_poses = PoseArray()
    # move_ext_poses.poses.append(before_ext_pose.pose)
    # move_ext_poses_pub.publish(move_ext_poses)

    #r.move(Lin(goal=Pose(position=Point(0.3,0.009637, 0.20025)), vel_scale=1, acc_scale = .2 ))





