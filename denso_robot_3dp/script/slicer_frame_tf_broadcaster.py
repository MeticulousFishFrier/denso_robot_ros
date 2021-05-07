#!/usr/bin/env python  

import rospy
import tf
from time import sleep
from geometry_msgs.msg import PoseStamped

def callback(slicer_frame):

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(100)

    #turn pose msg from slicer_frame_pose into translation and rotational tuples
    pos = slicer_frame.pose.position
    ort = slicer_frame.pose.orientation

    trans = (pos.x,pos.y,pos.z)
    rot = (ort.x,ort.y,ort.z,ort.w)

    while not rospy.is_shutdown():
        br.sendTransform(trans,
                         rot,
                         rospy.Time.now(),
                         "/slicer_frame",
                         "/world")
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('slicer_frame_tf')
    print("im on!!")
    rospy.Subscriber("slicer_frame", PoseStamped, callback)
    rospy.spin()

