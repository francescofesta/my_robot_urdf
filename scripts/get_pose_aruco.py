#!/usr/bin/env python

import rospy
import std_msgs
from fiducial_msgs.msg import FiducialTransform
from geometry_msgs.msg import Transform

def callback(msg):
    x_tag=(msg.transform.translation.x)
    #rospy.loginfo(rospy.get_caller_id() + "I heard %f", x_tag)
    print("%f", x_tag)


def get_pose():
    rospy.init_node('get_aruco_pose', anonymous=True)
    rospy.Subscriber('/fiducial_transforms', FiducialTransform, callback)
    rospy.spin()

if __name__ == '__main__':
    
    get_pose()
 

    
