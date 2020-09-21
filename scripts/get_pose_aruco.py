#!/usr/bin/env python

import rospy
import numpy as np
import std_msgs
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import *
from trajectory_tracking import Trajectory_Control
from trajectory_generation import Trajectory_generation
from IO_Linearization import io_linearization_control_law  

def callback(msg):
    #x_tag=(msg.transform.translation.x)
    x_goal=msg.transforms[0].transform.translation.z
    y_goal=-msg.transforms[0].transform.translation.x
    #rospy.loginfo(rospy.get_caller_id() + "I heard %f", x_tag)
    print(x_goal, y_goal)


def get_pose():
    rospy.init_node('get_aruco_pose', anonymous=True)
    rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, callback)
    rospy.spin()

if __name__ == '__main__':
    
  
    get_pose()
    trajectory= "cubic"
    tc=Trajectory_Control()
    tc.t = np.linspace(0, 100, 1000)
    tg=Trajectory_generation()
    tc.trajectory_generation(trajectory)
    tc.unicycle_linearized_control()
 

    
