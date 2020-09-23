#!/usr/bin/env python

import rospy
import numpy as np
import std_msgs
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import *
from trajectory_tracking import Trajectory_Control



    
            
# def callback(msg):
#     #x_tag=(msg.transform.translation.x)
#     Trajectory_Control.x_goal=msg.transforms[0].transform.translation.z
#     Trajectory_Control.y_goal=-msg.transforms[0].transform.translation.x
#     #rospy.loginfo(rospy.get_caller_id() + "I heard %f", x_tag)
#     print(Trajectory_Control.x_goal, Trajectory_Control.y_goal)


# def get_pose():
#     rospy.init_node('get_aruco_pose', anonymous=True)
#     rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, callback)
#     rospy.spin()



# if __name__ == '__main__':
    
  
#     get_pose()
#     trajectory= "cubic"
#     tc=Trajectory_Control()
#     tc.t = np.linspace(0, 100, 1000)

#     tc.trajectory_generation(trajectory)
#     tc.unicycle_linearized_control()
 

class Get_Aruco_Pose():

     #attributes
     x_goal=[]
     y_goal=[]


     #methods

     def get_pose_aruco(self):
        # rospy.init_node('get_aruco_pose', anonymous=True)
        rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.callback)
        #rospy.spin()
    
     def callback(self,msg):
        self.x_goal=msg.transforms[0].transform.translation.z
        self.y_goal=-msg.transforms[0].transform.translation.x
        print(self.x_goal, self.y_goal)        

if __name__ == "__main__":
    try:
        GP=Get_Aruco_Pose()
        

        rospy.init_node('get_aruco_pose', anonymous=True)
        GP.get_pose_aruco()
        


        rospy.spin()
    except rospy.ROSInterruptException:
        pass

