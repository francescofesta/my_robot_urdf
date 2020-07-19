#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion

from IO_Linearization import io_linearization_control_law
from trajectory_generation import Trajectory_generation

class Trajectory_Control():

    #attributes:

    t = []
    x_d = []
    y_d = []
    v_d = []
    w_d = []
    theta_d = []
    q=[]
    dotx_d=[]
    doty_d=[]

    #methods:

    def __init__(self):
        rospy.loginfo("Starting node Trajectory control")
        rospy.init_node('trajectory_control', anonymous=True) #make node
        #elf.twist_pub = rospy.Publisher('/r2d2_diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self.twist_pub = rospy.Publisher('/DD_controller/cmd_vel', Twist, queue_size=10) 

        rospy.Subscriber('/ground_truth/state',Odometry, self.odometryCb)

    

    #current robot pose
    def odometryCb(self,msg):
        x = round(msg.pose.pose.position.x,4)
        y = round(msg.pose.pose.position.y,4)
        theta = round(self.get_angle_pose(msg.pose.pose),4) 
        self.q = np.array([x, y, theta])
        return self.q
    
    #compute angle from quaternion
    def get_angle_pose(self, quaternion_pose):
        q = [quaternion_pose.orientation.x,
                quaternion_pose.orientation.y,
                quaternion_pose.orientation.z,
                quaternion_pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        theta = yaw
        return theta
    
    #Trajectory_generation()
    #Trajectory generation
    def trajectory_generation(self, trajectory):
        tg = Trajectory_generation()
        if(trajectory == "cubic"):
            #Cubic_trajectory
            q_i = np.array([0.,0., 3.14/2]) #Initial posture (x_i,y_i,theta_i)
            q_f = np.array([3.,6., 0.])    #Final posture   (x_f,y_f,theta_f)
            init_final_velocity = 2
            (self.x_d, self.y_d, self.v_d, self.w_d, self.theta_d) = tg.cubic_trajectory(q_i, q_f, init_final_velocity, self.t)   
        elif(trajectory == "eight"):
            #Eight trajectory
            (self.x_d, self.y_d, self.dotx_d, self.doty_d) = tg.eight_trajectory(self.t)
        elif (trajectory == "cyrcular"):    
            #Cyrcular_trajectory
            (self.x_d, self.y_d, self.v_d, self.w_d, self.theta_d, self.dotx_d, self.doty_d) = tg.cyrcular_trajectory(self.t)


    def get_point_coordinate(self, b):
        #get robot position updated from callback
        x = self.q[0]
        y = self.q[1]
        theta = self.q[2]
        #robot point cooordinate to consider
        y1 = x + b * np.cos(theta)
        y2 = y + b * np.sin(theta)
        return [y1, y2, theta]

    def unicycle_linearized_control(self):
        # Distance of point B from the point of contact P
        b = 0.2

        rospy.sleep(0.1)
        max_t = self.t[len(self.t) - 1]
        len_t = len(self.t)
        for i in np.arange(0, len(self.t)):
            (y1, y2, theta) = self.get_point_coordinate(b)
            (v, w) = io_linearization_control_law(y1, y2, theta, self.x_d[i], self.y_d[i], self.dotx_d[i], self.doty_d[i], b)
            print("linear:{} and angular:{}".format(v, w))           
            #move robot
            self.send_velocities(v, w, theta)
            rospy.sleep(max_t/len_t)
        
        #stop after time
        self.send_velocities(0,0,0)

    #publish v, w
    def send_velocities(self, v, w, theta):
        twist_msg = Twist() # Creating a new message to send to the robot
        # twist_msg.linear.x = v * np.cos(theta)
        # twist_msg.linear.y = v * np.sin(theta)
        # twist_msg.angular.z = w
        
        twist_msg.linear.x = v 
        twist_msg.angular.z = -w
        self.twist_pub.publish(twist_msg)


if __name__ == "__main__":
    try:
        tc=Trajectory_Control()
        tc.t = np.linspace(0, 100, 1000)
       
        trajectory = "cyrcular"  #cubic, eight, cyrcular
        tc.trajectory_generation(trajectory)
        #tc.unicicle_nonLinear_control()
        tc.unicycle_linearized_control()

        #tc.unicycle_cartesian_regulation()

    except rospy.ROSInterruptException:
        pass