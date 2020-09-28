#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion
from fiducial_msgs.msg import FiducialTransformArray

import numpy as np
from scipy.integrate import odeint
from trajectory_generation import Trajectory_generation
from IO_Linearization import io_linearization_control_law




class Trajectory_control():
    #attributes
    t = []
    x_d = []
    y_d = []
    v_d = []
    w_d = []
    theta_d = []
    q=[]
    dotx_d=[]
    doty_d=[]
    x_goal=0.
    y_goal=0.
    goal_pose=[]


    

    #methods
    def __init__(self):
        rospy.loginfo("Starting node Trajectory control")
        rospy.init_node('trajectory_control', anonymous=True) #make node
        self.twist_pub = rospy.Publisher('/DD_controller/cmd_vel', Twist, queue_size=10) 
        rospy.Subscriber('/ground_truth/state',Odometry, self.odometryCb) 
        #elf.twist_pub = rospy.Publisher('/r2d2_diff_drive_controller/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.ArucoCb)
        
        

         
       

        
        # print(self.x_goal, self.y_goal)
        #rospy.Subscriber('move_base_simple/goal', PoseStamped, self.on_goal)

    #current robot pose
    def odometryCb(self,msg):
        x = round(msg.pose.pose.position.x,4)
        y = round(msg.pose.pose.position.y,4)
        theta = round(self.get_angle_pose(msg.pose.pose),4) 
        self.q = np.array([x, y, theta])
        print("odometry:" , self.q)
        return self.q

    def ArucoCb(self,msg):
        if msg.transforms:
            self.x_goal=msg.transforms[0].transform.translation.z
            self.y_goal=-msg.transforms[0].transform.translation.x
        # self.x_goal=1.
        # self.y_goal=1.
        print(self.x_goal, self.y_goal) 
        # self.goal_pose=np.array([self.x_goal,self.y_goal])
        
        # return self.goal_pose

    #compute angle from quaternion
    def get_angle_pose(self, quaternion_pose):
        q = [quaternion_pose.orientation.x,
                quaternion_pose.orientation.y,
                quaternion_pose.orientation.z,
                quaternion_pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        theta = yaw
        return theta

    #Trajectory generation
    def trajectory_generation(self, trajectory):
        tg = Trajectory_generation()
        if(trajectory == "cubic"):
            # global_pose=self.get_pose()
            # global_aruco=self.from_vehicle2global_frame(global_pose[0],global_pose[1],global_pose[2],self.x_goal,self.y_goal)

        
            #Cubic_trajectory
            # q_i = np.array([self.q[0],self.q[1],self.q[2]]) #Initial posture (x_i,y_i,theta_i)
            q_i = np.array(0., 0., 0.)
            q_f = np.array(3., 6., 0.)    #Final posture   (x_f,y_f,theta_f)
            #print(self.x_goal,self.y_goal)
            # q_f = np.array([self.x_goal, self.y_goal, 0.])
            
            print(q_f)
            init_final_velocity = 0.8 # 2
            (self.x_d, self.y_d, self.v_d, self.w_d, self.theta_d, self.dotx_d, self.doty_d) = tg.cubic_trajectory(q_i, q_f, init_final_velocity, self.t)   
            # print("xd: ", self.x_d)
            #print("yd: ",self.y_d)
        elif(trajectory == "eight"):
            #Eight trajectory
            (self.x_d, self.y_d, self.dotx_d, self.doty_d) = tg.eight_trajectory(self.t)
        elif (trajectory == "cyrcular"):    
            #Cyrcular_trajectory
            (self.x_d, self.y_d, self.v_d, self.w_d, self.theta_d, self.dotx_d, self.doty_d) = tg.cyrcular_trajectory(self.t)


    def get_pose(self):
        #get robot position updated from callback
        x = self.q[0]
        y = self.q[1]
        theta = self.q[2]
        return np.array([x, y, theta])

    def get_error(self, T):
        #slide 80 LDC
        (x, y, theta) = self.get_pose()
        #rospy.loginfo("x={} y={} th={}".format(x,y,theta))
        #compute error
        e1 = (self.x_d[T] - x) * np.cos(theta) + (self.y_d[T] - y) * np.sin(theta)
        e2 = -(self.x_d[T] - x) * np.sin(theta) + (self.y_d[T] - y) * np.cos(theta)
        e3 = self.theta_d[T] - theta
        err = np.array([e1, e2, e3])
        return err


    
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

        for i in np.arange(0, len(self.t)-1):
            (y1, y2, theta) = self.get_point_coordinate(b)
            global_aruco=self.from_vehicle2global_frame(self.q[0],self.q[1],self.q[2],self.x_goal,self.y_goal)
            print(global_aruco)
            dist = np.sqrt((self.q[0]-global_aruco[0])**2+(self.q[1]-global_aruco[1])**2)
            if (dist>0.2):
                (v, w) = io_linearization_control_law(y1, y2, theta, self.x_d[i], self.y_d[i], self.dotx_d[i], self.doty_d[i], b)
                print("linear:{} and angular:{}".format(v, w))  
                            
                    #move robot
                self.send_velocities(v, w, theta)
            else:
                break
                 
            #     self.send_velocities(0,0,0)
            rospy.sleep(max_t/len_t)
        
        #stop after time
        self.send_velocities(0,0,0)

    def from_vehicle2global_frame(self, x, y, theta, xv, yv):
        mat=np.matrix([[np.cos(theta), -np.sin(theta), 0, x],[np.sin(theta), np.cos(theta), 0, y],[0, 0, 1, 0], [0, 0, 0, 1]])
        vehicle_cord=np.array([xv, yv, 0, 1])
        vehicle_cord=vehicle_cord.reshape(4,1)
        global_frame=mat*vehicle_cord
        return global_frame



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
        tc=Trajectory_control()
        tc.t = np.linspace(0, 100, 1000)
       
        trajectory = "cubic"  #cubic, eight, cyrcular
        while(1):
            if (tc.x_goal) != 0 and (tc.y_goal!=0):
                tc.trajectory_generation(trajectory)
                #tc.unicicle_nonLinear_control()
                tc.unicycle_linearized_control()
                break
        
        

        #tc.unicycle_cartesian_regulation()

    except rospy.ROSInterruptException:
        pass