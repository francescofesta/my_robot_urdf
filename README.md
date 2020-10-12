# my_robot_urdf
1 download aruco_detect ROS package from https://github.com/UbiquityRobotics/fiducials

2 download all aruco_detects' dependecies. you can check them on http://wiki.ros.org/aruco_detect

3 catkin_make

4 change aruco_detect launch file with respectto your project needs. In this case substute the original file with aruco_detectV2.launch that you can find in the main folder

5 roslaunch my_robot_urdf display.launch

6 rosrun my_robot_urdf scripts/diff_drive/nodes/diff_drive_go_to_goal

 you can move in the camera's FOV the aruco tag so that the robot follows the tag
