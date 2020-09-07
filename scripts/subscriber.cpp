
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

void msgCallback(const sensor_msgs::Image::ConstPtr& msg)
{
        ROS_INFO("height = %d, width = %d",msg->height, msg->width);
}


int main(int argc, char** argv)
{
    ros::init(argc,argv,"camera_image");
    ros::NodeHandle n;
    ros::Subscriber img_sub = n.subscribe("/camera1/image_raw", 100, msgCallback);
    ros::spin();
    return 0;
    
}