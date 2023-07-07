#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "ROS-PiSBUS/SBUS.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Rate loop_rate(250);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}