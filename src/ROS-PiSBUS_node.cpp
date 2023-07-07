#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "ROS-PiSBUS/SBUS.h"

double RC_in[7] = {};
float channels[16];

void joy_callback(const sensor_msgs::Joy &joy_in)
{
    RC_in[0] = -joy_in.axes[0];
    RC_in[1] = -joy_in.axes[1];
    RC_in[2] = -joy_in.axes[2];
    RC_in[3] = -joy_in.axes[3];
    RC_in[4] = -joy_in.axes[4];
    RC_in[5] = -joy_in.axes[5];
    RC_in[6] = -joy_in.axes[6];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    SBUS::SBUS virtual_t("/dev/ttyAMA0");
    virtual_t.begin();

    ros::Subscriber joy_sub = n.subscribe("joy", 1, joy_callback); // USB RC Remote

    while (ros::ok())
    {
        for (int i = 0; i < 8; i++){
            channels[i] = RC_in[i];
        }
        virtual_t.writeCal(channels);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}