#include "ros/ros.h"
#include "ROS-PiSBUS/SBUS.h"
#include "serial_cppm_array/PWM8.h"

uint16_t RC_in[8] = {};
uint16_t channels[16];

void ppm_callback(const serial_cppm_array::PWM8 &msg)
{
    RC_in[0] = msg.pwm[0];
    RC_in[1] = msg.pwm[1];
    RC_in[2] = msg.pwm[2];
    RC_in[3] = msg.pwm[3];
    RC_in[4] = msg.pwm[4];
    RC_in[5] = msg.pwm[5];
    RC_in[6] = msg.pwm[6];
    RC_in[7] = msg.pwm[7];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ppm_sbus_covnerter");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    SBUS::SBUS virtual_t("/dev/ttyAMA0");
    virtual_t.begin();

    ros::Subscriber pwm_sub = n.subscribe("ppm_output", 1, ppm_callback); // USB RC Remote

    while (ros::ok())
    {
        for (int i = 0; i < 8; i++){
            channels[i] = RC_in[i];
        }
        virtual_t.write(channels);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}