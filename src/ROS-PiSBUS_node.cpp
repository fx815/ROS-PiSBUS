#include "ros/ros.h"
#include "ROS-PiSBUS/SBUS.h"
#include "serial_cppm_array/PWM8.h"

uint16_t RC_in[7] = {1000,1000,1000,1000,1000,1000,1000};
float channels[16] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};

void ppm_callback(const serial_cppm_array::PWM8 &msg)
{
    RC_in[0] = msg.pwm[0];
    RC_in[1] = msg.pwm[1];
    RC_in[2] = msg.pwm[2];
    RC_in[3] = msg.pwm[3];
    RC_in[4] = msg.pwm[4];
    RC_in[5] = msg.pwm[5];
    RC_in[6] = msg.pwm[6];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ppm_sbus_covnerter");
    ros::NodeHandle n;
    ros::Rate loop_rate(111);

    SBUS::SBUS virtual_t("/dev/ttyAMA0");
    virtual_t.begin();

    ros::Subscriber pwm_sub = n.subscribe("ppm_output", 1, ppm_callback); // USB RC Remote

    while (ros::ok())
    {
        for (int i = 0; i < 16; i++){
            if (i<7){
                channels[i] = -1.0 + 2 * ((float(RC_in[i]) - 1000.0)/1000.0);}
            else{
                channels[i] = -1;
            }
        }
        virtual_t.writeCal(&channels[0]);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}