#ifndef TELE_CON_H
#define TELE_CON_H

#include <ros/ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>

namespace ps3
{
    class Joystick
    {
    private:
        ros::Subscriber _joy_sub;

        ros::Publisher _twist_pub;

        ros::Publisher _button_pub;

        ros::ServiceClient _start_rp_motor_cli;

        ros::ServiceClient _stop_rp_motor_cli;

        bool _l_pressed = false;

        bool _r_pressed = true;

        geometry_msgs::Twist _cmd_out;

        int _rp_motor_cmd;

        bool _rp_motor_status = true;

        std::vector<int8_t> _buttons_input;

        void _js_input_cb(const sensor_msgs::JoyConstPtr);

        void _cmd_pub();

        void _js_ascii_art();

    public:
        Joystick(ros::NodeHandle &);

        ~Joystick() { return; }
    };
} // namespace ps3

#endif