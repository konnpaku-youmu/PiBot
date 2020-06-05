#include "../include/ps3_con/tele_con.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ps3js");
    ros::NodeHandle nh;

    ps3::Joystick js(nh);
    return 0;
}