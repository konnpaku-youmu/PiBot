#include "../include/controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_controller");
    ros::NodeHandle nh;

    robot_control::Controller c(nh);
    return 0;
}