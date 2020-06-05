#include "../include/hector_app/hector_app.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_app");
    ros::NodeHandle nh;

    hector_app::App applet(nh);

    return 0;
}