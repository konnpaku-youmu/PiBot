#include "../include/hector_app/hector_app.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hector_app");
    ros::NodeHandle nh;

    hector_app::App applet(nh);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}