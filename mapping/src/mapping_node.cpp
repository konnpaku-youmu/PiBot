#include "../include/mapping.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mapping");
    ros::NodeHandle nh;

    mapping::SLAMCore core(nh);

    return 0;
}