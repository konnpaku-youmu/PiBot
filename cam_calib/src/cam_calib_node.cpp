#include "../include/cam_calib/CamCalib.h"

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calib");
    ros::NodeHandle nh;

    ExCalibur calib(nh);

    return 0;
}