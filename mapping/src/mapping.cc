#include "../include/mapping.h"

namespace mapping
{

    Localizer::Localizer(/* args */)
    {
    }

    Localizer::~Localizer()
    {
    }

    Mapper::Mapper(/* args */)
    {
    }

    Mapper::~Mapper()
    {
    }

    SLAMCore::SLAMCore(ros::NodeHandle &nh)
    {
        laserSub = nh.subscribe("/robot/scan", 20, &SLAMCore::laserCb, this);


        ros::Rate spinRate(20);
        while (ros::ok())
        {
            ros::spinOnce();
            spinRate.sleep();
        }
    }

    SLAMCore::~SLAMCore()
    {
    }

    void SLAMCore::laserCb(const sensor_msgs::LaserScanConstPtr rawScan)
    {
        std::vector<float> laserRanges = rawScan->ranges;
        for(auto range:laserRanges)
        {
            
        }
    }

} // namespace mapping