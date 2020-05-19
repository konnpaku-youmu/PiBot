#ifndef MAPPING_H
#define MAPPING_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace mapping
{

    class Localizer
    {
    private:
        /* data */
    public:
        Localizer(/* args */);
        ~Localizer();
    };

    class Mapper
    {
    private:
        /* data */
    public:
        Mapper(/* args */);
        ~Mapper();
    };

    class SLAMCore
    {
    private:
        ros::Subscriber laserSub;

        void laserCb(const sensor_msgs::LaserScanConstPtr);

    public:
        SLAMCore(ros::NodeHandle &);
        ~SLAMCore();
    };

} // namespace mapping

#endif