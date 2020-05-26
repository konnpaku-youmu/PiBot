#ifndef MAPPING_H
#define MAPPING_H

#include <memory>
#include <math.h>
#include <signal.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>

namespace mapping
{
    class Localizer
    {
    private:
        ros::NodeHandle n;

        ros::Publisher posePub;

        ros::Publisher currPclPub;

        ros::Publisher testPclPub;

        geometry_msgs::PoseStamped initPose;

        geometry_msgs::PoseStamped currPose;

        geometry_msgs::PoseStamped currVelo;

        Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();

        float _mean_dist = 0;

        uint32_t _seq_num = 0;

        void _align_icp(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr,
                      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr);

        void _calc_mean_dist(const pcl::PointCloud<pcl::PointXYZ>::Ptr);

        float _calc_dist(const pcl::PointXYZ &, const pcl::PointXYZ &);

    public:
        Localizer();

        ~Localizer() {}

        void init(ros::NodeHandle &);

        void update(const pcl::PointCloud<pcl::PointXYZ>::Ptr,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr);

        void estimateTrans(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr,
                           const pcl::PointCloud<pcl::PointXYZ>::ConstPtr,
                           Eigen::MatrixXf &);
    };

    class Mapper
    {
    private:
    public:
        Mapper();

        ~Mapper() {}

        void updateCloud();

        pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloud;
    };

    class SLAMCore
    {
    private:
        ros::Subscriber laserSub;

        ros::Publisher mapPub;

        ros::Publisher pclPub;

        std::unique_ptr<Localizer> localizer;

        std::shared_ptr<Mapper> mapper;

        std_msgs::Header genericHeader;

        void laserCb(const sensor_msgs::LaserScanConstPtr);

        void scanToPcl(const sensor_msgs::LaserScanConstPtr,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr);

        void publish_all();

        uint32_t __seqNum__ = 0;

    public:
        SLAMCore(ros::NodeHandle &);

        ~SLAMCore() {}
    };

} // namespace mapping

#endif