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
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>

namespace mapping
{
#define NDT_GRID_SIZE 1.0

    template <typename T>
    std::ostream &operator<<(std::ostream &os, const std::vector<T> &v)
    {
        os << "[";
        for (int i = 0; i < v.size(); ++i)
        {
            os << v[i];
            if (i != v.size() - 1)
                os << ", ";
        }
        os << "]";
        return os;
    }

    class Odometry
    {
    };

    class Localizer
    {
    private:
        geometry_msgs::PoseStamped initPose;

        geometry_msgs::PoseStamped currPose;

        float _mean_dist = 0;

        void _align_icp(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr,
                        const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr);

        void _ndt(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

        void _calc_mean_dist(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

    public:
        Localizer();

        ~Localizer() {}

        void init(ros::NodeHandle &);

        void update(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

        void estimateTrans(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr,
                           const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr,
                           Eigen::MatrixXf &);
    };

    class Mapper
    {
    private:
    public:
        Mapper();

        ~Mapper() {}

        void updateCloud();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalCloud;
    };

    class SLAMCore
    {
    private:
        ros::Subscriber _laser_sub;

        ros::Publisher _pcl_pub;

        std::unique_ptr<Localizer> _localizer;

        std::unique_ptr<Mapper> _mapper;

        uint32_t __seqNum__ = 0;

        void _laser_cb(const sensor_msgs::LaserScanConstPtr);

        void _scan_to_pcl(const sensor_msgs::LaserScanConstPtr,
                          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

        void _publish_all();

    public:
        SLAMCore(ros::NodeHandle &);

        ~SLAMCore() {}
    };

} // namespace mapping

#endif