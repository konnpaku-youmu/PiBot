#include "../include/mapping.h"

namespace mapping
{
    Localizer::Localizer()
    {
        // initializing header
        initPose.header.frame_id = "odometry";
        initPose.header.seq = 0;
        initPose.header.stamp = ros::Time::now();

        // set origin & initial orientation
        initPose.pose.orientation.w = 1.0;

        // set robot's intial origin at world origin
        currPose = initPose;

        return;
    }

    void Localizer::update(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr _new_cloud,
                           const pcl::PointCloud<pcl::PointXYZRGB>::Ptr _prev_cloud)
    {
        if (_prev_cloud->points.size() == 0)
        {
            // initialize pointcloud for the first frame
            _prev_cloud->header = _new_cloud->header;
            _prev_cloud->points = _new_cloud->points;

            // construct NDT
            this->_ndt(_prev_cloud);
        }
        else
        {
            Eigen::MatrixXf _transform;
            estimateTrans(_new_cloud, _prev_cloud, _transform);
        }

        _calc_mean_dist(_new_cloud);
        return;
    }

    void Localizer::estimateTrans(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr _curr,
                                  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr _prev,
                                  Eigen::MatrixXf &_T)
    {
    }

    void Localizer::_align_icp(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr _query,
                               const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr _train)
    {
    }

    void Localizer::_ndt(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr _src)
    {
        // subdivide
        // find boundary
        float _xlim[2] = {100, -100};
        float _ylim[2] = {100, -100};
        for(auto _point:_src->points)
        {
            _xlim[0] = (_point.x < _xlim[0]) ? _point.x:_xlim[0];
            _xlim[1] = (_point.x > _xlim[1]) ? _point.x:_xlim[1];
            _ylim[0] = (_point.y < _xlim[0]) ? _point.y:_ylim[0];
            _ylim[1] = (_point.y > _ylim[1]) ? _point.y:_ylim[1];   
        }
        size_t _xgrids = (_xlim[1] - _xlim[0]) / NDT_GRID_SIZE;
        size_t _ygrids = (_ylim[1] - _ylim[0]) / NDT_GRID_SIZE;
        std::cout << _xgrids << " " << _ygrids << std::endl;
    }

    void Localizer::_calc_mean_dist(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pointcloud)
    {
        this->_mean_dist = 0;
        for(auto _point:_pointcloud->points)
        {
            _mean_dist += sqrtf(_point.x * _point.x + _point.y * _point.y);
        }
        this->_mean_dist /= _pointcloud->points.size();
    }

    void Localizer::init(ros::NodeHandle &nh)
    {
    }

    Mapper::Mapper()
        : globalCloud(new pcl::PointCloud<pcl::PointXYZRGB>)
    {
    }

    SLAMCore::SLAMCore(ros::NodeHandle &nh)
        : _localizer(std::make_unique<Localizer>()),
          _mapper(std::make_unique<Mapper>())
    {
        _laser_sub = nh.subscribe("/robot/scan", 1, &SLAMCore::_laser_cb, this);
        _pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/robot/points", 1);

        this->_localizer->init(nh);

        ros::Rate spinRate(10);
        while (ros::ok())
        {
            ros::spinOnce();
            spinRate.sleep();
            __seqNum__++;
        }
    }

    void SLAMCore::_laser_cb(const sensor_msgs::LaserScanConstPtr _raw_scan)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr _raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // convert laser scan to pointcloud
        this->_scan_to_pcl(_raw_scan, _raw_cloud);
        this->_localizer->update(_raw_cloud, this->_mapper->globalCloud);

        sensor_msgs::PointCloud2 _pcl_msg;
        pcl::toROSMsg(*_raw_cloud, _pcl_msg);
        _pcl_msg.header = _raw_scan->header;
        _pcl_pub.publish(_pcl_msg);
    }

    void SLAMCore::_scan_to_pcl(const sensor_msgs::LaserScanConstPtr _rawScan,
                                const pcl::PointCloud<pcl::PointXYZRGB>::Ptr _dstPcl)
    {
        _dstPcl->header.frame_id = "laser";
        _dstPcl->header.seq = _rawScan->header.seq;
        _dstPcl->header.stamp = ros::Time::now().sec;

        float _d_phi = _rawScan->angle_increment;

        float _curr_phi = _rawScan->angle_min;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr _raw(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (float _range : _rawScan->ranges)
        {
            if (_range > 50)
            {
                _curr_phi += _d_phi;
                continue;
            }
            // convert polar to cartesian
            pcl::PointXYZRGB _newPt;
            _newPt.x = _range * cos(_curr_phi);
            _newPt.y = _range * sin(_curr_phi);
            _newPt.r = 255;
            _newPt.g = 255;
            _newPt.b = 255;

            _raw->points.push_back(_newPt);

            _curr_phi += _d_phi;
        }

        pcl::VoxelGrid<pcl::PointXYZRGB> _filter;
        _filter.setInputCloud(_raw);
        _filter.setLeafSize(0.1, 0.1, 0.1);
        _filter.filter(*_dstPcl);
    }

    void SLAMCore::_publish_all()
    {
    }

} // namespace mapping