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
        currPose.header.frame_id = "laser";

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
        }
        else
        {
            Eigen::MatrixXf _transform;
            estimateTrans(_new_cloud, _prev_cloud, _transform);
            // std::cout << _transform << std::endl;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr _trans(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::transformPointCloud(*_new_cloud, *_trans, _transform);

            for (auto _point : _prev_cloud->points)
            {
                _trans->points.push_back(_point);
            }

            pcl::VoxelGrid<pcl::PointXYZRGB> _filter;
            _filter.setInputCloud(_trans);
            _filter.setLeafSize(0.1, 0.1, 0.1);
            _filter.filter(*_prev_cloud);

            this->_test_pose_pub.publish(currPose);
        }

        _calc_mean_dist(_new_cloud);
        return;
    }

    void Localizer::estimateTrans(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr _curr,
                                  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr _prev,
                                  Eigen::MatrixXf &_T)
    {
        pcl::ExtractIndices<pcl::PointXYZRGB> _map_clipper;
        pcl::ExtractIndices<pcl::PointXYZRGB> _new_clipper;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr _local_map(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr _local_laser_scan(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        // move current laser scan to current position
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr _curr_trans(new pcl::PointCloud<pcl::PointXYZRGB>);
        // Eigen::Matrix4f _curr_t = Eigen::Matrix4f::Identity();
        // Eigen::Quaternionf _curr_r(this->currPose.pose.orientation.w,
        //                            this->currPose.pose.orientation.x,
        //                            this->currPose.pose.orientation.y,
        //                            this->currPose.pose.orientation.z);
        // _curr_t.block<3, 3>(0, 0) = _curr_r.toRotationMatrix();
        // _curr_t(0, 3) = this->currPose.pose.position.x;
        // _curr_t(1, 3) = this->currPose.pose.position.y;

        _map_clipper.setInputCloud(_prev);
        pcl::PointIndices _map_indices;
        for (size_t i = 0; i < _prev->points.size(); ++i)
        {
            float _dist = sqrtf((_curr->points[i].x - this->currPose.pose.position.x) * (_curr->points[i].x - this->currPose.pose.position.x) +
                                (_curr->points[i].y - this->currPose.pose.position.y) * (_curr->points[i].y - this->currPose.pose.position.y));
            if (_dist > this->_mean_dist)
            {
                _map_indices.indices.push_back(i);
            }
        }
        _map_clipper.setIndices(boost::make_shared<pcl::PointIndices>(_map_indices));
        _map_clipper.setNegative(true);
        _map_clipper.filter(*_local_map);

        _new_clipper.setInputCloud(_curr);
        pcl::PointIndices _new_indices;
        for (size_t i = 0; i < _curr->points.size(); ++i)
        {
            float _dist = sqrtf(_curr->points[i].x * _curr->points[i].x + _curr->points[i].y * _curr->points[i].y);
            if (_dist > this->_mean_dist)
            {
                _new_indices.indices.push_back(i);
            }
        }
        _new_clipper.setIndices(boost::make_shared<pcl::PointIndices>(_new_indices));
        _new_clipper.setNegative(true);
        _new_clipper.filter(*_local_laser_scan);

        // align local scan with local map
        pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> _ndt;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr _ndt_output(new pcl::PointCloud<pcl::PointXYZRGB>);
        _ndt.setTransformationEpsilon(0.01);
        _ndt.setStepSize(0.05);
        _ndt.setResolution(1);
        _ndt.setMaximumIterations(35);

        _ndt.setInputSource(_local_laser_scan);
        _ndt.setInputTarget(_local_map);

        Eigen::Matrix4f _initial_guess = Eigen::Matrix4f::Identity();
        _ndt.align(*_ndt_output, _initial_guess);

        if (_ndt.hasConverged())
        {
            _T = _ndt.getFinalTransformation();
            std::cout << _ndt.getFitnessScore() << std::endl;
            // update pose
            Eigen::Quaternionf _q(_T.block<3, 3>(0, 0));
            currPose.pose.orientation.w = _q.w();
            currPose.pose.orientation.x = _q.x();
            currPose.pose.orientation.y = _q.y();
            currPose.pose.orientation.z = _q.z();

            currPose.pose.position.x = _T(0, 3);
            currPose.pose.position.y = _T(1, 3);
        }

        return;
    }

    void Localizer::_align_icp(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr _query,
                               const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr _train)
    {
        return;
    }

    void Localizer::_ndt_(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr _src)
    {
        // subdivide
        // find boundary
        float _xlim[2] = {100, -100};
        float _ylim[2] = {100, -100};
        for (auto _point : _src->points)
        {
            _xlim[0] = (_point.x < _xlim[0]) ? _point.x : _xlim[0];
            _xlim[1] = (_point.x > _xlim[1]) ? _point.x : _xlim[1];
            _ylim[0] = (_point.y < _ylim[0]) ? _point.y : _ylim[0];
            _ylim[1] = (_point.y > _ylim[1]) ? _point.y : _ylim[1];
        }

        int _xgrids = (int)((_xlim[1] - _xlim[0]) / ((1 - NDT_OVERLAP) * NDT_GRID_SIZE)) + 1;
        int _ygrids = (int)((_ylim[1] - _ylim[0]) / ((1 - NDT_OVERLAP) * NDT_GRID_SIZE)) + 1;

        // map measurements to grids
        float _xstart = _xlim[0] - (1 - NDT_OVERLAP) * NDT_GRID_SIZE;
        float _ystart = _ylim[0] - (1 - NDT_OVERLAP) * NDT_GRID_SIZE;

        std::vector<pcl::PointXYZRGB> _tmp_points_in_grids[_xgrids * _ygrids];

        for (auto _point : _src->points)
        {
            int _gridx = (int)((_point.x - _xstart) / NDT_GRID_SIZE);
            int _gridy = (int)((_point.y - _ystart) / NDT_GRID_SIZE);

            _tmp_points_in_grids[_gridy * _xgrids + _gridx].push_back(_point);
            _tmp_points_in_grids[_gridy * _xgrids + _gridx + 1].push_back(_point);
            _tmp_points_in_grids[(_gridy + 1) * _xgrids + _gridx].push_back(_point);
            _tmp_points_in_grids[(_gridy + 1) * _xgrids + _gridx + 1].push_back(_point);
        }
        // std::cout << _xstart << " " << _ystart << std::endl;
        // std::cout << _tmp_points_in_grids[0] << std::endl;

        std::vector<std::pair<int, std::vector<pcl::PointXYZRGB>>> _points_in_grids;

        for (int i = 0; i < _xgrids * _ygrids; ++i)
        {
            if (_tmp_points_in_grids[i].size() <= 3)
            {
                continue;
            }
            _points_in_grids.push_back(std::make_pair(i, _tmp_points_in_grids[i]));
            // std::cout << i << " " << _tmp_points_in_grids[i] << std::endl;
        }

        //construct NDT
        std::vector<std::pair<int, std::pair<pcl::PointXYZRGB, Eigen::Matrix2d>>> _distri_grid;
        // compute average and covariance
        for (auto _grouped_points : _points_in_grids)
        {
            int _tmp_grid_idx = _grouped_points.first;
            auto _tmp_point_lst = _grouped_points.second;
            Eigen::Matrix2d _cov = Eigen::Matrix2d::Zero();
            pcl::PointXYZRGB _sum_p, _aver_p;
            // average
            _sum_p.x = 0;
            _sum_p.y = 0;
            for (auto _point : _tmp_point_lst)
            {
                _sum_p.x += _point.x;
                _sum_p.y += _point.y;
            }
            _aver_p.x = _sum_p.x / _tmp_point_lst.size();
            _aver_p.y = _sum_p.y / _tmp_point_lst.size();

            // cov
            for (auto _point : _tmp_point_lst)
            {
                Eigen::Vector2d _pt;
                _pt(0) = _point.x - _aver_p.x;
                _pt(1) = _point.y - _aver_p.y;
                _cov += _pt * _pt.transpose();
            }
            _cov /= (_tmp_point_lst.size() - 1);
            // prevent singualrity
            {
                auto _eigenvalues = _cov.eigenvalues().real();
                if (std::min(_eigenvalues(0), _eigenvalues(1)) <= 1e-3 * std::max(_eigenvalues(0), _eigenvalues(1)))
                {
                    continue;
                }
            }
            std::pair<pcl::PointXYZRGB, Eigen::Matrix2d> _distribution(_aver_p, _cov);
            _distri_grid.push_back(std::make_pair(_tmp_grid_idx, _distribution));
        }

        // visualize(optional)
        cv::Mat _vis = cv::Mat::zeros(50 * _ygrids, 50 * _xgrids, CV_8UC1);
        for (auto _grid : _distri_grid)
        {
            auto _grid_aver = _grid.second.first;
            auto _grid_cov = _grid.second.second;
            auto _grid_idx = _grid.first;

            int _gx = _grid_idx % _xgrids;
            int _gy = _grid_idx / _xgrids;
            // std::cout << _gx << " " << _gy << std::endl;

            for (size_t i = 0; i < 100; ++i)
            {
                for (size_t j = 0; j < 100; ++j)
                {
                    Eigen::Vector2d _de_aver;
                    _de_aver(0) = 0.01 * i + _gx * NDT_GRID_SIZE - _grid_aver.x;
                    _de_aver(1) = 0.01 * j + _gy * NDT_GRID_SIZE - _grid_aver.y;
                    double _prob = std::exp(-0.5 * _de_aver.transpose() * _grid_cov * _de_aver);
                    uint8_t _pixel_value = (uint8_t)(_prob * 255);
                    _vis.at<uint8_t>(_gy * 50 + i, _gx * 50 + j) += _pixel_value;
                }
            }
        }
        cv::imshow("grid", _vis);
        cv::waitKey(50);
    }

    void Localizer::_calc_mean_dist(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pointcloud)
    {
        this->_mean_dist = 0;
        for (auto _point : _pointcloud->points)
        {
            _mean_dist += sqrtf(_point.x * _point.x + _point.y * _point.y);
        }
        this->_mean_dist /= _pointcloud->points.size();
        return;
    }

    void Localizer::init(ros::NodeHandle &nh)
    {
        this->_test_nh = nh;
        this->_test_pcl_pub = _test_nh.advertise<sensor_msgs::PointCloud2>("/test/pcl", 10);
        this->_test_pose_pub = _test_nh.advertise<geometry_msgs::PoseStamped>("/test/pose", 10);
        return;
    }

    Mapper::Mapper()
        : globalCloud(new pcl::PointCloud<pcl::PointXYZRGB>)
    {
        return;
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

        return;
    }

    void SLAMCore::_laser_cb(const sensor_msgs::LaserScanConstPtr _raw_scan)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr _raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // convert laser scan to pointcloud
        this->_scan_to_pcl(_raw_scan, _raw_cloud);
        this->_localizer->update(_raw_cloud, this->_mapper->globalCloud);

        sensor_msgs::PointCloud2 _pcl_msg;
        pcl::toROSMsg(*this->_mapper->globalCloud, _pcl_msg);
        _pcl_msg.header = _raw_scan->header;
        _pcl_pub.publish(_pcl_msg);

        return;
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
            if (_range > 20)
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
        _filter.setLeafSize(0.05, 0.05, 0.1);
        _filter.filter(*_dstPcl);

        return;
    }

    void SLAMCore::_publish_all()
    {
        return;
    }

} // namespace mapping