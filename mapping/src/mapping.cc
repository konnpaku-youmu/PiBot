#include "../include/mapping.h"

namespace mapping
{
    Localizer::Localizer()
    {
        // initializing header
        initPose.header.frame_id = "laser";
        initPose.header.seq = _seq_num;
        initPose.header.stamp = ros::Time::now();

        // set origin & initial orientation
        initPose.pose.orientation.w = 1.0;

        // set robot's intial origin at world origin
        currPose = initPose;
    }

    void Localizer::update(const pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud,
                           const pcl::PointCloud<pcl::PointXYZ>::Ptr prevCloud)
    {
        if (prevCloud->points.size() == 0)
        {
            // initialize pointcloud for the first frame
            prevCloud->header = newCloud->header;
            prevCloud->points = newCloud->points;
        }
        else
        {
            Eigen::MatrixXf _transform;
            pcl::PointCloud<pcl::PointXYZ>::Ptr _transformed_pcl(new pcl::PointCloud<pcl::PointXYZ>);
            estimateTrans(newCloud, prevCloud, _transform);

            Eigen::Quaternionf q(_transform.block<3, 3>(0, 0));

            currPose.pose.position.x = _transform(0, 3);
            currPose.pose.position.y = _transform(1, 3);
            currPose.pose.orientation.w = q.w();
            currPose.pose.orientation.x = q.x();
            currPose.pose.orientation.y = q.y();
            currPose.pose.orientation.z = q.z();

            posePub.publish(currPose);

            trans(0, 3) =  1.1*currPose.pose.position.x;
            trans(1, 3) =  1.1*currPose.pose.position.y;
            trans.block<3, 3>(0, 0) = _transform.block<3, 3>(0, 0);

            pcl::transformPointCloud(*newCloud, *_transformed_pcl, trans);

            sensor_msgs::PointCloud2 _msg;
            pcl::toROSMsg(*_transformed_pcl, _msg);
            _msg.header.stamp = ros::Time::now();
            currPclPub.publish(_msg);

            pcl::toROSMsg(*newCloud, _msg);
            _msg.header.stamp = ros::Time::now();
            testPclPub.publish(_msg);

            for (auto _pt : _transformed_pcl->points)
            {
                prevCloud->points.push_back(_pt);
            }
            // raise(SIGINT);
        }

        // find the mean range of last frame of points
        _calc_mean_dist(newCloud);
        return;
    }

    void Localizer::estimateTrans(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr _curr,
                                  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr _prev,
                                  Eigen::MatrixXf &_T)
    {
        // select points from existing pcl by distance
        pcl::PointCloud<pcl::PointXYZ>::Ptr _pts_in(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> clipper;
        clipper.setInputCloud(_prev);
        pcl::PointIndices indices;

        for (size_t i = 0; i < _prev->points.size(); ++i)
        {
            pcl::PointXYZ currPt;
            currPt.x = currPose.pose.position.x;
            currPt.y = currPose.pose.position.y;
            float _dist = _calc_dist(currPt, _prev->points[i]);
            if (_dist > 10 * _mean_dist)
            {
                indices.indices.push_back(i);
            }
        }
        clipper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
        clipper.setNegative(true);
        clipper.filter(*_pts_in);

        pcl::PointCloud<pcl::PointXYZ>::Ptr _final(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> _icp;
        _icp.setInputCloud(_curr);
        _icp.setInputTarget(_pts_in);
        _icp.setMaximumIterations(50);
        _icp.setTransformationEpsilon(1e-8);
        _icp.setMaxCorrespondenceDistance(5);
        _icp.setEuclideanFitnessEpsilon(0.01);
        _icp.setRANSACOutlierRejectionThreshold(0.01);

        _icp.align(*_final);

        if (_icp.hasConverged())
        {
            _T = _icp.getFinalTransformation();
        }
    }

    void Localizer::_align_icp(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr _query,
                               const pcl::PointCloud<pcl::PointXYZ>::ConstPtr _train)
    {
    }

    void Localizer::_calc_mean_dist(const pcl::PointCloud<pcl::PointXYZ>::Ptr points)
    {
        float _mean = 0.0;
        size_t _no_hit_count = 0;
        for (auto point : points->points)
        {
            float _range = sqrtf(point.x * point.x + point.y * point.y);
            if (_range < 100)
            {
                _mean += _range;
            }
            else
            {
                _no_hit_count++;
            }
        }
        _mean /= (points->points.size() - _no_hit_count);
        _mean_dist = _mean;
    }

    float Localizer::_calc_dist(const pcl::PointXYZ &src, const pcl::PointXYZ &dst)
    {
        float dx = dst.x - src.x;
        float dy = dst.y - src.y;
        return sqrtf(dx * dx + dy * dy);
    }

    void Localizer::init(ros::NodeHandle &nh)
    {
        this->n = nh;

        posePub = this->n.advertise<geometry_msgs::PoseStamped>("/slam_pose", 1);
        currPclPub = this->n.advertise<sensor_msgs::PointCloud2>("/robot/currpts", 1);
        testPclPub = this->n.advertise<sensor_msgs::PointCloud2>("/robot/tectpcl", 1);
    }

    Mapper::Mapper() : globalCloud(new pcl::PointCloud<pcl::PointXYZ>)
    {
    }

    SLAMCore::SLAMCore(ros::NodeHandle &nh) : localizer(std::make_unique<Localizer>()),
                                              mapper(std::make_unique<Mapper>())
    {
        laserSub = nh.subscribe("/robot/scan", 1, &SLAMCore::laserCb, this);
        pclPub = nh.advertise<sensor_msgs::PointCloud2>("/robot/points", 1);

        this->localizer->init(nh);

        genericHeader.frame_id = "laser";
        genericHeader.seq = __seqNum__;
        genericHeader.stamp = ros::Time::now();

        ros::Rate spinRate(10);
        while (ros::ok())
        {
            __seqNum__++;
            genericHeader.stamp = ros::Time::now();
            ros::spinOnce();
            spinRate.sleep();
        }
    }

    void SLAMCore::laserCb(const sensor_msgs::LaserScanConstPtr rawScan)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr rawCloud(new pcl::PointCloud<pcl::PointXYZ>);
        // convert laser scan to pointcloud
        scanToPcl(rawScan, rawCloud);

        localizer->update(rawCloud, mapper->globalCloud);

        sensor_msgs::PointCloud2 cloudMsg;
        pcl::toROSMsg(*mapper->globalCloud, cloudMsg);
        cloudMsg.header = genericHeader;
        pclPub.publish(cloudMsg);
    }

    void SLAMCore::scanToPcl(const sensor_msgs::LaserScanConstPtr originScan,
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr dstPcl)
    {
        dstPcl->header.frame_id = "laser";
        dstPcl->header.stamp = ros::Time::now().sec;

        float minPhi = originScan->angle_min;
        float maxPhi = originScan->angle_max;
        float dPhi = originScan->angle_increment;

        float currPhi = minPhi;
        for (float range : originScan->ranges)
        {
            if (range > 100)
            {
                currPhi += dPhi;
                continue;
            }
            // convert polar to cartesian
            pcl::PointXYZ newPt;
            newPt.x = range * cos(currPhi);
            newPt.y = range * sin(currPhi);

            dstPcl->points.push_back(newPt);

            currPhi += dPhi;
        }
    }

    void SLAMCore::publish_all()
    {
    }

} // namespace mapping