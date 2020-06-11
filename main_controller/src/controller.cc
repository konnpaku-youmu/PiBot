#include "../include/controller.h"

namespace robot_control
{
    Controller::Controller(ros::NodeHandle &_nh)
    {
        _pos_sub = _nh.subscribe("/slam_out_pose", 1, &Controller::_pos_callback, this);
        _control_pub = _nh.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1);
        _traj_pub = _nh.advertise<visualization_msgs::Marker>("traj", 10);
        _route_pub = _nh.advertise<visualization_msgs::Marker>("route", 10);

        _route.push_back(Eigen::Vector3d(0.7, 0.45, 0));
        _route.push_back(Eigen::Vector3d(0.8, 1, 0));
        _route.push_back(Eigen::Vector3d(0.8, 5.2, 0));
        _route.push_back(Eigen::Vector3d(-0.7, 5.5, 0));
        _route.push_back(Eigen::Vector3d(-1, 6, 0));
        _route.push_back(Eigen::Vector3d(-0.7, 6.6, 0));
        _route.push_back(Eigen::Vector3d(0, 6.4, 0));
        _route.push_back(Eigen::Vector3d(0.5, 6, 0));
        _route.push_back(Eigen::Vector3d(1, 5, 0));
        _route.push_back(Eigen::Vector3d(1, 1, 0));
        _route.push_back(Eigen::Vector3d(0.7, 0.5, 0));
        _route.push_back(Eigen::Vector3d(1.3, -0.5, 0));
        _route.push_back(Eigen::Vector3d(0, 0, 0));

        _rue.header.stamp = ros::Time::now();
        _rue.header.frame_id = "map";

        _rue.action = visualization_msgs::Marker::ADD;
        _rue.type = visualization_msgs::Marker::LINE_STRIP;
        _rue.pose.orientation.w = 1.0;
        _rue.ns = "points_and_lines";
        _rue.id = 2;
        _rue.scale.x = 0.05;
        _rue.color.b = 0.7;
        _rue.color.a = 1.0;

        for (auto _pt : _route)
        {
            geometry_msgs::Point _p;
            _p.x = _pt[0];
            _p.y = _pt[1];
            _rue.points.push_back(_p);
        }

        ros::Rate loop(30);
        while (ros::ok())
        {
            ros::spinOnce();
            _route_pub.publish(_rue);
            loop.sleep();
        }
    }

    void Controller::_pos_callback(const geometry_msgs::PoseStampedConstPtr _pose_ptr)
    {
        _traj.header.stamp = ros::Time::now();
        _traj.header.frame_id = "map";

        _traj.action = visualization_msgs::Marker::ADD;
        _traj.type = visualization_msgs::Marker::LINE_STRIP;
        _traj.pose.orientation.w = 1.0;
        _traj.ns = "points_and_lines";
        _traj.id = 1;
        _traj.scale.x = 0.05;
        _traj.color.r = 0.9;
        _traj.color.a = 1.0;

        _traj.points.push_back(_pose_ptr->pose.position);
        _traj_pub.publish(_traj);

        if (_route.empty())
        {
            return;
        }

        Eigen::Vector3d _dst = _route.front();

        // transform _dst to robot frame
        Eigen::Quaterniond q(_pose_ptr->pose.orientation.w,
                             _pose_ptr->pose.orientation.x,
                             _pose_ptr->pose.orientation.y,
                             _pose_ptr->pose.orientation.z);
        Eigen::Matrix3d R(q);
        Eigen::Vector3d t(_pose_ptr->pose.position.x,
                          _pose_ptr->pose.position.y,
                          _pose_ptr->pose.position.z);

        Eigen::Vector3d _dst_body = R.transpose() * (_dst - t);
        double direction = _dst_body.dot(Eigen::Vector3d(1, 0, 0));

        double _angular_e = (direction > 0) ? atan(_dst_body[1] / _dst_body[0]) : -atan(_dst_body[1] / fabs(_dst_body[0]));
        double _linear_e = (direction > 0) ? _dst_body.norm() : -_dst_body.norm();
        double _kp_angular = 0.35;
        double _kp_linear = 0.32;

        geometry_msgs::Twist _output;
        if (fabs(_linear_e) > 0.15)
        {
            _output.linear.x = (direction > 0) ? 0.15 : -0.15;

            // std::cout << _linear_e << std::endl;
            _output.angular.z = _kp_angular * _angular_e;
            // std::cout << _output.angular.z << std::endl;
        }
        else
        {
            Eigen::Vector3d _tmp = _route.front();
            _route.pop_front();
            _route.push_back(_tmp);
        }

        _control_pub.publish(_output);
    }
} // namespace robot_control