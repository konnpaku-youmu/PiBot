#include "../include/controller.h"

namespace robot_control
{
    Controller::Controller(ros::NodeHandle &_nh)
    {
        _pos_sub = _nh.subscribe("/slam_out_pose", 1, &Controller::_pos_callback, this);
        _control_pub = _nh.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1);
        _traj_pub = _nh.advertise<visualization_msgs::Marker>("/traj", 1);

        _traj.action = visualization_msgs::Marker::ADD;
        _traj.type = visualization_msgs::Marker::LINE_STRIP;
        _traj.ns = "points_and_lines";
        _traj.id = 1;
        _traj.scale.x = 0.05;
        _traj.color.r = 0.7;
        _traj.color.b = 0.4;

        ros::Rate loop(60);
        while (ros::ok())
        {
            ros::spinOnce();
            loop.sleep();
        }
    }

    void Controller::_pos_callback(const geometry_msgs::PoseStampedConstPtr _pose_ptr)
    {
        _traj.header = _pose_ptr->header;
        _traj.points.push_back(_pose_ptr->pose.position);

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

        double _angular_e = atan(_dst_body[1] / _dst_body[0]);
        double _linear_e = _dst_body.norm();
        double _kp_angular = 1.5;
        double _kp_linear = 0.25;

        geometry_msgs::Twist _output;
        if (_linear_e > 0.05)
        {
            _output.linear.x = _kp_linear * +_linear_e;
            _output.angular.z = _kp_angular * _angular_e;
        }

        _control_pub.publish(_output);
        _traj_pub.publish(_traj);
    }
} // namespace robot_control