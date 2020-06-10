#include "../include/controller.h"

namespace robot_control
{
    Controller::Controller(ros::NodeHandle &_nh)
    {
        _pos_sub = _nh.subscribe("/slam_out_pose", 1, &Controller::_pos_callback, this);
        _control_pub = _nh.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1);

        ros::Rate loop(60);
        while (ros::ok())
        {
            ros::spinOnce();
            loop.sleep();
        }
    }

    void Controller::_pos_callback(const geometry_msgs::PoseStampedConstPtr _pose_ptr)
    {
        // transform _dst to robot frame
        Eigen::Quaterniond q(_pose_ptr->pose.orientation.w,
                             _pose_ptr->pose.orientation.x,
                             _pose_ptr->pose.orientation.y,
                             _pose_ptr->pose.orientation.z);
        Eigen::Matrix3d R(q);
        Eigen::Vector3d t(_pose_ptr->pose.position.x,
                          _pose_ptr->pose.position.y,
                          _pose_ptr->pose.position.z);

        Eigen::Vector3d _dst_body = R.transpose() * this->_dst - t;

        std::cout << _dst_body << std::endl;
    }
} // namespace robot_control