#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

namespace robot_control
{
    class Controller
    {
    private:
        ros::Subscriber _pos_sub;

        ros::Publisher _control_pub;

        ros::Publisher _traj_pub;

        Eigen::Vector3d _dst = Eigen::Vector3d(2,0,0);

        std::vector<Eigen::Vector3d> _route;

        visualization_msgs::Marker _traj;

        void _pos_callback(const geometry_msgs::PoseStampedConstPtr);

        void _set_goal(const geometry_msgs::PoseStampedConstPtr);

    public:
        Controller(ros::NodeHandle &);

        ~Controller() { return; }
    };

} // namespace robot_control

#endif