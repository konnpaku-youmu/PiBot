#ifndef HECTOR_APP_H
#define HECTOR_APP_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

namespace hector_app
{
    typedef std::vector<geometry_msgs::PoseArray> RouteNetwork;

    enum CONTROL_FLAG
    {
        MANUAL,
        TASK,
        EXPLORE,
        PATH_RECORD,
        IDLE
    };

    struct JoystickInput
    {
        bool L1;
        bool R1;

        bool left;
        bool right;
        bool up;
        bool down;

        bool select;
        bool start;
        bool ps;

        bool square;
        bool triangle;
        bool circle;
        bool cross;

        double L2;
        double R2;

        double L_stick_x;
        double L_stick_y;
        double R_stick_x;
        double R_stick_y;

        JoystickInput()
        {
            L1 = R1 = left = right = up = down = select = start =
                ps = square = triangle = circle = cross = false;

            L2 = R2 = L_stick_x = R_stick_x = L_stick_y = R_stick_y = 0.0;
        }
    };

    class MapManager
    {
    private:
        /* data */
    public:
        MapManager() {}

        ~MapManager() { return; }
    };

    class RouteManager
    {
    private:
        ros::NodeHandlePtr _nh;

        ros::Subscriber _pose_recorder_sub;

        ros::Subscriber _request_sub;

        ros::Publisher _route_dispenser;

        geometry_msgs::PoseArray _new_route_container;

        RouteNetwork _network;

        void _record_new_route(const geometry_msgs::PoseStampedConstPtr);

        void _request_listener(const std_msgs::Int16ConstPtr);
        
        void _load_route_file();

    public:
        RouteManager();

        ~RouteManager() { return; }

        void setRouteManager(ros::NodeHandle &);
    };

    class VehicleController
    {
    private:
        ros::NodeHandlePtr _nh;

        ros::Subscriber _vehicle_pose_sub;

        ros::Subscriber _joy_command_sub;

        ros::Subscriber _task_route_sub;

        ros::Publisher _remote_cmd_pub;

        ros::Publisher _record_pose_relay;

        ros::Publisher _route_request_pub;

        JoystickInput _ps3_input;

        geometry_msgs::PoseArray _curr_task_route;

        CONTROL_FLAG _FLAG;

        void _slam_pose_cb(const geometry_msgs::PoseStampedConstPtr);

        void _joy_command_cb(const sensor_msgs::JoyConstPtr);

        void _use_remote_control();

        void _record_curr_path(const geometry_msgs::PoseStampedConstPtr);

        void _task_route_cb(const geometry_msgs::PoseArrayConstPtr);

        void _run_task();

    public:
        VehicleController();

        ~VehicleController() { return; }

        void setController(ros::NodeHandle &);
    };

    class App
    {
    private:
        std::unique_ptr<VehicleController> _vc;

        std::unique_ptr<RouteManager> _rm;

    public:
        App(ros::NodeHandle &);

        ~App() { return; }
    };

} // namespace hector_app

#endif
