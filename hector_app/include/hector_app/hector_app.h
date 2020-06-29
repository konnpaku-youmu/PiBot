#ifndef HECTOR_APP_H
#define HECTOR_APP_H

#include <ros/ros.h>
#include <fstream>
#include <eigen3/Eigen/Eigen>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/opencv.hpp>

namespace hector_app
{
#define MAP_RES 0.05
#define MAP_SIZE 768

    typedef std::vector<geometry_msgs::PoseArray> RouteNetwork;

    enum CONTROL_FLAG
    {
        MANUAL,
        TASK,
        EXPLORE,
        PATH_RECORD,
        MAP_RECORD,
        IDLE
    };

    enum RECORD_FLAG
    {
        START,
        REC,
        STOP
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
        ros::NodeHandlePtr _nh;

        ros::Subscriber _map_data_sub;

        ros::Subscriber _map_cmd_sub;

        nav_msgs::OccupancyGrid _map;

        void _map_cb(const nav_msgs::OccupancyGridConstPtr);

        void _save_map(const std_msgs::Int16ConstPtr);

    public:
        MapManager() {}

        ~MapManager() { return; }

        void setMapManager(ros::NodeHandle &);
    };

    class RouteManager
    {
    private:
        ros::NodeHandlePtr _nh;

        ros::Subscriber _pose_recorder_sub;

        ros::Subscriber _request_sub;

        ros::Publisher _route_dispenser;

        RECORD_FLAG _R_FLAG;

        geometry_msgs::PoseArray _new_route_container;

        RouteNetwork _network;

        std::size_t _route_hash = 0;

        const std::string _DEFAULT_ROUTE_PATH{"/home/hcrd/PiBot/utils/lane_marker/lane.csv"};

        void _record_new_route(const geometry_msgs::PoseStampedConstPtr);

        void _request_listener(const std_msgs::Int16ConstPtr);

        void _load_route_file();

    public:
        RouteManager();

        ~RouteManager() { return; }

        void setRouteManager(ros::NodeHandle &);

        void refreshRouteFile();
    };

    class VehicleController
    {
    private:
        ros::NodeHandlePtr _nh;

        ros::Subscriber _vehicle_pose_sub;

        ros::Subscriber _joy_command_sub;

        ros::Subscriber _task_route_sub;

        ros::Publisher _remote_cmd_pub;

        ros::Publisher _map_ctrl_pub;

        ros::Publisher _record_pose_relay;

        ros::Publisher _route_request_pub;

        ros::Publisher _route_vis_pub;

        ros::Publisher _traj_vis_pub;

        JoystickInput _ps3_input;

        std::list<Eigen::Vector3d> _curr_task_route;

        CONTROL_FLAG _FLAG;

        visualization_msgs::Marker _route_vis;

        visualization_msgs::Marker _traj_vis;

        void _slam_pose_cb(const geometry_msgs::PoseStampedConstPtr);

        void _joy_command_cb(const sensor_msgs::JoyConstPtr);

        void _use_remote_control();

        void _record_curr_path(const geometry_msgs::PoseStampedConstPtr);

        void _draw_map();

        void _task_route_cb(const geometry_msgs::PoseArrayConstPtr);

        void _run_task(const geometry_msgs::PoseStampedConstPtr);

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

        std::unique_ptr<MapManager> _mm;

    public:
        App(ros::NodeHandle &);

        ~App() { return; }
    };

} // namespace hector_app

#endif
