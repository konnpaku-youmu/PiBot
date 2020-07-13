#include "../include/hector_app/hector_app.h"

namespace std
{
    template <>
    struct hash<std::ifstream>
    {
        std::size_t operator()(std::ifstream &_file) const
        {
            std::string __line__;
            std::size_t __hash__ = 0;

            while (std::getline(_file, __line__))
            {
                std::size_t __h_tmp__ = std::hash<std::string>()(__line__);
                __hash__ = __hash__ ^ (__h_tmp__ << 1);
            }

            return __hash__;
        }
    };
} // namespace std

namespace hector_app
{
    App::App(ros::NodeHandle &_nodehandle)
        : _vc(std::make_unique<VehicleController>()),
          _rm(std::make_unique<RouteManager>()),
          _mm(std::make_unique<MapManager>())
    {
        system("clear");
        this->_vc->setController(_nodehandle);
        this->_rm->setRouteManager(_nodehandle);
        this->_mm->setMapManager(_nodehandle);

        ros::Rate __loop__(20);
        while (ros::ok())
        {
            ros::spinOnce();

            this->_rm->refreshRouteFile();

            __loop__.sleep();
        }

        return;
    }

    void MapManager::setMapManager(ros::NodeHandle &_nodehandle)
    {
        this->_nh = boost::make_shared<ros::NodeHandle>(_nodehandle);

        this->_map_cmd_sub = this->_nh->subscribe("/map_cmd", 1, &MapManager::_save_map, this);
        this->_map_data_sub = this->_nh->subscribe("/map", 1, &MapManager::_map_cb, this);

        return;
    }

    void MapManager::_map_cb(const nav_msgs::OccupancyGridConstPtr _map_ptr)
    {
        this->_map = *_map_ptr;

        return;
    }

    void MapManager::_save_map(const std_msgs::Int16ConstPtr _cmd_ptr)
    {
        if (_cmd_ptr->data == 0x0001)
        {
            size_t __map_h__ = this->_map.info.height;
            size_t __map_w__ = this->_map.info.width;

            // save map
            cv::Mat __map_img__ = cv::Mat::zeros(__map_h__, __map_w__, CV_8UC1);
            for (auto _grid = this->_map.data.begin(); _grid != this->_map.data.end(); ++_grid)
            {
                size_t __idx__ = _grid - this->_map.data.begin();
                int __row__ = __idx__ / __map_w__;
                int __col__ = __idx__ % __map_w__;

                int8_t __map_val_recast__ = 64;
                if (*_grid == 0)
                {
                    __map_val_recast__ = 0;
                }
                else if (*_grid == -1)
                {
                    __map_val_recast__ = 128;
                }
                else if (*_grid == 100)
                {
                    __map_val_recast__ = 255;
                }

                __map_img__.at<int8_t>(__row__, __col__) = __map_val_recast__;
            }
            cv::flip(__map_img__, __map_img__, 0);
            cv::medianBlur(__map_img__, __map_img__, 3);

            cv::imwrite("/home/hcrd/Projects/PiBot/utils/lane_marker/map_view.bmp", __map_img__);
        }
        std::cout << "Map saved as image" << std::endl;

        return;
    }

    RouteManager::RouteManager()
    {
    }

    void RouteManager::setRouteManager(ros::NodeHandle &_nodehandle)
    {
        this->_nh = boost::make_shared<ros::NodeHandle>(_nodehandle);

        this->_pose_recorder_sub = this->_nh->subscribe("/pose_record", 1, &RouteManager::_record_new_route, this);
        this->_request_sub = this->_nh->subscribe("/route_request", 1, &RouteManager::_request_listener, this);
        this->_route_dispenser = this->_nh->advertise<geometry_msgs::PoseArray>("/task", 1);

        this->_R_FLAG = START;
        this->_load_route_file();

        this->_new_route_container.poses.clear();
    }

    void RouteManager::refreshRouteFile()
    {
        this->_load_route_file();
        return;
    }

    void RouteManager::_record_new_route(const geometry_msgs::PoseStampedConstPtr _pose_ptr)
    {
        switch (this->_R_FLAG)
        {
        case START:
            this->_R_FLAG = REC;
        case REC:
            if (_pose_ptr->pose.orientation.w >= 99.9)
            {
                this->_R_FLAG = STOP;
                std::cout << "Waypoints: " << this->_new_route_container.poses.size() << std::endl;
                this->_network.push_back(this->_new_route_container);
                this->_new_route_container.poses.clear();
                std::cout << "Routes: " << this->_network.size() << std::endl;
                std::cout << "Stop recording" << std::endl;
            }
            else
            {
                if (this->_new_route_container.poses.empty())
                {
                    this->_new_route_container.header = _pose_ptr->header;
                    this->_new_route_container.poses.push_back(_pose_ptr->pose);
                }
                else
                {
                    double __dist__ = sqrt(pow(_pose_ptr->pose.position.x - this->_new_route_container.poses.back().position.x, 2) +
                                           pow(_pose_ptr->pose.position.y - this->_new_route_container.poses.back().position.y, 2));
                    if (__dist__ > 0.5)
                    {
                        this->_new_route_container.header = _pose_ptr->header;
                        this->_new_route_container.poses.push_back(_pose_ptr->pose);
                    }
                }
            }
            break;
        case STOP:
            break;
        default:
            break;
        }

        return;
    }

    void RouteManager::_request_listener(const std_msgs::Int16ConstPtr _request_ptr)
    {
        int __vehicle_id__ = (_request_ptr->data >> 8) && 0x00ff;
        int __route_id__ = _request_ptr->data && 0x00ff;

        std::cout << "Routes: " << this->_network.size() << std::endl;

        try
        {
            // send route
            geometry_msgs::PoseArray __requested_route__ = this->_network[__route_id__];
            __requested_route__.header.stamp = ros::Time::now();
            this->_route_dispenser.publish(__requested_route__);
            std::cout << "route published, length: " << __requested_route__.poses.size() << std::endl;
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void RouteManager::_load_route_file()
    {
        std::ifstream __hash_csv__(_DEFAULT_ROUTE_PATH);

        // check for modification on route file
        std::hash<std::ifstream> __hash_lane__;
        std::size_t __hash__ = __hash_lane__(__hash_csv__);
        if (__hash__ != this->_route_hash)
        {
            // modification detected, (re)load route file
            std::cout << "Loading route from " << _DEFAULT_ROUTE_PATH << std::endl;

            this->_route_hash = __hash__;
            // std::cout << this->_route_hash << std::endl;

            geometry_msgs::PoseArray __route_in_file__;

            try
            {
                std::string __coord__;
                std::ifstream __lane_csv__(_DEFAULT_ROUTE_PATH);

                while (std::getline(__lane_csv__, __coord__))
                {
                    geometry_msgs::Pose __waypoint__;

                    size_t __split__ = std::find(__coord__.begin(), __coord__.end(), ',') - __coord__.begin();

                    double _u = std::stod(__coord__.substr(0, __split__));
                    double _v = std::stod(__coord__.substr(__split__ + 1, __coord__.size()));

                    __waypoint__.position.x = (_u - MAP_SIZE / 2) * MAP_RES;
                    __waypoint__.position.y = (MAP_SIZE / 2 - _v) * MAP_RES;

                    __route_in_file__.poses.push_back(__waypoint__);
                }

                if (this->_network.empty())
                {
                    this->_network.push_back(__route_in_file__);
                }
                else
                {
                    this->_network[0] = __route_in_file__;
                }
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
            }
        }
        else
        {
            // do nothing
        }

        return;
    }

    VehicleController::VehicleController()
    {
        this->_timestamp = ros::Time::now().toSec();
        this->_err_a_diff = 0;
        this->_prev_err_a = 0;
        this->_err_a_int = 0;
    }

    void VehicleController::setController(ros::NodeHandle &_nodehandle)
    {
        this->_nh = boost::make_shared<ros::NodeHandle>(_nodehandle);
        this->_FLAG = IDLE;

        // subscribe to HectorSLAM localization result
        this->_vehicle_pose_sub = this->_nh->subscribe("/slam_out_pose", 1, &VehicleController::_slam_pose_cb, this);

        this->_raw_scan_sub = this->_nh->subscribe("/robot/laser", 1, &VehicleController::_raw_scan_cb, this);
        // subscribe to joystick input
        this->_joy_command_sub = this->_nh->subscribe("/joy", 1, &VehicleController::_joy_command_cb, this);
        // setup joystick publisher
        this->_remote_cmd_pub = this->_nh->advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1);

        // link to map manager
        this->_map_ctrl_pub = this->_nh->advertise<std_msgs::Int16>("/map_cmd", 1);

        // link to route manager
        this->_record_pose_relay = this->_nh->advertise<geometry_msgs::PoseStamped>("/pose_record", 1);
        this->_task_route_sub = this->_nh->subscribe("/task", 1, &VehicleController::_task_route_cb, this);
        this->_route_request_pub = this->_nh->advertise<std_msgs::Int16>("/route_request", 1);

        // visualization publisher
        this->_route_vis_pub = this->_nh->advertise<visualization_msgs::Marker>("/route", 10);
        this->_traj_vis_pub = this->_nh->advertise<visualization_msgs::Marker>("/traj", 10);

        this->_curr_task_route.clear();

        return;
    }

    void VehicleController::_joy_command_cb(const sensor_msgs::JoyConstPtr _joy_ptr)
    {
        // RC
        this->_ps3_input.cross = _joy_ptr->buttons[0];
        this->_ps3_input.circle = _joy_ptr->buttons[1];
        this->_ps3_input.triangle = _joy_ptr->buttons[2];
        this->_ps3_input.square = _joy_ptr->buttons[3];

        this->_ps3_input.start = _joy_ptr->buttons[9];
        this->_ps3_input.R1 = _joy_ptr->buttons[5];

        this->_ps3_input.L_stick_x = _joy_ptr->axes[0];
        this->_ps3_input.L_stick_y = _joy_ptr->axes[1];
        this->_ps3_input.L2 = _joy_ptr->axes[2];
        this->_ps3_input.R_stick_x = _joy_ptr->axes[3];
        this->_ps3_input.R_stick_y = _joy_ptr->axes[4];
        this->_ps3_input.R2 = _joy_ptr->axes[5];

        if (this->_ps3_input.triangle)
        {
            std::cout << "Switch to Manual Control..." << std::endl;
            this->_FLAG = MANUAL;
        }
        else if (this->_ps3_input.circle)
        {
            std::cout << "Running Task..." << std::endl;
            this->_FLAG = TASK;
        }
        else if (this->_ps3_input.square)
        {
            std::cout << "Recording Path..." << std::endl;
            this->_FLAG = PATH_RECORD;
        }
        else if (this->_ps3_input.start)
        {
            std::cout << "Recording Map..." << std::endl;
            this->_FLAG = MAP_RECORD;
        }
        else if (this->_ps3_input.cross && !this->_ps3_input.square)
        {
            this->_FLAG = IDLE;
        }
    }

    void VehicleController::_raw_scan_cb(const sensor_msgs::LaserScanConstPtr _scan_ptr)
    {
        int __leftmost_range__, __rightmost_range__;
        __leftmost_range__ = M_PI_4;
    }

    void VehicleController::_slam_pose_cb(const geometry_msgs::PoseStampedConstPtr _pose_ptr)
    {
        switch (this->_FLAG)
        {
        case MANUAL:
            this->_use_remote_control();
            break;
        case TASK:
            this->_run_task(_pose_ptr);
            break;
        case PATH_RECORD:
            this->_record_curr_path(_pose_ptr);
            break;
        case MAP_RECORD:
            this->_draw_map();
            break;
        default:
            this->_curr_task_route.clear();
            break;
        }

        return;
    }

    void VehicleController::_use_remote_control()
    {
        float __linear_velocity__ = this->_ps3_input.L_stick_y;
        float __angular_velocity__ = this->_ps3_input.R_stick_x;

        geometry_msgs::Twist __cmd_vel__;

        __cmd_vel__.linear.x = exp(0.18 * __linear_velocity__) - 1;
        __cmd_vel__.angular.z = (__angular_velocity__ > 0) ? exp(0.3 * __angular_velocity__) - 1 : 1 - exp(-0.3 * __angular_velocity__);

        // publish control commands
        this->_remote_cmd_pub.publish(__cmd_vel__);

        return;
    }

    void VehicleController::_record_curr_path(const geometry_msgs::PoseStampedConstPtr _curr_pose_ptr)
    {
        this->_use_remote_control();
        // bypass current pose to trigger RouteManager::record
        this->_record_pose_relay.publish(_curr_pose_ptr);

        if (this->_ps3_input.R1)
        {
            geometry_msgs::PoseStamped _terminate_pose;
            _terminate_pose.header = _curr_pose_ptr->header;
            _terminate_pose.pose.orientation.w = 100;
            _terminate_pose.pose.orientation.x = -100;
            this->_record_pose_relay.publish(_terminate_pose);
            this->_FLAG = IDLE;
        }

        return;
    }

    void VehicleController::_draw_map()
    {
        this->_use_remote_control();
        // send save request
        if (this->_ps3_input.R1)
        {
            std_msgs::Int16 __map_cmd__;
            __map_cmd__.data = 0x0001;
            this->_map_ctrl_pub.publish(__map_cmd__);
        }
    }

    void VehicleController::_task_route_cb(const geometry_msgs::PoseArrayConstPtr _route_ptr)
    {
        for (auto _pose : _route_ptr->poses)
        {
            Eigen::Vector3d __way_pt__ = Eigen::Vector3d::Zero();
            __way_pt__[0] = _pose.position.x;
            __way_pt__[1] = _pose.position.y;

            this->_route_vis.points.push_back(_pose.position);

            _curr_task_route.push_back(__way_pt__);
        }

        return;
    }

    void VehicleController::_run_task(const geometry_msgs::PoseStampedConstPtr _pose_ptr)
    {
        // request route

        if (_curr_task_route.size() == 0)
        {
            std_msgs::Int16 __route_request__;
            __route_request__.data = 0x0001;
            this->_route_request_pub.publish(__route_request__);
            std::cout << "Request task" << std::endl;
            std::cout << "Route Length: " << this->_curr_task_route.size() << std::endl;

            if (!this->_curr_task_route.empty())
            {
            }
            else
            {
                return;
            }
        }

        try
        {
            Eigen::Quaterniond _q(_pose_ptr->pose.orientation.w,
                                  _pose_ptr->pose.orientation.x,
                                  _pose_ptr->pose.orientation.y,
                                  _pose_ptr->pose.orientation.z);
            Eigen::Matrix3d _R(_q);
            Eigen::Vector3d _t = Eigen::Vector3d::Zero();

            _t[0] = _pose_ptr->pose.position.x;
            _t[1] = _pose_ptr->pose.position.y;

            Eigen::Vector3d _dst = _curr_task_route.front();
            Eigen::Vector3d _dst_body = _R.transpose() * (_dst - _t);

            geometry_msgs::Twist __output__;

            // PID Control -- Point
            {
                double __dt__ = ros::Time::now().toSec() - this->_timestamp;

                if (__dt__ > 0.2)
                {
                    __dt__ = 0.2;
                }

                double __dir__ = _dst_body.dot(Eigen::Vector3d(1, 0, 0));

                double __error_a__, __error_l__;

                double _kp_angular = 1.5, _ki_angular = 0.05, _kd_angular = -0.4;
                double _kp_linear = 0.25;

                if (__dir__ >= 0)
                {
                    __error_a__ = atan(_dst_body[1] / _dst_body[0]) / M_PI;
                    __error_l__ = _dst_body.norm();
                }
                else
                {
                    __error_a__ = -atan(_dst_body[1] / fabs(_dst_body[0])) / M_PI;
                    __error_l__ = -_dst_body.norm();
                }

                _err_a_int += __error_a__ * __dt__;
                _err_a_diff = (__error_a__ - _prev_err_a) / __dt__;
                __output__.angular.z = _kp_angular * __error_a__ + _ki_angular * _err_a_int + _kd_angular * _err_a_diff;

                if (__output__.angular.z >= 0.7)
                {
                    __output__.angular.z = 0.7;
                }
                else if (__output__.angular.z <= -0.7)
                {
                    __output__.angular.z = -0.7;
                }

                if (__dir__ >= 0)
                {
                    __output__.linear.x = 0.3 - 0.4 * fabs(__error_a__);
                }
                else
                {
                    __output__.linear.x = -0.1;
                }

                if (fabs(__error_l__) <= 0.2)
                {
                    Eigen::Vector3d __last_pt__ = this->_curr_task_route.front();
                    this->_curr_task_route.pop_front();
                    this->_curr_task_route.push_back(__last_pt__);

                    this->_err_a_int = 0;

                    if (_curr_task_route.empty())
                    {
                        std::cout << "Arrival" << std::endl;
                        this->_route_vis.points.clear();
                        this->_FLAG = IDLE;
                    }
                }

                this->_timestamp = ros::Time::now().toSec();
                this->_prev_err_a = __error_a__;
            }
            // PID Control -- Line
            {
            }

            this->_remote_cmd_pub.publish(__output__);

            {
                this->_traj_vis.header.stamp = ros::Time::now();
                this->_traj_vis.header.frame_id = "map";
                this->_traj_vis.action = visualization_msgs::Marker::ADD;
                this->_traj_vis.type = visualization_msgs::Marker::LINE_STRIP;
                this->_traj_vis.pose.orientation.w = 1.0;
                this->_traj_vis.ns = "points_and_lines";
                this->_traj_vis.id = 1;
                this->_traj_vis.scale.x = 0.03;
                this->_traj_vis.color.b = 0.7;
                this->_traj_vis.color.a = 1.0;

                this->_traj_vis.points.push_back(_pose_ptr->pose.position);

                this->_traj_vis_pub.publish(this->_traj_vis);

                this->_route_vis.header.stamp = ros::Time::now();
                this->_route_vis.header.frame_id = "map";
                this->_route_vis.action = visualization_msgs::Marker::ADD;
                this->_route_vis.type = visualization_msgs::Marker::LINE_STRIP;
                this->_route_vis.pose.orientation.w = 1.0;
                this->_route_vis.ns = "points_and_lines";
                this->_route_vis.id = 2;
                this->_route_vis.scale.x = 0.05;
                this->_route_vis.color.r = 0.7;
                this->_route_vis.color.a = 1.0;
                this->_route_vis_pub.publish(this->_route_vis);
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

        if (this->_ps3_input.R1)
        {
            this->_curr_task_route.clear();
            this->_FLAG = IDLE;
        }

        return;
    }
} // namespace hector_app
