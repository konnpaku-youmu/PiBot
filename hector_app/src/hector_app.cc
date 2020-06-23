#include "../include/hector_app/hector_app.h"

namespace hector_app
{
    App::App(ros::NodeHandle &_nodehandle)
        : _vc(std::make_unique<VehicleController>()),
          _rm(std::make_unique<RouteManager>()),
          _mm(std::make_unique<MapManager>())
    {
        this->_vc->setController(_nodehandle);
        this->_rm->setRouteManager(_nodehandle);
        this->_mm->setMapManager(_nodehandle);

        ros::Rate __loop__(20);
        while (ros::ok())
        {
            ros::spinOnce();
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
            cv::rotate(__map_img__, __map_img__, cv::ROTATE_180);
            cv::medianBlur(__map_img__, __map_img__, 3);

            cv::imwrite("/home/hcrd/PiBot/utils/lane_marker/map_view.bmp", __map_img__);
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
        // this->_load_route_file();

        this->_new_route_container.poses.clear();
    }

    void RouteManager::_record_new_route(const geometry_msgs::PoseStampedConstPtr _pose_ptr)
    {
        switch (this->_R_FLAG)
        {
        case START:
            this->_R_FLAG = REC;
        case REC:
            if (_pose_ptr->pose.orientation.w == 100 && _pose_ptr->pose.orientation.x == -100)
            {
                this->_R_FLAG = STOP;
            }
            else
            {
                this->_new_route_container.header = _pose_ptr->header;
                this->_new_route_container.poses.push_back(_pose_ptr->pose);
            }
            break;
        case STOP:
            std::cout << "Waypoints: " << this->_new_route_container.poses.size() << std::endl;
            this->_network.push_back(this->_new_route_container);
            this->_new_route_container.poses.clear();
            std::cout << "Routes: " << this->_network.size() << std::endl;
            std::cout << "Stop recording" << std::endl;
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
            geometry_msgs::PoseArray __requested_route__ = this->_network[0];
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
        std::ifstream __lane_csv__(_DEFAULT_ROUTE_PATH);
        geometry_msgs::PoseArray __route_in_file__;

        try
        {
            std::string __coord__;
            while (std::getline(__lane_csv__, __coord__))
            {
                geometry_msgs::Pose __waypoint__;

                size_t __split__ = std::find(__coord__.begin(), __coord__.end(), ',') - __coord__.begin();

                double _u = std::stod(__coord__.substr(0, __split__));
                double _v = std::stod(__coord__.substr(__split__ + 1, __coord__.size()));

                __waypoint__.position.x = (_u - MAP_SIZE / 2) * MAP_RES;
                __waypoint__.position.y = (_v - MAP_SIZE / 2) * MAP_RES;

                __route_in_file__.poses.push_back(__waypoint__);
            }

            this->_network.push_back(__route_in_file__);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    VehicleController::VehicleController()
    {
    }

    void VehicleController::setController(ros::NodeHandle &_nodehandle)
    {
        this->_nh = boost::make_shared<ros::NodeHandle>(_nodehandle);
        this->_FLAG = IDLE;

        // subscribe to HectorSLAM localization result
        this->_vehicle_pose_sub = this->_nh->subscribe("/slam_out_pose", 1, &VehicleController::_slam_pose_cb, this);

        // subscribe to joystick input
        this->_joy_command_sub = this->_nh->subscribe("/joy", 1, &VehicleController::_joy_command_cb, this);
        // setup joystick publisher
        this->_remote_cmd_pub = this->_nh->advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1);

        this->_map_ctrl_pub = this->_nh->advertise<std_msgs::Int16>("/map_cmd", 1);
        // link to route manager
        this->_record_pose_relay = this->_nh->advertise<geometry_msgs::PoseStamped>("/pose_record", 1);
        this->_task_route_sub = this->_nh->subscribe("/task", 1, &VehicleController::_task_route_cb, this);
        this->_route_request_pub = this->_nh->advertise<std_msgs::Int16>("/route_request", 1);

        this->_curr_task_route.poses.clear();

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
        this->_curr_task_route = *_route_ptr;
    }

    void VehicleController::_run_task(const geometry_msgs::PoseStampedConstPtr _pose_ptr)
    {
        // request route

        if (_curr_task_route.poses.size() == 0)
        {
            std_msgs::Int16 __route_request__;
            __route_request__.data = 0x0000;
            this->_route_request_pub.publish(__route_request__);
            std::cout << "Request task" << std::endl;
            std::cout << "Route Length: " << this->_curr_task_route.poses.size() << std::endl;
        }

        try
        {
            std::vector<Eigen::Vector3d> __route_wrt_body__;

            Eigen::Quaterniond _q(_pose_ptr->pose.orientation.w,
                                  _pose_ptr->pose.orientation.w,
                                  _pose_ptr->pose.orientation.y,
                                  _pose_ptr->pose.orientation.z);
            Eigen::Matrix3d _R(_q);
            Eigen::Vector3d _t = Eigen::Vector3d::Ones();

            _t[0] = _pose_ptr->pose.position.x;
            _t[1] = _pose_ptr->pose.position.y;

            // transform to body frame
            for (auto _way_pt : this->_curr_task_route.poses)
            {
                Eigen::Vector3d __way_pt__ = Eigen::Vector3d::Ones();
                __way_pt__[0] = _way_pt.position.x;
                __way_pt__[1] = _way_pt.position.y;

                Eigen::Vector3d _way_pt_t = _R.inverse() * (__way_pt__ - _t);

                __route_wrt_body__.push_back(_way_pt_t);
            }

            // find nearest intercept waypoint
            std::vector<int> __indices__;
            for (auto _way_pt = __route_wrt_body__.begin(); _way_pt != __route_wrt_body__.end(); ++_way_pt)
            {
                double _dist = std::sqrt(std::pow(_way_pt->x() - _t.x(), 2) + std::pow(_way_pt->y() - _t.y(), 2));
                if (_dist <= 1.0)
                {
                    __indices__.push_back(_way_pt - __route_wrt_body__.begin());
                }
            }
            int _pt_idx = *std::max_element(__indices__.begin(), __indices__.end());

            std::list<Eigen::Vector3d> _trunc_route(&__route_wrt_body__[_pt_idx], &__route_wrt_body__.back());

            Eigen::Vector3d _dst_body = _trunc_route.front();
            double __dir__ = _dst_body.dot(Eigen::Vector3d(1, 0, 0));

            double _angular_e = (__dir__ > 0) ? atan(_dst_body[1] / _dst_body[0]) : -atan(_dst_body[1] / fabs(_dst_body[0]));
            double _linear_e = (__dir__ > 0) ? _dst_body.norm() : -_dst_body.norm();
            double _kp_angular = 0.35;
            double _kp_linear = 0.32;

            geometry_msgs::Twist _output;
            _output.linear.x = (__dir__ > 0) ? 0.15 : -0.15;
            _output.angular.z = _kp_angular * _angular_e;
            if (fabs(_linear_e) > 0.1)
            {
            }
            else
            {
                Eigen::Vector3d _tmp = _trunc_route.front();
                _trunc_route.pop_front();
                _trunc_route.push_back(_tmp);
            }

            this->_remote_cmd_pub.publish(_output);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

        return;
    }
} // namespace hector_app
