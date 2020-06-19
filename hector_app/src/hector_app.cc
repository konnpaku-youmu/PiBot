#include "../include/hector_app/hector_app.h"

namespace hector_app
{
    App::App(ros::NodeHandle &_nodehandle)
        : _vc(std::make_unique<VehicleController>()),
          _rm(std::make_unique<RouteManager>())
    {
        this->_vc->setController(_nodehandle);
        this->_rm->setRouteManager(_nodehandle);

        ros::Rate __loop__(20);
        while (ros::ok())
        {
            ros::spinOnce();
            __loop__.sleep();
        }

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

    void RouteManager::_record_new_route(const geometry_msgs::PoseStampedConstPtr _pose_ptr)
    {
        switch (this->_R_FLAG)
        {
        case START:
            this->_R_FLAG = REC;
        case REC:
            std::cout << "Recording" << std::endl;
            this->_new_route_container.header = _pose_ptr->header;
            this->_new_route_container.poses.push_back(_pose_ptr->pose);
            if(_pose_ptr->pose.orientation.w == 100 && _pose_ptr->pose.orientation.x == -100)
            {
                this->_R_FLAG = STOP;
            }
            break;
        case STOP:
            this->_network.push_back(this->_new_route_container);
            this->_new_route_container.poses.clear();
            this->_R_FLAG = START;
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

        try
        {
            geometry_msgs::PoseArray __requested_route__ = this->_network[__route_id__];
            __requested_route__.header.stamp = ros::Time::now();
            this->_route_dispenser.publish(__requested_route__);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void RouteManager::_load_route_file()
    {
        std::ifstream __lane_csv__(_DEFAULT_ROUTE_PATH);
        std::cout << _DEFAULT_ROUTE_PATH << std::endl;
        try
        {
            std::string __coord__;
            while(std::getline(__lane_csv__, __coord__))
            {
                std::cout << __coord__ << std::endl;
            }
        }
        catch(const std::exception& e)
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

        // link to
        this->_record_pose_relay = this->_nh->advertise<geometry_msgs::PoseStamped>("/pose_record", 1);
        this->_task_route_sub = this->_nh->subscribe("/task", 1, &VehicleController::_task_route_cb, this);
        this->_route_request_pub = this->_nh->advertise<std_msgs::Int16>("/route_request", 1);

        this->_curr_task_route.poses.clear();

        return;
    }

    void VehicleController::_joy_command_cb(const sensor_msgs::JoyConstPtr _joy_ptr)
    {
        this->_ps3_input.cross = _joy_ptr->buttons[0];
        this->_ps3_input.circle = _joy_ptr->buttons[1];
        this->_ps3_input.triangle = _joy_ptr->buttons[2];
        this->_ps3_input.square = _joy_ptr->buttons[3];

        this->_ps3_input.L_stick_x = _joy_ptr->axes[0];
        this->_ps3_input.L_stick_y = _joy_ptr->axes[1];
        this->_ps3_input.L2 = _joy_ptr->axes[2];
        this->_ps3_input.R_stick_x = _joy_ptr->axes[3];
        this->_ps3_input.R_stick_y = _joy_ptr->axes[4];
        this->_ps3_input.R2 = _joy_ptr->axes[5];

        if (this->_ps3_input.triangle)
        {
            this->_FLAG = MANUAL;
        }
        else if (this->_ps3_input.circle)
        {
            this->_FLAG = TASK;
        }
        else if (this->_ps3_input.square)
        {
            this->_FLAG = PATH_RECORD;
        }
        else if (this->_ps3_input.cross)
        {
            this->_FLAG = IDLE;
        }
    }

    void VehicleController::_slam_pose_cb(const geometry_msgs::PoseStampedConstPtr _pose_ptr)
    {
        switch (this->_FLAG)
        {
        case MANUAL:
            std::cout << "Switch to Manual Control" << std::endl;
            this->_use_remote_control();
            break;
        case TASK:
            break;
        case PATH_RECORD:
            this->_record_curr_path(_pose_ptr);
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
        return;
    }

    void VehicleController::_task_route_cb(const geometry_msgs::PoseArrayConstPtr _route_ptr)
    {
        this->_curr_task_route = *_route_ptr;
    }

    void VehicleController::_run_task()
    {
        // request route
        std_msgs::Int16 __route_request__;
        __route_request__.data = 0x0000;
        this->_route_request_pub.publish(__route_request__);
        try
        {
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }
} // namespace hector_app
