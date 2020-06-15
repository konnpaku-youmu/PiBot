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
        this->_request_pub = this->_nh->advertise<geometry_msgs::PoseArray>("/task", 1);
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

        // subscribe to route manager


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
            this->_FLAG = EXPLORE;
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
            std::cout << "Controlled by joystick" << std::endl;
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
        
        return;
    }

    void VehicleController::_run_task()
    {
    }
} // namespace hector_app