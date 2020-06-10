#include "../include/ps3_con/tele_con.h"

namespace ps3
{
    Joystick::Joystick(ros::NodeHandle &nh)
    {
        _joy_sub = nh.subscribe("/joy", 1, &Joystick::_js_input_cb, this);
        _twist_pub = nh.advertise<geometry_msgs::Twist>("/robot/cmd_vel", 1);
        _button_pub = nh.advertise<std_msgs::Int8MultiArray>("/robot/buttons", 1);
        _start_rp_motor_cli = nh.serviceClient<std_srvs::Empty>("/robot/start_motor");
        _stop_rp_motor_cli = nh.serviceClient<std_srvs::Empty>("/robot/stop_motor");

        ros::Rate loop(20);
        while (ros::ok())
        {
            // _js_ascii_art();
            this->_cmd_pub();
            ros::spinOnce();
            loop.sleep();
        }
    }

    void Joystick::_js_input_cb(const sensor_msgs::JoyConstPtr _raw_input_ptr)
    {
        // wheel control
        // read joy input
        float _linear_velocity = _raw_input_ptr->axes[1];
        float _angular_velocity = _raw_input_ptr->axes[3];

        uint8_t _dash = _raw_input_ptr->buttons[11];
        uint8_t _drift = _raw_input_ptr->buttons[12];

        // translate to Twist message
        _cmd_out.linear.x = exp(0.18 * (_dash + 1) * _linear_velocity) - 1;
        _cmd_out.angular.z = (_angular_velocity > 0) ? exp(0.3 * (_drift + 1) * _angular_velocity) - 1 : 1 - exp(0.3 * (_drift + 1) * -_angular_velocity);

        // read buttons input
        _l_pressed = _raw_input_ptr->buttons[4];
        _r_pressed = _raw_input_ptr->buttons[5];

        // L/R Key to turn on/off rplidar motor
        if (_l_pressed && !_r_pressed)
        {
            // spin lidar
            _rp_motor_cmd = 0;
        }
        else if (_r_pressed && !_l_pressed)
        {
            // stop spinning lidar
            _rp_motor_cmd = 1;
        }
        else
        {
            // keep current status
            _rp_motor_cmd = -1;
        }

        // function control
        for (auto _button : _raw_input_ptr->buttons)
        {
            _buttons_input.push_back((int8_t)_button);
        }

        return;
    }

    void Joystick::_cmd_pub()
    {
        // main motors controller
        _twist_pub.publish(_cmd_out);

        std_msgs::Int8MultiArray _buttons_msg;
        std_msgs::MultiArrayDimension _tmp;
        // _tmp.size = 15;
        _buttons_msg.layout.data_offset = 0;
        _buttons_msg.layout.dim.push_back(_tmp);
        _buttons_msg.data = this->_buttons_input;
        this->_button_pub.publish(_buttons_msg);

        // rplidar motor controller
        std_srvs::Empty _srv;
        if (_rp_motor_cmd == 0 && _rp_motor_status)
        {
            _stop_rp_motor_cli.call(_srv);
            _rp_motor_status = !_rp_motor_status;
        }
        else if (_rp_motor_cmd == 1 && !_rp_motor_status)
        {
            _start_rp_motor_cli.call(_srv);
            _rp_motor_status = !_rp_motor_status;
        }

        _buttons_input.clear();

        return;
    }

    void Joystick::_js_ascii_art()
    {
        const char *_clear_cmd = "clear";
        system(_clear_cmd);

        std::cout << "\033[0;33m" << std::endl;
        std::cout << "//////////////////////PS3 CONTROLLER//////////////////////" << std::endl;

        // interactive guide animation
        std::cout << "\033[1;32m" << std::endl;
        std::cout << "        ";
        std::cout << "\033[" << (_l_pressed == 0) + 1 << ";32m";
        std::cout << "L1:";
        std::cout << "\033[0m";
        std::cout << "\033[1;32m";
        std::cout << "                                   ";
        std::cout << "\033[" << (_r_pressed == 0) + 1 << ";32m";
        std::cout << "R1:" << std::endl;
        std::cout << "\033[0m";
        std::cout << "\033[1;32m";
        std::cout << "  ";
        std::cout << "\033[" << (_l_pressed == 0) + 1 << ";32m";
        std::cout << "Stop LiDAR Motor";
        std::cout << "\033[0m";
        std::cout << "\033[1;32m";
        std::cout << "                     ";
        std::cout << "\033[" << (_r_pressed == 0) + 1 << ";32m";
        std::cout << "Start LiDAR Motor" << std::endl;
        std::cout << "\033[0m";

        // top keys animation
        std::cout << "\033[1;36m" << std::endl;
        std::cout << "      _";

        // toggle when l key is pressed
        std::cout << "\033[" << 7 * _l_pressed << ";36m";
        std::cout << "=====";
        std::cout << "\033[0m";
        std::cout << "\033[1;36m";

        std::cout << "_                               _";

        // toggle when r key is pressed
        std::cout << "\033[" << 7 * _r_pressed << ";36m";
        std::cout << "=====";
        std::cout << "\033[0m";
        std::cout << "\033[1;36m_" << std::endl;
        // end of top section

        std::cout << "     / __L__ \\                             / __R__ \\" << std::endl;
        std::cout << "   +.-'_____'-.---------------------------.-'_____'-.+" << std::endl;
        std::cout << "  /   |     |  '.        S O N Y        .'  |  _  |   \\" << std::endl;
        std::cout << " / ___| /|\\ |___ \\                     / ___| /_\\ |___ \\" << std::endl;
        std::cout << "/ |      |      | ;SE";
        std::cout << "\033[4;36m";
        std::cout << "LE";
        std::cout << "\033[0m";
        std::cout << "\033[1;36m";
        std::cout << "CT       START ; | _         _ | ;" << std::endl;
        std::cout << "| | <---   ---> | | |__|         |_:> | ||_|       (_)| |" << std::endl;
        std::cout << "| |___   |   ___| ;        __         ; |___       ___| ;" << std::endl;
        std::cout << "|\\    | \\|/ |    /  _     /P_\\    _    \\    | (X) |    /|" << std::endl;
        std::cout << "| \\   |_____|  .\',\'\" \"\',  \\__/  ,\'\" \"\', \'.  |_____|  .\' |" << std::endl;
        std::cout << "|  \'-.______.-\' /       \\      /       \\  \'-._____.-\'   |" << std::endl;
        std::cout << "|               |       |------|       |                |" << std::endl;
        std::cout << "|              /\\       /      \\       /\\               |" << std::endl;
        std::cout << "|             /  '.___.'        '.___.'  \\              |" << std::endl;
        std::cout << "|            /                            \\             |" << std::endl;
        std::cout << " \\          /                              \\           /" << std::endl;

        std::cout << "  \\________/     ";
        std::cout << "\033[" << (_cmd_out.linear.x == 0) + 1 << ";32m";
        std::cout << "L stick:";
        std::cout << "\033[0m";
        std::cout << "\033[1;32m";
        std::cout << "      ";
        std::cout << "\033[" << (_cmd_out.angular.z == 0) + 1 << ";32m";
        std::cout << "R stick:";
        std::cout << "\033[0m";
        std::cout << "\033[1;36m";
        std::cout << "     \\_________/" << std::endl;

        std::cout << "                 ";
        std::cout << "\033[" << (_cmd_out.linear.x == 0) + 1 << ";32m";
        std::cout << "Throttle";
        std::cout << "      ";
        std::cout << "\033[0m";
        std::cout << "\033[1;32m";
        std::cout << "\033[" << (_cmd_out.angular.z == 0) + 1 << ";32m";
        std::cout << "Turning" << std::endl;

        std::cout << "\033[0;33m" << std::endl;
        std::cout << "//////////////////////PS3 CONTROLLER//////////////////////" << std::endl;
        std::cout << "\033[0m" << std::endl;
    }
} // namespace ps3