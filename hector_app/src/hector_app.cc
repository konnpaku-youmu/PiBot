#include "../include/hector_app/hector_app.h"

namespace hector_app
{
    App::App(ros::NodeHandle &_nh)
    {
        ros::Rate loop(20);
        while(ros::ok())
        {
            ros::spinOnce();
            loop.sleep();
        }
    }
} // namespace hector_app