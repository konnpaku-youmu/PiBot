#ifndef HECTOR_APP_H
#define HECTOR_APP_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <std_msgs/Int8MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

namespace hector_app
{
    class App
    {
    private:
        
    public:
        App(ros::NodeHandle &);

        ~App() { return; }
    };

} // namespace hector_app

#endif
