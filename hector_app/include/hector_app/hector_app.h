#ifndef HECTOR_APP_H
#define HECTOR_APP_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <nav_msgs/OccupancyGrid.h>

namespace hector_app
{
    struct MapData
    {
        Eigen::Vector2d origin_position;
        Eigen::Quaterniond map_orientation;
        Eigen::Vector2i map_size; // (height, width)

        ////////////////////////////////
        Eigen::MatrixXi occupancy_data;
    };
    typedef std::vector<MapData> MapList;

    class MapLoader
    {
    private:
        ros::NodeHandle _nh;

        ros::Subscriber _map_sub;

        MapData _map;

        void _map_cb(const nav_msgs::OccupancyGridConstPtr);

    public:
        MapLoader(ros::NodeHandle &);

        ~MapLoader() { return; }
    };

    class Navigator
    {
    private:
    public:
        Navigator() { return; }

        ~Navigator() { return; }
    };

    class App
    {
    private:
        std::unique_ptr<MapLoader> _map;

        std::unique_ptr<Navigator> _nav;

    public:
        App(ros::NodeHandle &_nh)
            : _map(std::make_unique<MapLoader>(_nh)),
              _nav(std::make_unique<Navigator>()) {}

        ~App() { return; }
    };

} // namespace hector_app

#endif
