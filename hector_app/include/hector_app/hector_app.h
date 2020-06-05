#ifndef HECTOR_APP_H
#define HECTOR_APP_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <opencv-3.3.1-dev/opencv2/opencv.hpp>
#include <opencv-3.3.1-dev/opencv2/core/eigen.hpp>

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

    struct NavigationGoal
    {
        Eigen::Vector2d position;
        Eigen::Quaterniond orientation;

        Eigen::Vector2d end_velocity;
    };
    typedef std::vector<NavigationGoal> NavGoalList;

    class MapLoader
    {
    private:
        ros::NodeHandle _nh;

        ros::Subscriber _map_sub;

        std::shared_ptr<MapData> _map;

        void _map_cb(const nav_msgs::OccupancyGridConstPtr);

    public:
        MapLoader(ros::NodeHandle &, MapData &);

        ~MapLoader() { return; }
    };

    class Navigator
    {
    private:
        ros::NodeHandle _nh;

        ros::Subscriber _nav_goal_sub;

        std::shared_ptr<MapData> _map;

        NavigationGoal _goal;

        void _nav_goal_cb(const geometry_msgs::PoseStampedConstPtr);

    public:
        Navigator(ros::NodeHandle &, MapData &);

        ~Navigator() { return; }

        void updateRoutePlanning();
    };

    class App
    {
    private:
        std::unique_ptr<MapLoader> _map_loader;

        std::unique_ptr<Navigator> _nav;

        MapData _nav_map;

    public:
        App(ros::NodeHandle &);

        ~App() { return; }
    };

} // namespace hector_app

#endif
