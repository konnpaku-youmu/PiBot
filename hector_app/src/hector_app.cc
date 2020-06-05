#include "../include/hector_app/hector_app.h"

namespace hector_app
{
    App::App(ros::NodeHandle &_nh)
        : _map_loader(std::make_unique<MapLoader>(_nh, _nav_map)),
          _nav(std::make_unique<Navigator>(_nh, _nav_map))
    {
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            _nav->updateRoutePlanning();

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    Navigator::Navigator(ros::NodeHandle &_nodehandle, MapData &_map)
        : _nh(_nodehandle),
          _map(std::make_shared<MapData>(_map))
    {
        this->_nav_goal_sub = this->_nh.subscribe("/move_base_simple/goal", 10, &Navigator::_nav_goal_cb, this);
    }

    void Navigator::_nav_goal_cb(const geometry_msgs::PoseStampedConstPtr _goal_pose_ptr)
    {
    }

    void Navigator::updateRoutePlanning()
    {
    }

    MapLoader::MapLoader(ros::NodeHandle &_nodehandle, MapData &_map_ptr)
        : _nh(_nodehandle),
          _map(std::make_shared<MapData>(_map_ptr))
    {
        this->_map_sub = this->_nh.subscribe("/map", 10, &MapLoader::_map_cb, this);

        return;
    }

    void MapLoader::_map_cb(const nav_msgs::OccupancyGridConstPtr _map_ptr)
    {
        if (!_map_ptr->data.empty())
        {
            std::cout << "Start...";
            // extract map info
            _map->origin_position = Eigen::Vector2d(_map_ptr->info.origin.position.x,
                                                    _map_ptr->info.origin.position.y);
            _map->map_orientation = Eigen::Quaterniond(_map_ptr->info.origin.orientation.w,
                                                       _map_ptr->info.origin.orientation.x,
                                                       _map_ptr->info.origin.orientation.y,
                                                       _map_ptr->info.origin.orientation.z);
            _map->map_size(0) = _map_ptr->info.height;
            _map->map_size(1) = _map_ptr->info.width;

            _map->occupancy_data = Eigen::MatrixXi::Zero(_map->map_size(0),
                                                         _map->map_size(1));

            // fill with occupancy data
            for (size_t i = 0; i < _map_ptr->data.size(); ++i)
            {
                int _row = i / _map->occupancy_data.rows();
                int _col = i % _map->occupancy_data.rows();

                _map->occupancy_data(_row, _col) = _map_ptr->data[i];
            }

            std::cout << "Finished" << std::endl;
        }

        cv::Mat view;
        cv::eigen2cv(_map->occupancy_data, view);
        // view = cv::abs(view);

        for (int row = 0; row < view.rows; ++row)
        {
            for (int col = 0; col < view.cols; ++col)
            {
                if (view.at<int>(row, col) > 50)
                {
                    view.at<int>(row, col) = 255;
                }
                else if (view.at<int>(row, col) == -1)
                {
                    view.at<int>(row, col) = 128;
                }
            }
        }

        cv::convertScaleAbs(view, view);
        cv::medianBlur(view, view, 3);

        cv::flip(view, view, 0);

        // cv::imwrite("/home/hcrd/PiBot/map_view.bmp", view);

        return;
    }

} // namespace hector_app