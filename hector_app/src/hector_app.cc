#include "../include/hector_app/hector_app.h"

namespace hector_app
{
    MapLoader::MapLoader(ros::NodeHandle &_nodehandle)
        : _nh(_nodehandle)
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
            _map.origin_position = Eigen::Vector2d(_map_ptr->info.origin.position.x,
                                                   _map_ptr->info.origin.position.y);
            _map.map_orientation = Eigen::Quaterniond(_map_ptr->info.origin.orientation.w,
                                                      _map_ptr->info.origin.orientation.x,
                                                      _map_ptr->info.origin.orientation.y,
                                                      _map_ptr->info.origin.orientation.z);
            _map.map_size(0) = _map_ptr->info.height;
            _map.map_size(1) = _map_ptr->info.width;

            _map.occupancy_data = Eigen::MatrixXi::Zero(_map.map_size(0),
                                                        _map.map_size(1));

            // fill with occupancy data
            for(size_t i = 0; i < _map_ptr->data.size(); ++i)
            {
                int _row = i / _map.occupancy_data.rows();
                int _col = i % _map.occupancy_data.rows();

                _map.occupancy_data(_row, _col) = _map_ptr->data[i];
            }
            std::cout << "Finished" << std::endl; 
        }

        return;
    }

} // namespace hector_app