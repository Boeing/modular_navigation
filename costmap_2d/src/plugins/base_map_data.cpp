#include <costmap_2d/plugins/base_map_data.h>
#include <costmap_2d/params.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::BaseMapData, costmap_2d::DataSource)

namespace costmap_2d
{

BaseMapData::BaseMapData()
{
}

BaseMapData::~BaseMapData()
{
}

void BaseMapData::onInitialize(const XmlRpc::XmlRpcValue& parameters)
{
    lethal_threshold_ = get_config_with_default_warn<int>(parameters, "lethal_threshold", 100, XmlRpc::XmlRpcValue::TypeInt);
    const std::string map_topic = get_config_with_default_warn<std::string>(parameters, "map_topic", "map", XmlRpc::XmlRpcValue::TypeString);

    ros::NodeHandle g_nh;
    map_sub_ = g_nh.subscribe(map_topic, 1, &BaseMapData::incomingMap, this);
}

void BaseMapData::matchSize()
{
}

void BaseMapData::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
    auto lock = map_data_->getLock();

    ROS_INFO_STREAM("Received occupancy grid of size=" << new_map->info.width << "x" << new_map->info.height
                                                       << " resolution=" << new_map->info.resolution);

    // We want to keep the resolution provided in configuration
    // But we need to increase the size of the costmap data to fit the map

    const double size_x_m = static_cast<double>(new_map->info.width * new_map->info.resolution);
    const double size_y_m = static_cast<double>(new_map->info.height * new_map->info.resolution);

    const double resolution = map_data_->resolution();

    const unsigned int size_x = static_cast<unsigned int>(size_x_m / resolution);
    const unsigned int size_y = static_cast<unsigned int>(size_y_m / resolution);

    ROS_INFO_STREAM("Resizing costmap to size=" << size_x << "x" << size_y << " resolution=" << resolution);

    map_data_->resize(size_x, size_y, resolution, new_map->info.origin.position.x, new_map->info.origin.position.y);

    //
    // Copy the occupancy grid into the costmap
    //
    unsigned int index = 0;
    for (unsigned int i = 0; i < size_y; ++i)
    {
        for (unsigned int j = 0; j < size_x; ++j)
        {
            const float x_min_m = static_cast<float>(j * resolution);
            const float x_max_m = static_cast<float>((j + 1) * resolution);

            const float y_min_m = static_cast<float>(i * resolution);
            const float y_max_m = static_cast<float>((i + 1) * resolution);

            const unsigned int _x_min = static_cast<unsigned int>(x_min_m / new_map->info.resolution);
            const unsigned int _x_max =
                std::min(static_cast<unsigned int>(x_max_m / new_map->info.resolution), new_map->info.width - 1);

            const unsigned int _y_min = static_cast<unsigned int>(y_min_m / new_map->info.resolution);
            const unsigned int _y_max =
                std::min(static_cast<unsigned int>(y_max_m / new_map->info.resolution), new_map->info.height - 1);

            char value = 0;
            for (unsigned int _i = _y_min; _i <= _y_max; ++_i)
            {
                for (unsigned int _j = _x_min; _j <= _x_max; ++_j)
                {
                    const unsigned int _index = (_i * new_map->info.width) + _j;
                    value = std::max(static_cast<char>(new_map->data[_index]), value);
                }
            }

            if (value == -1)
                map_data_->data()[index] = 0.5;
            else if (value >= lethal_threshold_)
                map_data_->data()[index] = map_data_->clampingThresMax();
            else
                map_data_->data()[index] = map_data_->clampingThresMin();

            ++index;
        }
    }

}

}
