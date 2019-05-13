#include <costmap_2d/cost_values.h>
#include <costmap_2d/map_publisher.h>

#include <opencv2/imgproc.hpp>

#include <boost/bind.hpp>

namespace costmap_2d
{

MapPublisher::MapPublisher(const double update_frequency,
                           const std::shared_ptr<MapData>& map_data,
                           const std::string& global_frame)
    : map_data_(map_data),
      global_frame_(global_frame)
{
    costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("costmap", 1, true);
    running_ = true;
    publish_thread_ = std::thread(&MapPublisher::publishThread, this, update_frequency);
}

MapPublisher::~MapPublisher()
{
    running_ = false;
    publish_thread_.join();
}

void MapPublisher::prepareGrid()
{
    auto lock = map_data_->getLock();
    const double resolution = map_data_->resolution();
    const int down_sample = 4;

    grid_.header.frame_id = global_frame_;
    grid_.header.stamp = ros::Time::now();
    grid_.info.resolution = resolution * down_sample;

    grid_.info.width = map_data_->sizeX() / down_sample;
    grid_.info.height = map_data_->sizeY() / down_sample;

    double wx, wy;
    map_data_->mapToWorld(0, 0, wx, wy);
    grid_.info.origin.position.x = wx - resolution / 2;
    grid_.info.origin.position.y = wy - resolution / 2;
    grid_.info.origin.position.z = 0.0;
    grid_.info.origin.orientation.w = 1.0;

    grid_.data.resize(grid_.info.width * grid_.info.height);
    std::vector<double> temp(grid_.info.width * grid_.info.height);
    unsigned int index = 0;
    for (unsigned int i = 0; i < map_data_->sizeX(); ++i)
    {
        for (unsigned int j = 0; j < map_data_->sizeY(); ++j)
        {
            const unsigned int sub_i = i / down_sample;
            const unsigned int sub_j = j / down_sample;
            const unsigned int sub_index = sub_i * grid_.info.width + sub_j;
            temp[sub_index] = std::max(temp[sub_index], map_data_->data()[index]);
            ++index;
        }
    }

    for (unsigned int i = 0; i < temp.size(); ++i)
    {
        grid_.data[i] = std::min(static_cast<int8_t>(100), static_cast<int8_t>(probability(temp[i]) * 100.0));
    }
}

void MapPublisher::publishThread(const double frequency)
{
    if (frequency <= 0.0)
        return;

    ros::Rate rate(frequency);
    while (ros::ok() && running_)
    {
        if (costmap_pub_.getNumSubscribers() != 0)
        {
            prepareGrid();
            costmap_pub_.publish(grid_);
        }

        rate.sleep();
    }
}

}
