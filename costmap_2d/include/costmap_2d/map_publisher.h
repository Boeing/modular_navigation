#ifndef COSTMAP_2D_MAP_PUBLISHER_H
#define COSTMAP_2D_MAP_PUBLISHER_H

#include <costmap_2d/map_data.h>

#include <nav_msgs/OccupancyGrid.h>

#include <ros/ros.h>

#include <memory>
#include <atomic>
#include <thread>

namespace costmap_2d
{

class MapPublisher
{
  public:
    MapPublisher(const double update_frequency,
                 const std::shared_ptr<MapData>& map_data,
                 const std::string& global_frame);

    ~MapPublisher();

  private:
    void prepareGrid();
    void publishThread(const double frequency);

    const std::shared_ptr<MapData> map_data_;
    const std::string global_frame_;
    const std::string topic_name_;

    ros::NodeHandle nh_;

    std::atomic<bool> running_;
    std::thread publish_thread_;

    ros::Publisher costmap_pub_;
    nav_msgs::OccupancyGrid grid_;
};
}

#endif
