#ifndef GRIDMAP_LASER_DATA_H
#define GRIDMAP_LASER_DATA_H

#include <gridmap/layers/obstacle_data/data_source.h>
#include <gridmap/operations/raytrace.h>

#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>

#include <sensor_msgs/LaserScan.h>

namespace gridmap
{

class LaserData : public TopicDataSource<sensor_msgs::LaserScan>
{
  public:
    LaserData();
    virtual ~LaserData() override
    {
    }

    virtual void onInitialize(const XmlRpc::XmlRpcValue& parameters) override;
    virtual void onMapDataChanged() override;
    virtual bool processData(const sensor_msgs::LaserScan::ConstPtr& msg,
                             const Eigen::Isometry3d& sensor_transform) override;

  private:
    double hit_probability_log_;
    double miss_probability_log_;

    double min_obstacle_height_;
    double max_obstacle_height_;
    double obstacle_range_;
    double raytrace_range_;

    std::vector<Eigen::Vector3d> laser_directions_;
};
}  // namespace gridmap

#endif
