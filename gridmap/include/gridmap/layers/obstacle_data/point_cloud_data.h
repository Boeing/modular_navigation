#ifndef GRIDMAP_POINT_CLOUD_DATA_H
#define GRIDMAP_POINT_CLOUD_DATA_H

#include <gridmap/layers/obstacle_data/data_source.h>
#include <gridmap/operations/raytrace.h>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace gridmap
{

class PointCloudData : public TopicDataSource<sensor_msgs::msg::PointCloud2>
{
  public:
    PointCloudData();
    virtual ~PointCloudData() override;

    virtual void onInitialize(const YAML::Node& parameters) override;
    virtual void onMapDataChanged() override;
    virtual bool processData(const sensor_msgs::msg::PointCloud2::ConstPtr& msg, const Eigen::Isometry2d& robot_pose,
                             const Eigen::Isometry3d& sensor_transform) override;

  private:
    double hit_probability_log_;
    double miss_probability_log_;

    double obstacle_height_;
    float max_range_;

    std::vector<double> log_cost_lookup_;
};

}  // namespace gridmap

#endif
