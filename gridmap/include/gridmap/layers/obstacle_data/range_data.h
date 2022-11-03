#ifndef GRIDMAP_RANGE_DATA_H
#define GRIDMAP_RANGE_DATA_H

#include <gridmap/layers/obstacle_data/data_source.h>
#include <gridmap/operations/raytrace.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/msg/range.hpp>
#include <tf2_ros/message_filter.h>

namespace gridmap
{

class RangeData : public TopicDataSource<sensor_msgs::msg::Range>
{
  public:
    RangeData();
    virtual ~RangeData() override;

    virtual void onInitialize(const YAML::Node& parameters) override;
    virtual void onMapDataChanged() override;
    virtual bool processData(const sensor_msgs::msg::Range::SharedPtr msg, const Eigen::Isometry2d& robot_pose,
                             const Eigen::Isometry3d& sensor_transform) override;

  private:
    void generateLogCostLookup(const double max_range);

    double hit_probability_;
    double miss_probability_;

    double std_deviation_;

    double max_range_;
    double obstacle_range_;
    double raytrace_range_;

    std::vector<std::vector<double>> log_cost_lookup_;
};
}  // namespace gridmap

#endif
