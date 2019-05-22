#ifndef GRIDMAP_RANGE_DATA_H
#define GRIDMAP_RANGE_DATA_H

#include <gridmap/layers/obstacle_data/data_source.h>
#include <gridmap/operations/raytrace.h>

#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>

#include <sensor_msgs/Range.h>

namespace gridmap
{

class RangeData : public DataSource
{
  public:
    RangeData();
    virtual ~RangeData() override;

    virtual void onInitialize(const XmlRpc::XmlRpcValue& parameters) override;
    virtual void onMapDataChanged() override;

    void rangeCallback(const sensor_msgs::RangeConstPtr& message);

  private:
    double hit_probability_log_;
    double miss_probability_log_;

    double min_obstacle_height_;
    double max_obstacle_height_;

    double raytrace_range_;

    int sub_sample_;
    int sub_sample_count_;

    std::vector<std::vector<double>> log_cost_lookup_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Range>> subscriber_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::Range>> message_filter_;
};
}

#endif
