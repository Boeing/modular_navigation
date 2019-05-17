#ifndef GRIDMAP_POINT_RANGE_DATA_H
#define GRIDMAP_POINT_RANGE_DATA_H

#include <gridmap/data_source.h>
#include <gridmap/raytrace.h>

#include <laser_geometry/laser_geometry.h>

#include <message_filters/subscriber.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <opencv2/imgproc.hpp>

#include <tf2_ros/message_filter.h>

namespace gridmap
{

class RangeData : public DataSource
{
  public:
    RangeData();
    virtual ~RangeData() override;

    virtual void onInitialize(const XmlRpc::XmlRpcValue& parameters) override;

    virtual void matchSize() override;

    void rangeCallback(const sensor_msgs::RangeConstPtr& message);

  private:
    double hit_probability_log_;
    double miss_probability_log_;

    double min_obstacle_height_;
    double max_obstacle_height_;

    double obstacle_range_;
    double raytrace_range_;

    int sub_sample_;
    int sub_sample_count_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Range>> subscriber_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::Range>> message_filter_;
};
}

#endif
