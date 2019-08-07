#ifndef GRIDMAP_POINT_CLOUD_DATA_H
#define GRIDMAP_POINT_CLOUD_DATA_H

#include <gridmap/layers/obstacle_data/data_source.h>
#include <gridmap/operations/raytrace.h>

#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>

#include <sensor_msgs/PointCloud2.h>

namespace gridmap
{

class PointCloudData : public DataSource
{
  public:
    PointCloudData();
    virtual ~PointCloudData() override;

    virtual void onInitialize(const XmlRpc::XmlRpcValue& parameters) override;
    virtual void onMapDataChanged() override;

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& message);

  private:
    double hit_probability_log_;
    double miss_probability_log_;

    double obstacle_height_;
    float max_range_;

    int sub_sample_;
    int sub_sample_count_;

    std::vector<double> log_cost_lookup_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> subscriber_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> message_filter_;
};
}

#endif
