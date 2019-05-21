#ifndef GRIDMAP_LASER_DATA_H
#define GRIDMAP_LASER_DATA_H

#include <gridmap/layers/obstacle_data/data_source.h>
#include <gridmap/operations/raytrace.h>

#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>

#include <sensor_msgs/LaserScan.h>

namespace gridmap
{

class LaserData : public DataSource
{
  public:
    LaserData();
    virtual ~LaserData() override;

    virtual void onInitialize(const XmlRpc::XmlRpcValue& parameters) override;

    void laserScanCallback(const sensor_msgs::LaserScanConstPtr& message);

  private:
    double hit_probability_log_;
    double miss_probability_log_;

    double min_obstacle_height_;
    double max_obstacle_height_;
    double obstacle_range_;
    double raytrace_range_;

    int sub_sample_;
    int sub_sample_count_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> subscriber_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> message_filter_;

    std::vector<Eigen::Vector3d> laser_directions_;
};
}

#endif
