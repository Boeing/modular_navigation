#ifndef GRIDMAP_POINT_CLOUD_DATA_H
#define GRIDMAP_POINT_CLOUD_DATA_H

/*
#include <gridmap/data_source.h>

#include <laser_geometry/laser_geometry.h>

#include <message_filters/subscriber.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <opencv2/imgproc.hpp>

#include <tf2_ros/message_filter.h>

namespace gridmap
{

struct Sensor
{
    boost::shared_ptr<message_filters::SubscriberBase> subscriber;
    boost::shared_ptr<tf2_ros::MessageFilterBase> message_filter;

    double observation_keep_time;
    double expected_update_rate;
    double min_obstacle_height;
    double max_obstacle_height;
    double obstacle_range;
    double raytrace_range;

    int sub_sample;
    int sub_sample_count;

    bool inf_is_valid;
};


class ObstacleData : public DataSource
{
  public:
    ObstacleData();
    virtual ~ObstacleData() override;

    virtual void onInitialize() override;

    virtual void matchSize() override;

    void laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                           const std::shared_ptr<Sensor>& sensor);

    void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                             const std::shared_ptr<Sensor>& sensor);

  private:
    laser_geometry::LaserProjection projector_;
    std::vector<std::shared_ptr<Sensor>> sensors_;

    void raytrace(const double sensor_x, const double sensor_y, const sensor_msgs::PointCloud2& cloud,
                  const double min_obstacle_height, const double max_obstacle_height,
                  const double raytrace_range, const double obstacle_range);
};
}

*/

#endif
