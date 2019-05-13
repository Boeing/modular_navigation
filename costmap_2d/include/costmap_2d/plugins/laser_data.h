#ifndef COSTMAP_2D_LASER_DATA_H
#define COSTMAP_2D_LASER_DATA_H

#include <costmap_2d/data_source.h>

#include <laser_geometry/laser_geometry.h>

#include <message_filters/subscriber.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <opencv2/imgproc.hpp>

#include <tf2_ros/message_filter.h>

namespace costmap_2d
{

class LaserData : public DataSource
{
  public:
    LaserData();
    virtual ~LaserData() override;

    virtual void onInitialize(const XmlRpc::XmlRpcValue& parameters) override;

    virtual void matchSize() override;

    void laserScanCallback(const sensor_msgs::LaserScanConstPtr& message);

  private:
    laser_geometry::LaserProjection projector_;

    double hit_probability_log_;
    double miss_probability_log_;

    double min_obstacle_height_;
    double max_obstacle_height_;
    double obstacle_range_;
    double raytrace_range_;

    int sub_sample_;
    int sub_sample_count_;

    bool inf_is_valid_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> subscriber_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> message_filter_;

    void raytrace(const double sensor_x, const double sensor_y, const sensor_msgs::PointCloud2& cloud);
};
}

#endif
