#ifndef COSTMAP_2D_OBSTACLE_LAYER_H_
#define COSTMAP_2D_OBSTACLE_LAYER_H_

#include <unordered_map>

#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>

#include <ros/ros.h>

#include <laser_geometry/laser_geometry.h>

#include <message_filters/subscriber.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <opencv2/imgproc.hpp>

#include <tf2_ros/message_filter.h>

namespace costmap_2d
{

struct DataSource
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


class ObstacleLayer : public Layer, public Costmap2D
{
  public:
    ObstacleLayer();
    virtual ~ObstacleLayer() override;

    virtual void onInitialize() override;
    virtual void updateBounds(const double robot_x, const double robot_y, const double robot_yaw, double* min_x,
                              double* min_y, double* max_x, double* max_y) override;
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, unsigned int min_i, unsigned int min_j,
                             unsigned int max_i, unsigned int max_j) override;

    virtual void activate() override;
    virtual void deactivate() override;
    virtual void reset() override;
    virtual void matchSize() override;

    void laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                           const std::shared_ptr<DataSource>& data_source);

    void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                             const std::shared_ptr<DataSource>& data_source);

  private:
    laser_geometry::LaserProjection projector_;
    std::vector<std::shared_ptr<DataSource>> data_sources_;

    std::vector<float> marking_;
    std::vector<float> marking_timestamp_;

    bool footprint_clearing_enabled_;

    void raytraceClearing(const double sensor_x, const double sensor_y, const sensor_msgs::PointCloud2& cloud,
                                         const double min_obstacle_height, const double max_obstacle_height,
                                         const double raytrace_range, const double obstacle_range);
};
}

#endif
