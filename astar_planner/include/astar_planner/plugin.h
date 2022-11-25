#ifndef ASTAR_PLANNER_PLUGIN_H
#define ASTAR_PLANNER_PLUGIN_H

#include <astar_planner/costmap.h>
#include <gridmap/map_data.h>
#include <navigation_interface/path_planner.h>
#include <opencv2/core.hpp>
#include <visualization_msgs/msg/marker_array.h>
//#include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

namespace astar_planner
{

class AStarPlanner : public navigation_interface::PathPlanner
{
  public:
    AStarPlanner();
    ~AStarPlanner();

    virtual Result plan(const gridmap::AABB& local_region, const Eigen::Isometry2d& start,
                        const Eigen::Isometry2d& goal, const GoalSampleSettings& sample, const double avoid_distance,
                        const double backwards_mult, const double strafe_mult, const double rotation_mult) override;

    virtual bool valid(const navigation_interface::Path& path, const double avoid_distance) const override;
    virtual double cost(const navigation_interface::Path& path, const double avoid_distance) const override;

    virtual void onInitialize(const YAML::Node& parameters) override;
    virtual void onMapDataChanged() override;

  private:
    bool debug_viz_ = false;
    double robot_radius_ = 0.230;
    // width of default offset = 0.185
    // conservative = width + robot_radius = 0.185 + 0.230 = 0.415
    double conservative_robot_radius_ = 0.416;

    double backwards_mult_ = 1.5;
    double strafe_mult_ = 1.5;
    double rotation_mult_ = 0.3 / M_PI;

    std::vector<Eigen::Vector2d> offsets_;

    // ros::Publisher explore_pub_; // this is created on initialized_pub
    rclcpp::Node::SharedPtr node_ = nullptr;
    typename rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr explore_pub_;

    std::shared_ptr<Costmap> costmap_;
    std::shared_ptr<cv::Mat> traversal_cost_;
};
}  // namespace astar_planner

#endif
