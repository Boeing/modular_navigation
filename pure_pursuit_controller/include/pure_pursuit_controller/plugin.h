#ifndef PURE_PURSUIT_CONTROLLER_PLUGIN_H
#define PURE_PURSUIT_CONTROLLER_PLUGIN_H

#include <gridmap/map_data.h>
#include <navigation_interface/controller.h>
//#include <ros/ros.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcpputils/asserts.hpp"

namespace pure_pursuit_controller
{

class PurePursuitController : public navigation_interface::Controller
{
  public:
    PurePursuitController();//rclcpp::Node::SharedPtr& node);
    virtual ~PurePursuitController() override;

    virtual bool setTrajectory(const navigation_interface::Trajectory& trajectory) override;
    virtual void clearTrajectory() override;

    virtual boost::optional<std::string> trajectoryId() const override;
    virtual boost::optional<navigation_interface::Trajectory> trajectory() const override;

    // virtual Result control(const ros::SteadyTime& time, const gridmap::AABB& local_region,
    virtual Result control(const rclcpp::Time& time, const gridmap::AABB& local_region,
                           const navigation_interface::KinodynamicState& robot_state,
                           const Eigen::Isometry2d& map_to_odom, const Eigen::Vector3d max_velocity,
                           const double xy_goal_tolerance, const double yaw_goal_tolerance);  // override;

    virtual void onInitialize(const YAML::Node& parameters) override;
    virtual void onMapDataChanged() override;

  private:
    // Runtime values
    std::size_t path_index_ = 0;
    std::unique_ptr<const navigation_interface::Trajectory> trajectory_;
    // ros::SteadyTime last_update_;
    rclcpp::Time last_update_;

    // Parameters
    double look_ahead_time_ = 1.5;
    unsigned int interpolation_steps_ = 20;

    Eigen::Vector3d max_velocity_ = {0.20, 0.20, 0.20};
    double max_translation_accel_ = 0.2;
    double max_rotation_accel_ = 0.2;

    double tracking_error_ = 0.25;

    double xy_goal_tolerance_ = 0.02;
    double yaw_goal_tolerance_ = 0.01;

    Eigen::Vector3d p_gain_ = {1.1, 1.1, 1.1};
    Eigen::Vector3d d_gain_ = {0.4, 0.4, 0.4};

    // I term only activated when close to goal to close the steady-state error
    Eigen::Vector3d i_gain_ = {0.1, 0.1, 0.1};

    Eigen::Vector3d control_error_;
    Eigen::Vector3d control_integral_;
    Eigen::Vector3d control_integral_max_ = {2.0, 2.0, 2.0};
    Eigen::Vector3d prev_cmd_vel = {0.0, 0.0, 0.0};

    // When approaching an obstacle, the speed will be scaled linearly between
    // max_avoid_distance_ and min_avoid_distance_
    double max_avoid_distance_ = 0.6;
    double min_avoid_distance_ = 0.12;

    bool debug_viz_ = true;

    // Making a node attrb instead of usig publishers
    // Although it is only used if debug_viz_ is set inside a yaml
    typename rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_state_pub_;
    typename rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr footprint_pub_;

    // Now it is assumed that onInitialize is always called first
    // ros::Publisher target_state_pub_;
    // ros::Publisher footprint_pub_;

    std::vector<Eigen::Vector2d> robot_footprint_;
};
}  // namespace pure_pursuit_controller

#endif
