#ifndef GRIDMAP_ROBOT_TRACKER
#define GRIDMAP_ROBOT_TRACKER

#include <Eigen/Geometry>

#include <boost/circular_buffer.hpp>
#include <cartographer_ros_msgs/msg/system_state.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Include rclcpp just for time?
#include "rclcpp/rclcpp.hpp"
//#include <ros/time.h>

#include <condition_variable>
#include <memory>
#include <mutex>

namespace gridmap
{

struct TimedOdom
{
    rclcpp::Time time;
    Eigen::Isometry2d pose;
    Eigen::Vector3d velocity;
};

struct RobotState
{
    TimedOdom odom;
    bool localised;  // true if map_to_odom is valid
    Eigen::Isometry2d map_to_odom;
};

class RobotTracker
{
  public:
    RobotTracker();

    RobotTracker(const RobotTracker&) = delete;
    RobotTracker& operator=(const RobotTracker&) = delete;

    bool localised() const
    {
        return localisation_.localised;
    }

    RobotState waitForRobotState(const double timeout_ms) const;

    RobotState robotState() const;
    RobotState robotState(const rclcpp::Time& time) const;

    void addOdometryData(const nav_msgs::msg::Odometry& odometry_data);
    void addLocalisationData(const cartographer_ros_msgs::msg::SystemState& localisation);

  private:
    struct LocalisationState
    {
        bool localised;
        Eigen::Isometry2d map_to_odom;
    };

    mutable std::condition_variable conditional_;
    mutable std::mutex mutex_;

    LocalisationState localisation_;
    boost::circular_buffer<TimedOdom> odometry_data_;
};

}  // namespace gridmap

#endif
