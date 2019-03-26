#ifndef EBAND_LOCAL_PLANNER_EBAND_TRAJECTORY_CONTROLLER_H
#define EBAND_LOCAL_PLANNER_EBAND_TRAJECTORY_CONTROLLER_H

#include <ros/assert.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

#include <eband_local_planner/conversions_and_types.h>
#include <eband_local_planner/pid.h>

#include <geometry_msgs/Twist.h>

#include <navigation_interface/base_local_planner.h>
#include <nav_msgs/Odometry.h>

namespace eband_local_planner
{

class EBandController
{
  public:
    EBandController(const std::shared_ptr<costmap_2d::Costmap2DROS>& local_costmap, const double max_velocity_x,
                    const double max_velocity_y, const double max_velocity_w, const double max_acceleration_x,
                    const double max_acceleration_y, const double max_acceleration_w, const double goal_radius,
                    const double xy_goal_tolerance, const double yaw_goal_tolerance, const double k_prop,
                    const double k_damp);
    ~EBandController();

    navigation_interface::Control computeControl(const std::vector<Bubble>& elastic_band, const ros::SteadyTime& steady_time,
                                     const ros::Time& ros_time, const nav_msgs::Odometry& odom);

  private:
    const std::shared_ptr<costmap_2d::Costmap2DROS> local_costmap_;

    PID goal_x_pid_ = PID(1, 0.5, 0.0, 0, 0, true);
    PID goal_y_pid_ = PID(1, 0.5, 0.0, 0, 0, true);
    PID goal_w_pid_ = PID(1, 0.1, 0.0, 0, 0, true);

    // parameters
    const double max_velocity_x_;
    const double max_velocity_y_;
    const double max_velocity_w_;

    const double max_acceleration_x_;
    const double max_acceleration_y_;
    const double max_acceleration_w_;

    const double goal_radius_ = 0.06;

    const double xy_goal_tolerance_;
    const double yaw_goal_tolerance_;

    const double k_prop_;
    const double k_damp_;

    const double robot_radius_;

    ros::SteadyTime last_update_;
};
}

#endif
