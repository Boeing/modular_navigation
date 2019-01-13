#ifndef OMNI_PID_CONTROLLER_H
#define OMNI_PID_CONTROLLER_H

#include <omni_pid_controller/pid.h>

#include <nav_core/base_local_planner.h>

#include <costmap_2d/costmap_2d.h>

#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <atomic>
#include <memory>
#include <mutex>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <ros/ros.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>

namespace omni_pid_controller
{

struct ControlData
{
    Eigen::Isometry3d global_to_local;

    // This is the requested global path in "map" frame
    std::vector<geometry_msgs::PoseStamped> global_path;

    // This is the execution of the current local trajectory in "odom" frame
    std::vector<Eigen::Isometry3d> local_trajectory;

    std::size_t execution_index = 1;

    ros::SteadyTime last_time_step = ros::SteadyTime::now();
};

unsigned char getCost(const costmap_2d::Costmap2D& costmap, const double x, const double y);
double getDistanceToCollision(const costmap_2d::Costmap2D& costmap, const double x, const double y,
                              const double inflation_weight);
double getDistanceToCollision(const unsigned char cost, const double inflation_weight);

class OmniPIDController : public nav_core::BaseLocalPlanner
{
  public:
    OmniPIDController();
    virtual ~OmniPIDController() override;

    //
    // BaseLocalPlanner
    //
    virtual nav_core::Control computeControl(const ros::SteadyTime& steady_time, const ros::Time& ros_time,
                                             const nav_msgs::Odometry& odom) override;
    virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
    virtual bool clearPlan() override;
    virtual void initialize(const std::string& name, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                            const std::shared_ptr<costmap_2d::Costmap2DROS>& local_costmap) override;

  private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<costmap_2d::Costmap2DROS> local_costmap_;

    // This is the execution of the current local trajectory in "odom" frame
    std::mutex control_mutex_;
    std::unique_ptr<ControlData> control_data_;
    PID goal_x_pid_ = PID(1, 0.5, 0.0, -0, 0, true);
    PID goal_y_pid_ = PID(1, 0.5, 0.0, -0, 0, true);
    PID tracking_x_pid_ = PID(4, 2.0, 0.0, -0, 0, true);
    PID tracking_y_pid_ = PID(4, 2.0, 0.0, -0, 0, true);
    PID rotation_pid_ = PID(1, 0.1, 0.0, -0, 0, true);

    const double control_p_gain_ = 10.0;
    const double control_d_gain_ = 2.0;

    const double goal_linear_threshold = 0.005;
    const double goal_angular_threshold = 0.0175;

    const double max_velocity_x_ = 0.15;
    const double max_acceleration_x_ = 0.01;

    const double max_velocity_y_ = 0.10;
    const double max_acceleration_y_ = 0.01;

    const double max_velocity_w_ = 0.5;
    const double max_acceleration_w_ = 0.1;

    const double goal_radius_ = 0.04;

    const double max_linear_tracking_error_ = 0.05;
    const double max_angular_tracking_error_ = 0.4;

    const double costmap_weight_ = 2.0;
};
}  // namespace omni_pid_controller

#endif
