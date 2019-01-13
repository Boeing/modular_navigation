#ifndef NAV_CORE_BASE_LOCAL_PLANNER_H
#define NAV_CORE_BASE_LOCAL_PLANNER_H

#include <costmap_2d/costmap_2d_ros.h>
#include <memory>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>

#include <tf2_ros/buffer.h>

namespace nav_core
{

enum class ControlState
{
    RUNNING,
    EMERGENCY_BRAKING,
    COMPLETE,
    FAILED
};

enum class ControlFailure
{
    NONE = 0,
    BAD_ODOMETRY,
    TRACKING_LIMIT_EXCEEDED
};


struct Control
{
    ControlState state;
    ControlFailure error = ControlFailure::NONE;
    geometry_msgs::Twist cmd_vel;
};

class BaseLocalPlanner
{
  public:
    virtual Control computeControl(const ros::SteadyTime& steady_time, const ros::Time& ros_time,
                                   const nav_msgs::Odometry& odom) = 0;

    virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) = 0;
    virtual bool clearPlan() = 0;

    virtual void initialize(const std::string& name, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                            const std::shared_ptr<costmap_2d::Costmap2DROS>& local_costmap) = 0;

    virtual ~BaseLocalPlanner()
    {
    }

  protected:
    BaseLocalPlanner()
    {
    }
};
}

#endif
