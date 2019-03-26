#ifndef NAVIGATION_INTERFACE_BASE_LOCAL_PLANNER_H_BASE_GLOBAL_PLANNER_H
#define NAVIGATION_INTERFACE_BASE_LOCAL_PLANNER_H_BASE_GLOBAL_PLANNER_H

#include <costmap_2d/costmap_2d_ros.h>
#include <memory>

#include <geometry_msgs/PoseStamped.h>

#include <tf2_ros/buffer.h>

namespace navigation_interface
{

struct PlanResult
{
    bool success;
    double cost;
    std::vector<geometry_msgs::PoseStamped> plan;
};

class BaseGlobalPlanner
{
  public:
    virtual PlanResult makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal) = 0;
    virtual double cost(const std::vector<geometry_msgs::PoseStamped>& plan) = 0;

    virtual void initialize(const std::string& name, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                            const std::shared_ptr<costmap_2d::Costmap2DROS>& global_costmap,
                            const std::shared_ptr<costmap_2d::Costmap2DROS>& local_costmap) = 0;

    virtual ~BaseGlobalPlanner()
    {
    }

  protected:
    BaseGlobalPlanner()
    {
    }
};
}

#endif