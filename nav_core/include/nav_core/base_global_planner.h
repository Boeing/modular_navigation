#ifndef NAV_CORE_BASE_GLOBAL_PLANNER_H
#define NAV_CORE_BASE_GLOBAL_PLANNER_H

#include <costmap_2d/costmap_2d_ros.h>
#include <memory>

#include <geometry_msgs/PoseStamped.h>

namespace nav_core
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

    virtual void initialize(std::string name, std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                            std::shared_ptr<costmap_2d::Costmap2DROS> global_costmap,
                            std::shared_ptr<costmap_2d::Costmap2DROS> local_costmap) = 0;

    virtual ~BaseGlobalPlanner()
    {
    }

  protected:
    BaseGlobalPlanner()
    {
    }
};
};  // namespace nav_core

#endif  // NAV_CORE_BASE_GLOBAL_PLANNER_H
