#ifndef ASTAR_PLANNER_PLUGIN_H
#define ASTAR_PLANNER_PLUGIN_H

#include <astar_planner/orientation_filter.h>

#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/PoseStamped.h>

#include <navigation_interface/base_global_planner.h>

#include <ros/ros.h>

#include <mutex>
#include <vector>

namespace astar_planner
{

class AStarPlanner : public navigation_interface::BaseGlobalPlanner
{
  public:
    AStarPlanner();
    ~AStarPlanner();

    void initialize(const std::string& name, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                    const std::shared_ptr<costmap_2d::Costmap2DROS>& global_costmap,
                    const std::shared_ptr<costmap_2d::Costmap2DROS>& local_costmap) override;

    navigation_interface::PlanResult makePlan(const geometry_msgs::PoseStamped& start,
                                              const geometry_msgs::PoseStamped& goal) override;
    virtual double cost(const std::vector<geometry_msgs::PoseStamped>& plan) override;

  private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<costmap_2d::Costmap2DROS> global_costmap_;
    std::shared_ptr<costmap_2d::Costmap2DROS> local_costmap_;

    ros::Publisher plan_pub_;
    ros::Publisher potential_pub_;

    bool publish_potential_;

    std::mutex mutex_;

    OrientationFilter orientation_filter_;

    void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);

    double neutral_cost_;
};
}

#endif
