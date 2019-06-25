#ifndef ASTAR_PLANNER_PLUGIN_H
#define ASTAR_PLANNER_PLUGIN_H

#include <gridmap/map_data.h>

#include <opencv2/core.hpp>

#include <navigation_interface/path_planner.h>

#include <ros/ros.h>

namespace astar_planner
{

class AStarPlanner : public navigation_interface::PathPlanner
{
  public:
    AStarPlanner();
    ~AStarPlanner();

    virtual Result plan(const Eigen::Isometry2d& start, const Eigen::Isometry2d& goal) override;

    virtual bool valid(const navigation_interface::Path& path) const override;
    virtual double cost(const navigation_interface::Path& path) const override;

    virtual void onInitialize(const XmlRpc::XmlRpcValue& parameters) override;
    virtual void onMapDataChanged() override;

  private:
    bool debug_viz_ = false;
    double neutral_cost_ = 0.1;
    double robot_radius_ = 0.5;
    double exponential_weight_ = 2.0;
    int down_sample_ = 4;

    ros::Publisher pub_;
    ros::Publisher explore_pub_;
};
}

#endif
