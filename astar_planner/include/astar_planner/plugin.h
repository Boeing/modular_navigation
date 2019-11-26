#ifndef ASTAR_PLANNER_PLUGIN_H
#define ASTAR_PLANNER_PLUGIN_H

#include <astar_planner/costmap.h>
#include <gridmap/map_data.h>
#include <navigation_interface/path_planner.h>
#include <opencv2/core.hpp>
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
    double robot_radius_ = 0.210;
    double conservative_robot_radius_ = 0.480;
    double max_holonomic_distance_ = 2.0;
    double max_reverse_distance_ = 4.0;
    double avoid_zone_cost_ = 10.0;
    double path_cost_ = 1e-3;

    std::vector<Eigen::Vector2d> offsets_;

    ros::Publisher explore_pub_;

    std::shared_ptr<Costmap> costmap_;
    std::shared_ptr<cv::Mat> traversal_cost_;
    std::shared_ptr<CollisionChecker> collision_checker_;
};
}  // namespace astar_planner

#endif
