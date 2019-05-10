#ifndef ASTAR_PLANNER_PLUGIN_H
#define ASTAR_PLANNER_PLUGIN_H

#include <astar_planner/orientation_filter.h>

#include <costmap_2d/costmap_2d.h>

#include <navigation_interface/path_planner.h>

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

    virtual void initialize(const XmlRpc::XmlRpcValue& parameters,
                            const std::shared_ptr<const costmap_2d::Costmap2D>& costmap) override;

  private:
    std::shared_ptr<const costmap_2d::Costmap2D> costmap_;

    OrientationFilter orientation_filter_;

    double neutral_cost_ = 30.0;
};
}

#endif
