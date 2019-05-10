#ifndef NAVIGATION_INTERFACE_BASE_LOCAL_PLANNER_H_BASE_GLOBAL_PLANNER_H
#define NAVIGATION_INTERFACE_BASE_LOCAL_PLANNER_H_BASE_GLOBAL_PLANNER_H

#include <navigation_interface/types/path.h>

#include <costmap_2d/costmap_2d.h>

#include <xmlrpcpp/XmlRpc.h>

#include <memory>

namespace navigation_interface
{

class PathPlanner
{
  public:

    enum class Outcome
    {
        FAILED,
        SUCCESSFUL,
        PARTIAL
    };

    struct Result
    {
        Outcome outcome;
        double cost;
        Path path;
    };

    virtual Result plan(const Eigen::Isometry2d& start, const Eigen::Isometry2d& goal) = 0;

    virtual bool valid(const Path& path) const = 0;
    virtual double cost(const Path& path) const = 0;

    virtual void initialize(const XmlRpc::XmlRpcValue& parameters,
                            const std::shared_ptr<const costmap_2d::Costmap2D>& costmap) = 0;

    virtual ~PathPlanner()
    {
    }

  protected:
    PathPlanner()
    {
    }
};
}

#endif