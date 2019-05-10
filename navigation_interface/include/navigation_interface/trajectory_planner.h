#ifndef NAVIGATION_INTERFACE_BASE_LOCAL_PLANNER_H
#define NAVIGATION_INTERFACE_BASE_LOCAL_PLANNER_H

#include <navigation_interface/types/trajectory.h>
#include <navigation_interface/types/path.h>

#include <costmap_2d/costmap_2d.h>

#include <xmlrpcpp/XmlRpc.h>

#include <boost/optional.hpp>

#include <memory>

namespace navigation_interface
{

class TrajectoryPlanner
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

        // trajectory includes [start_i, end_i) of Path
        std::size_t path_start_i;
        std::size_t path_end_i;

        Trajectory trajectory;
    };

    virtual bool setPath(const Path& path) = 0;
    virtual void clearPath() = 0;

    virtual boost::optional<std::string> pathId() const = 0;
    virtual boost::optional<Path> path() const = 0;

    virtual Result plan(const KinodynamicState& robot_state, const Eigen::Isometry2d& map_to_odom) = 0;

    virtual bool valid(const Trajectory& trajectory) const = 0;
    virtual double cost(const Trajectory& trajectory) const = 0;

    virtual void initialize(const XmlRpc::XmlRpcValue& parameters,
                            const std::shared_ptr<const costmap_2d::Costmap2D>& costmap) = 0;

    virtual ~TrajectoryPlanner()
    {
    }

  protected:
    TrajectoryPlanner()
    {
    }
};
}

#endif
