#ifndef NAVIGATION_INTERFACE_BASE_LOCAL_PLANNER_H
#define NAVIGATION_INTERFACE_BASE_LOCAL_PLANNER_H

#include <boost/optional.hpp>
#include <gridmap/map_data.h>
#include <navigation_interface/types/path.h>
#include <navigation_interface/types/trajectory.h>
#include <yaml-cpp/yaml.h>

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

    TrajectoryPlanner() = default;
    virtual ~TrajectoryPlanner() = default;

    virtual bool setPath(const Path& path) = 0;
    virtual void clearPath() = 0;

    virtual boost::optional<std::string> pathId() const = 0;
    virtual boost::optional<Path> path() const = 0;

    virtual Result plan(const gridmap::AABB& local_region, const KinodynamicState& robot_state,
                        const Eigen::Isometry2d& map_to_odom, const double avoid_distance) = 0;

    virtual bool valid(const Trajectory& trajectory, const double avoid_distance) const = 0;
    virtual double cost(const Trajectory& trajectory, const double avoid_distance) const = 0;

    virtual void onInitialize(const YAML::Node& parameters) = 0;
    virtual void onMapDataChanged() = 0;

    void initialize(const YAML::Node& parameters, const std::shared_ptr<const gridmap::MapData>& map_data)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        map_data_ = map_data;
        onInitialize(parameters);
    }

    void setMapData(const std::shared_ptr<const gridmap::MapData>& map_data)
    {
        RCLCPP_INFO(rclcpp::get_logger(""), "Updating map: TrajectoryPlanner");
        std::lock_guard<std::mutex> lock(mutex_);
        map_data_ = map_data;
        onMapDataChanged();
        RCLCPP_INFO(rclcpp::get_logger(""), "Updating map: TrajectoryPlanner DONE");
    }

  protected:
    std::mutex mutex_;
    std::shared_ptr<const gridmap::MapData> map_data_;
};
}  // namespace navigation_interface

#endif
