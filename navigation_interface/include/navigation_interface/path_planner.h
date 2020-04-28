#ifndef NAVIGATION_INTERFACE_PATH_PLANNER_H
#define NAVIGATION_INTERFACE_PATH_PLANNER_H

#include <gridmap/map_data.h>
#include <navigation_interface/types/path.h>
#include <yaml-cpp/yaml.h>

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

    struct GoalSampleSettings
    {
        const double std_x;
        const double std_y;
        const double std_w;
        const std::size_t max_samples;
    };

    PathPlanner() = default;
    virtual ~PathPlanner() = default;

    virtual Result plan(const Eigen::Isometry2d& start, const Eigen::Isometry2d& goal,
                        const GoalSampleSettings& sample) = 0;

    virtual bool valid(const Path& path) const = 0;
    virtual double cost(const Path& path) const = 0;

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
        ROS_INFO("Updating map: PathPlanner");
        std::lock_guard<std::mutex> lock(mutex_);
        map_data_ = map_data;
        onMapDataChanged();
        ROS_INFO("Updating map: PathPlanner DONE");
    }

  protected:
    std::mutex mutex_;
    std::shared_ptr<const gridmap::MapData> map_data_;
};
}  // namespace navigation_interface

#endif
