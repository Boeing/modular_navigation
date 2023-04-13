#ifndef NAVIGATION_INTERFACE_PATH_PLANNER_H
#define NAVIGATION_INTERFACE_PATH_PLANNER_H

#include <gridmap/map_data.h>
#include <navigation_interface/types/path.h>
#include <yaml-cpp/yaml.h>

#include <memory>

#include "rclcpp/rclcpp.hpp"

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
        double std_x;
        double std_y;
        double std_w;
        std::size_t max_samples;
    };

    PathPlanner() = default;
    virtual ~PathPlanner() = default;

    virtual Result plan(const gridmap::AABB& local_region, const Eigen::Isometry2d& start,
                        const Eigen::Isometry2d& goal, const GoalSampleSettings& sample, const double avoid_distance,
                        const double backwards_mult, const double strafe_mult, const double rotation_mult) = 0;

    virtual bool valid(const Path& path, const double avoid_distance) const = 0;
    virtual double cost(const Path& path, const double avoid_distance) const = 0;

    virtual void onInitialize(const YAML::Node& parameters) = 0;
    virtual void onMapDataChanged() = 0;

    void initialize(const YAML::Node& parameters, const std::shared_ptr<const gridmap::MapData>& map_data,
                    const rclcpp::Node::SharedPtr node)
    {
        std::lock_guard<std::mutex> lock(mutex_);        
        map_data_ = map_data;
        node_ = node;
        onInitialize(parameters);
    }

    void setMapData(const std::shared_ptr<const gridmap::MapData>& map_data)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        RCLCPP_INFO(rclcpp::get_logger(""), "Updating map: PathPlanner");
        map_data_ = map_data;
        onMapDataChanged();
        RCLCPP_INFO(rclcpp::get_logger(""), "Updating map: PathPlanner DONE") ;
    }

  protected:
    std::mutex mutex_;
    std::shared_ptr<const gridmap::MapData> map_data_;
    rclcpp::Node::SharedPtr node_;
};
}  // namespace navigation_interface

#endif //NAVIGATION_INTERFACE_PATH_PLANNER_H
