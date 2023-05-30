#ifndef NAVIGATION_INTERFACE_CONTROLLER_INTERFACE_H
#define NAVIGATION_INTERFACE_CONTROLLER_INTERFACE_H

#include <boost/optional.hpp>
#include <gridmap/map_data.h>
#include <navigation_interface/types/control.h>
#include <navigation_interface/types/trajectory.h>
#include <yaml-cpp/yaml.h>

#include <memory>

namespace navigation_interface
{

class Controller
{
  public:
    enum class Outcome
    {
        FAILED,
        SUCCESSFUL,
        COMPLETE
    };

    struct Result
    {
        Outcome outcome;

        std::size_t target_i;

        Eigen::VectorXd command;
    };
    Controller() = default;
    virtual ~Controller() = default;

    virtual bool setTrajectory(const Trajectory& trajectory) = 0;
    virtual void clearTrajectory() = 0;

    virtual boost::optional<std::string> trajectoryId() const = 0;
    virtual boost::optional<Trajectory> trajectory() const = 0;

    virtual Result control(const rclcpp::Time& time, const gridmap::AABB& local_region,
                           const KinodynamicState& robot_state, const Eigen::Isometry2d& map_to_odom,
                           const Eigen::Vector3d max_velocity, const double xy_goal_tolerance,
                           const double yaw_goal_tolerance) = 0;

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
        RCLCPP_INFO(rclcpp::get_logger(""), "Updating map: Controller");
        std::lock_guard<std::mutex> lock(mutex_);
        map_data_ = map_data;
        onMapDataChanged();
        RCLCPP_INFO(rclcpp::get_logger(""), "Updating map: Controller DONE");
    }

  protected:
    std::mutex mutex_;
    std::shared_ptr<const gridmap::MapData> map_data_;
    rclcpp::Node::SharedPtr node_;
};
};  // namespace navigation_interface

#endif
