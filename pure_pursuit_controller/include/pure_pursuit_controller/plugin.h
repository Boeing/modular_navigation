#ifndef PURE_PURSUIT_CONTROLLER_PLUGIN_H
#define PURE_PURSUIT_CONTROLLER_PLUGIN_H

#include <gridmap/map_data.h>
#include <navigation_interface/controller.h>
#include <ros/ros.h>

#include <memory>
#include <vector>

namespace pure_pursuit_controller
{

class PurePursuitController : public navigation_interface::Controller
{
  public:
    PurePursuitController();
    virtual ~PurePursuitController() override;

    virtual bool setTrajectory(const navigation_interface::Trajectory& trajectory) override;
    virtual void clearTrajectory() override;

    virtual boost::optional<std::string> trajectoryId() const override;
    virtual boost::optional<navigation_interface::Trajectory> trajectory() const override;

    virtual Result control(const ros::SteadyTime& time, const gridmap::AABB& local_region,
                           const navigation_interface::KinodynamicState& robot_state,
                           const Eigen::Isometry2d& map_to_odom) override;

    virtual void onInitialize(const XmlRpc::XmlRpcValue& parameters) override;
    virtual void onMapDataChanged() override;

  private:
    // Runtime values
    std::size_t path_index_ = 0;
    std::unique_ptr<const navigation_interface::Trajectory> trajectory_;
    ros::SteadyTime last_update_;

    // Parameters
    double look_ahead_time_ = 0.5;

    Eigen::Vector3d max_velocity_ = {0.25, 0.15, 0.25};
    Eigen::Vector3d max_acceleration_ = {0.5, 0.5, 0.5};

    double goal_radius_ = 0.08;

    double xy_goal_tolerance_ = 0.002;
    double yaw_goal_tolerance_ = 0.01;

    Eigen::Vector3d goal_p_gain_ = {0.5, 0.5, 0.2};
    Eigen::Vector3d goal_i_gain_ = {0.0, 0.0, 0.0};
    Eigen::Vector3d goal_d_gain_ = {0.0, 0.0, 0.0};

    Eigen::Vector3d control_error_;
    Eigen::Vector3d control_integral_;

    bool debug_viz_ = true;
    ros::Publisher target_state_pub_;
    ros::Publisher footprint_pub_;
    ros::Publisher future_footprint_pub_;

    std::vector<Eigen::Vector2d> robot_footprint_;
};
}  // namespace pure_pursuit_controller

#endif
