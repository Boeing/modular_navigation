#ifndef PURE_PURSUIT_CONTROLLER_PLUGIN_H
#define PURE_PURSUIT_CONTROLLER_PLUGIN_H

#include <ros/ros.h>

#include <memory>
#include <vector>

#include <navigation_interface/controller.h>

#include <gridmap/map_data.h>

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

    virtual Result control(const ros::SteadyTime& time, const navigation_interface::KinodynamicState& robot_state,
                           const Eigen::Isometry2d& map_to_odom) override;

    virtual void onInitialize(const XmlRpc::XmlRpcValue& parameters) override;
    virtual void onMapDataChanged() override;

  private:
    // Runtime values
    std::unique_ptr<const navigation_interface::Trajectory> trajectory_;
    ros::SteadyTime last_update_;

    // Parameters
    double look_ahead_ = 0.1;

    double max_velocity_x_ = 0.20;
    double max_velocity_y_ = 0.05;
    double max_velocity_w_ = 0.2;

    double max_acceleration_x_ = 0.1;
    double max_acceleration_y_ = 0.1;
    double max_acceleration_w_ = 0.2;

    double goal_radius_ = 0.1;

    double xy_goal_tolerance_ = 0.002;
    double yaw_goal_tolerance_ = 0.01;

    Eigen::Vector3d goal_p_gain_ = {1.0, 1.0, 2.0};
    Eigen::Vector3d goal_i_gain_ = {0.0, 0.0, 0.0};
    Eigen::Vector3d goal_d_gain_ = {0.0, 0.0, 0.0};

    Eigen::Vector3d control_error_;
    Eigen::Vector3d control_integral_;

    bool debug_viz_ = true;
    ros::Publisher footprint_pub_;
    ros::Publisher future_footprint_pub_;

    std::vector<Eigen::Vector2d> robot_footprint_;
};
}

#endif
