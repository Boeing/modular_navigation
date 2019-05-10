#ifndef OMNI_PID_CONTROLLER_PLUGIN_H
#define OMNI_PID_CONTROLLER_PLUGIN_H

#include <ros/ros.h>

#include <memory>
#include <vector>

#include <navigation_interface/controller.h>

#include <costmap_2d/costmap_2d.h>

namespace omni_pid_controller
{

struct ControlGains
{
    const Eigen::Vector3d control_p_gain = {0.0, 0.0, 1.0};
    const Eigen::Vector3d control_i_gain = {0.0, 0.0, 0.0};
    const Eigen::Vector3d control_d_gain = {0.0, 0.0, 0.5};

    const Eigen::Vector3d control_vp_gain = {1.0, 1.0, 0.0};
    const Eigen::Vector3d control_vi_gain = {0.0, 0.0, 0.0};
    const Eigen::Vector3d control_vd_gain = {0.0, 0.0, 0.0};

    const Eigen::Vector3d goal_p_gain = {1.0, 1.0, 1.0};
    const Eigen::Vector3d goal_i_gain = {0.0, 0.0, 0.0};
    const Eigen::Vector3d goal_d_gain = {0.0, 0.0, 0.0};
};

class OmniPIDController : public navigation_interface::Controller
{
  public:
    OmniPIDController();
    virtual ~OmniPIDController() override;

    virtual bool setTrajectory(const navigation_interface::Trajectory& trajectory) override;
    virtual void clearTrajectory() override;

    virtual boost::optional<std::string> trajectoryId() const override;
    virtual boost::optional<navigation_interface::Trajectory> trajectory() const override;

    virtual Result control(const ros::SteadyTime& time, const navigation_interface::KinodynamicState& robot_state, const Eigen::Isometry2d& map_to_odom) override;

    virtual void initialize(const XmlRpc::XmlRpcValue& parameters,
                            const std::shared_ptr<const costmap_2d::Costmap2D>& costmap) override;

  private:
    std::shared_ptr<const costmap_2d::Costmap2D> costmap_;

    // Runtime values
    std::unique_ptr<const navigation_interface::Trajectory> trajectory_;
    std::size_t target_i_;
    Eigen::Vector3d control_error_;
    Eigen::Vector3d velocity_integral_;
    ros::SteadyTime last_update_;

    // Parameters
    double max_velocity_x_ = 0.15;
    double max_velocity_y_ = 0.05;
    double max_velocity_w_ = 0.1;

    double max_acceleration_x_ = 0.1;
    double max_acceleration_y_ = 0.1;
    double max_acceleration_w_ = 0.1;

    double goal_radius_ = 0.06;

    double xy_goal_tolerance_ = 0.002;
    double yaw_goal_tolerance_ = 0.01;

    ControlGains control_gains_;

};
}

#endif
