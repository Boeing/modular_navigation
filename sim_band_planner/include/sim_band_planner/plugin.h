#ifndef sim_band_planner_PLUGIN_H
#define sim_band_planner_PLUGIN_H

#include <gridmap/map_data.h>
#include <navigation_interface/trajectory_planner.h>
#include <ros/ros.h>
#include <sim_band_planner/moving_window.h>
#include <sim_band_planner/simulate.h>

#include <memory>

namespace sim_band_planner
{

class SimBandPlanner : public navigation_interface::TrajectoryPlanner
{
  public:
    SimBandPlanner();
    virtual ~SimBandPlanner() override;

    virtual bool setPath(const navigation_interface::Path& path) override;
    virtual void clearPath() override;

    virtual boost::optional<std::string> pathId() const override;
    virtual boost::optional<navigation_interface::Path> path() const override;

    virtual Result plan(const gridmap::AABB& local_region, const navigation_interface::KinodynamicState& robot_state,
                        const Eigen::Isometry2d& map_to_odom, const double avoid_distance) override;

    virtual bool valid(const navigation_interface::Trajectory& trajectory, const double) const override;
    virtual double cost(const navigation_interface::Trajectory& trajectory, const double) const override;

    virtual void onInitialize(const YAML::Node& parameters) override;
    virtual void onMapDataChanged() override;

    boost::optional<Band> band() const
    {
        if (moving_window_)
            return boost::optional<Band>(moving_window_->window);
        else
            return {};
    }

  private:
    ros::Publisher marker_pub_;

    // Runtime data
    std::unique_ptr<MovingWindow> moving_window_;

    // Configuration
    bool debug_viz_ = true;
    int num_iterations_ = 100;

    double collision_distance_ = 0.5;
    double nominal_force_gain_ = 0.02;
    double internal_force_gain_ = 0.002;
    double external_force_gain_ = 0.001;
    double rotation_gain_ = 0.005;

    double max_window_length_ = 4.0;
    double velocity_decay_ = 0.6;
    double alpha_decay_ = 1.0 - std::pow(0.001, 1.0 / 100.0);

    Eigen::Vector3d max_acc_ = {0.1, 0.1, 0.2};

    double robot_radius_ = 0.240;
    std::vector<Eigen::Vector2d> offsets_;
};
}  // namespace sim_band_planner

#endif
