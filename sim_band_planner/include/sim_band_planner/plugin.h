#ifndef sim_band_planner_PLUGIN_H
#define sim_band_planner_PLUGIN_H

#include <memory>

#include <navigation_interface/trajectory_planner.h>

#include <sim_band_planner/moving_window.h>
#include <sim_band_planner/simulate.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <gridmap/map_data.h>

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
                        const Eigen::Isometry2d& map_to_odom) override;

    virtual bool valid(const navigation_interface::Trajectory& trajectory) const override;
    virtual double cost(const navigation_interface::Trajectory& trajectory) const override;

    virtual void onInitialize(const XmlRpc::XmlRpcValue& parameters) override;
    virtual void onMapDataChanged() override;

    boost::optional<Band> band() const
    {
        if (moving_window_)
            return boost::optional<Band>(moving_window_->window);
        else
            return {};
    }

  private:
    std::unique_ptr<rviz_visual_tools::RvizVisualTools> viz_;

    // Runtime data
    std::unique_ptr<MovingWindow> moving_window_;

    // Configuration
    bool debug_viz_ = true;
    int num_iterations_ = 10;
    double internal_force_gain_ = 0.004;
    double external_force_gain_ = 0.004;
    double min_distance_ = 0.02;
    double max_distance_ = 1.0;
    double min_overlap_ = 0.8;
    double max_window_length_ = 4.0;
    double robot_radius_ = 0.7;
    double rotation_factor_ = 1.0;
    double velocity_decay_ = 0.6;
    double alpha_decay_ = 1.0 / std::pow(0.001, 1.0 / 20.0);
    double desired_speed_ = 0.2;
    bool spline_ = true;
};
}

#endif
