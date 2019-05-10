#ifndef sim_band_planner_PLUGIN_H
#define sim_band_planner_PLUGIN_H

#include <memory>

#include <navigation_interface/trajectory_planner.h>

#include <sim_band_planner/moving_window.h>
#include <sim_band_planner/simulate.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <costmap_2d/costmap_2d.h>

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

    virtual Result plan(const navigation_interface::KinodynamicState& robot_state, const Eigen::Isometry2d& map_to_odom) override;

    virtual bool valid(const navigation_interface::Trajectory& trajectory) const override;
    virtual double cost(const navigation_interface::Trajectory& trajectory) const override;

    virtual void initialize(const XmlRpc::XmlRpcValue& parameters,
                            const std::shared_ptr<const costmap_2d::Costmap2D>& costmap) override;

    boost::optional<Band> band() const
    {
        if (moving_window_)
            return boost::optional<Band>(moving_window_->window);
        else
            return {};
    }

  private:
    std::shared_ptr<const costmap_2d::Costmap2D> costmap_;

    std::unique_ptr<rviz_visual_tools::RvizVisualTools> viz_;

    // Runtime data
    std::unique_ptr<MovingWindow> moving_window_;

    // Configuration
    int num_iterations_ = 6;
    double internal_force_gain_ = 0.004;
    double external_force_gain_ = 0.002;
    double min_distance_ = 0.02;
    double min_overlap_ = 0.2;
    double max_window_length_ = 1.5;
    double robot_radius_ = 0.5;
    double rotation_factor_ = 1.0;
    double velocity_decay_ = 0.6;
    double alpha_decay_ = 1.0 / std::pow(0.001, 1.0 / 12.0);
};
}

#endif
