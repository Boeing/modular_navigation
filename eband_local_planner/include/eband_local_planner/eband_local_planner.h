#ifndef EBAND_LOCAL_PLANNER_EBAND_LOCAL_PLANNER_H
#define EBAND_LOCAL_PLANNER_EBAND_LOCAL_PLANNER_H

#include <ros/assert.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

#include <eband_local_planner/conversions_and_types.h>
#include <eband_local_planner/eband_visualization.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <costmap_2d/costmap_2d_ros.h>

namespace eband_local_planner
{

class EBandPlanner
{
  public:
    EBandPlanner(const std::shared_ptr<costmap_2d::Costmap2DROS>& local_costmap, const int num_optim_iterations,
                 const double internal_force_gain, const double external_force_gain, const double tiny_bubble_distance,
                 const double tiny_bubble_expansion, const double min_bubble_overlap,
                 const int equilibrium_max_recursion_depth, const double equilibrium_relative_overshoot,
                 const double significant_force, const double costmap_weight, const double costmap_inflation_radius);
    ~EBandPlanner();

    void setVisualization(std::shared_ptr<EBandVisualization> eband_visual);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);

    bool getPlan(std::vector<geometry_msgs::PoseStamped>& global_plan);

    bool getBand(std::vector<Bubble>& elastic_band);

    bool addFrames(const std::vector<geometry_msgs::PoseStamped>& robot_pose, const AddAtPosition& add_frames_at);

    void optimizeBand();

  private:
    const std::shared_ptr<costmap_2d::Costmap2DROS> local_costmap_;

    const int num_optim_iterations_;      // maximal number of iteration steps during optimization of band
    const double internal_force_gain_;    // gain for internal forces ("Elasticity of Band")
    const double external_force_gain_;    // gain for external forces ("Penalty on low distance to abstacles")
    const double tiny_bubble_distance_;   // internal forces between two bubbles are only calc. if there distance is
                                          // bigger than this lower bound
    const double tiny_bubble_expansion_;  // lower bound for bubble expansion. below this bound bubble is considered as
                                          // "in collision"
    const double min_bubble_overlap_;     // minimum relative overlap two bubbles must have to be treated as connected
    const int max_recursion_depth_approx_equi_;  // maximum depth for recursive approximation to constrain computational
                                                 // burden
    const double equilibrium_relative_overshoot_;  // percentage of old force for which a new force is considered
                                                   // significant when higher as this value
    const double significant_force_;  // lower bound for absolute value of force below which it is treated as
                                      // insignificant (no recursive approximation)

    // These must match the inflation layer params
    const double costmap_weight_;
    const double costmap_inflation_radius_;

    bool visualization_;

    std::shared_ptr<EBandVisualization> eband_visual_;
    costmap_2d::Costmap2D* costmap_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    std::vector<Bubble> elastic_band_;

    const double robot_radius_;

    void optimizeBand(std::vector<Bubble>& band) const;
    void updateDistances(std::vector<Bubble>& band) const;
    void refineBand(std::vector<Bubble>& band) const;
    void modifyBandArtificialForce(std::vector<Bubble>& band) const;

    Bubble applyForce(const geometry_msgs::Wrench& wrench, const Bubble& prev_bubble, const Bubble& curr_bubble,
                      const Bubble& next_bubble) const;
    Bubble moveBubble(const geometry_msgs::Wrench& wrench, const Bubble& curr_bubble, const double step_size) const;
    Bubble moveToEquilibrium(const geometry_msgs::Wrench& wrench, const Bubble& prev_bubble, const Bubble& curr_bubble,
                             const Bubble& next_bubble) const;

    geometry_msgs::Wrench force(const Bubble& prev_bubble, const Bubble& curr_bubble, const Bubble& next_bubble) const;
    geometry_msgs::Wrench internalForce(const Bubble& prev_bubble, const Bubble& curr_bubble,
                                        const Bubble& next_bubble) const;
    geometry_msgs::Wrench externalForce(const Bubble& curr_bubble) const;
    geometry_msgs::Wrench tangentialForce(const geometry_msgs::Wrench& wrench, const Bubble& prev_bubble,
                                          const Bubble& curr_bubble, const Bubble& next_bubble) const;
};
}

#endif
