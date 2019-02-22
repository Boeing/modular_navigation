#ifndef EBAND_LOCAL_PLANNER_EBAND_LOCAL_PLANNER_H
#define EBAND_LOCAL_PLANNER_EBAND_LOCAL_PLANNER_H

// #define DEBUG_EBAND_

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
                 const double significant_force, const double costmap_weight);
    ~EBandPlanner();

    /**
     * @brief passes a reference to the eband visualization object which can be used to visualize the band optimization
     * @param pointer to visualization object
     */
    void setVisualization(std::shared_ptr<EBandVisualization> eband_visual);

    /**
     * @brief Set plan which shall be optimized to elastic band planner
     * @param global_plan The plan which shall be optimized
     * @return True if the plan was set successfully
     */
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);

    /**
     * @brief This transforms the refined band to a path again and outputs it
     * @param reference via which the path is passed to the caller
     * @return true if path could be handed over
     */
    bool getPlan(std::vector<geometry_msgs::PoseStamped>& global_plan);

    /**
     * @brief This outputs the elastic_band
     * @param reference via which the band is passed to the caller
     * @return true if there was a band to pass
     */
    bool getBand(std::vector<Bubble>& elastic_band);

    /**
     * @brief converts robot_pose to a bubble and tries to connect to the path
     * @param pose of the robot which shall be connected to the band
     * @return true if pose could be connected
     */
    bool addFrames(const std::vector<geometry_msgs::PoseStamped>& robot_pose, const AddAtPosition& add_frames_at);

    /**
     * @brief cycles over the elastic band set before and optimizes it locally by minimizing an energy-function
     * @return true if band is valid
     */
    bool optimizeBand();

    /**
     * @brief cycles over the elastic band checks it for validity and optimizes it locally by minimizing an
     * energy-function
     * @param reference to band which shall be optimized
     * @return true if band is valid
     */
    bool optimizeBand(std::vector<Bubble>& band);

  private:
    const std::shared_ptr<costmap_2d::Costmap2DROS> local_costmap_;

    // parameters
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
    const double costmap_weight_;     // the costmap weight or scaling factor

    bool visualization_;

    std::shared_ptr<EBandVisualization> eband_visual_;
    costmap_2d::Costmap2D* costmap_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    std::vector<Bubble> elastic_band_;

    /**
     * @brief Cycles over an band and searches for gaps and redundant bubbles. Tries to close gaps and remove redundant
     * bubbles.
     * @param band is a reference to the band that shall be refined
     * @return true if all gaps could be closed and band is valid
     */
    bool refineBand(std::vector<Bubble>& band);

    /**
     * @brief recursively checks an intervall of a band whether gaps need to be filled or bubbles may be removed and
     * fills or removes if possible. This exploits the sequential structure of the band. Therefore it can not detect
     * redundant cycles which are closed over several on its own not redundant bubbles.
     * @param reference to the elastic band that shall be checked
     * @param reference to the iterator pointing to the start of the intervall that shall be checked
     * @param reference to the iterator pointing to the end of the intervall that shall be checked
     * @return true if segment is valid (all gaps filled), falls if band is broken within this segment
     */
    bool removeAndFill(std::vector<Bubble>& band, std::vector<Bubble>::iterator& start_iter,
                       std::vector<Bubble>::iterator& end_iter);


    /**
     * @brief recursively fills gaps between two bubbles (if possible)
     * @param reference to the elastic band that is worked on
     * @param reference to the iterator pointing to the start of the intervall that shall be filled
     * @param reference to the iterator pointing to the end of the intervall that shall be filled
     * @return true if gap was successfully filled otherwise false (band broken)
     */
    bool fillGap(std::vector<Bubble>& band, std::vector<Bubble>::iterator& start_iter,
                 std::vector<Bubble>::iterator& end_iter);

    /**
     * @brief calculates internal and external forces and applies changes to the band
     * @param reference to band which shall be modified
     * @return true if band could be modified, all steps finished cleanly
     */
    bool modifyBandArtificialForce(std::vector<Bubble>& band);

    /**
     * @brief Applies forces to move bubbles and recalculates expansion of bubbles
     * @param reference to number of bubble which shall be modified - number might be modified if additional bubbles are
     * introduced to fill band
     * @param reference to band which shall be modified
     * @param forces and torques which shall be applied
     * @return true if modified band valid, false if band is broken (one or more bubbles in collision)
     */
    bool applyForces(int bubble_num, std::vector<Bubble>& band, std::vector<geometry_msgs::WrenchStamped> forces);


    /**
     * @brief Checks for zero-crossings and large changes in force-vector after moving of bubble - if detected it tries
     * to approximate the equilibrium point
     * @param position of the checked bubble within the band
     * @param band within which equilibrium position shall be approximated
     * @param force calculated for last bubble pose
     * @param current step width
     * @param current recursion depth to constrain number of recurions
     * @return true if recursion successfully
     */
    bool moveApproximateEquilibrium(const int& bubble_num, const std::vector<Bubble>& band, Bubble& curr_bubble,
                                    const geometry_msgs::WrenchStamped& curr_bubble_force,
                                    geometry_msgs::Twist& curr_step_width, const int& curr_recursion_depth);

    bool getForcesAt(int bubble_num, std::vector<Bubble> band, Bubble curr_bubble,
                     geometry_msgs::WrenchStamped& forces);

    bool calcInternalForces(int bubble_num, std::vector<Bubble> band, Bubble curr_bubble,
                            geometry_msgs::WrenchStamped& forces);

    bool calcExternalForces(int bubble_num, Bubble curr_bubble, geometry_msgs::WrenchStamped& forces);

    bool suppressTangentialForces(int bubble_num, std::vector<Bubble> band, geometry_msgs::WrenchStamped& forces);

    bool interpolateBubbles(geometry_msgs::PoseStamped start_center, geometry_msgs::PoseStamped end_center,
                            geometry_msgs::PoseStamped& interpolated_center);

    bool checkOverlap(Bubble bubble1, Bubble bubble2);
};
}

#endif
