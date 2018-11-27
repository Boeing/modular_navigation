#ifndef EBAND_LOCAL_PLANNER_EBAND_LOCAL_PLANNER_H
#define EBAND_LOCAL_PLANNER_EBAND_LOCAL_PLANNER_H

//  #define DEBUG_EBAND_

#include <ros/assert.h>
#include <ros/ros.h>

#include <string>
#include <vector>

#include <eband_local_planner/conversions_and_types.h>
#include <eband_local_planner/costmap_model.h>
#include <eband_local_planner/eband_visualization.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <angles/angles.h>
#include <tf/tf.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <boost/shared_ptr.hpp>

namespace eband_local_planner
{

class EBandPlanner
{
  public:
    EBandPlanner();
    EBandPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    ~EBandPlanner();

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief passes a reference to the eband visualization object which can be used to visualize the band optimization
     * @param pointer to visualization object
     */
    void setVisualization(boost::shared_ptr<EBandVisualization> eband_visual);

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
    // pointer to external objects (do NOT delete object)
    costmap_2d::Costmap2DROS* costmap_ros_;

    // flags
    bool initialized_;
    bool visualization_;

    // parameters
    std::vector<double> acc_lim_;   // acceleration limits for translational and rotational motion
    int num_optim_iterations_;      // maximal number of iteration steps during optimization of band
    double internal_force_gain_;    // gain for internal forces ("Elasticity of Band")
    double external_force_gain_;    // gain for external forces ("Penalty on low distance to abstacles")
    double tiny_bubble_distance_;   // internal forces between two bubbles are only calc. if there distance is bigger
                                    // than this lower bound
    double tiny_bubble_expansion_;  // lower bound for bubble expansion. below this bound bubble is considered as "in
                                    // collision"
    double min_bubble_overlap_;     // minimum relative overlap two bubbles must have to be treated as connected
    int max_recursion_depth_approx_equi_;    // maximum depth for recursive approximation to constrain computational
                                             // burden
    double equilibrium_relative_overshoot_;  // percentage of old force for which a new force is considered significant
                                             // when higher as this value
    double significant_force_;  // lower bound for absolute value of force below which it is treated as insignificant
                                // (no recursive approximation)
    double costmap_weight_;     // the costmap weight or scaling factor

    // pointer to locally created objects (delete)
    CostmapModel* world_model_;                           // local world model
    boost::shared_ptr<EBandVisualization> eband_visual_;  // pointer to visualization object

    // data
    std::vector<geometry_msgs::Point> footprint_spec_;  // specification of robot footprint as vector of corner points
    costmap_2d::Costmap2D* costmap_;                    // pointer to underlying costmap
    std::vector<geometry_msgs::PoseStamped> global_plan_;  // copy of the plan that shall be optimized
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

    /**
     * @brief interpolates between two bubbles by calculating the pose for the center of a bubble in the middle of the
     * path connecting the bubbles [depends kinematics]
     * @param center of first of the two bubbles between which shall be interpolated
     * @param center of second of the two bubbles between which shall be interpolated
     * @param reference to hand back the interpolated bubble's center
     * @return true if interpolation was successful
     */
    bool interpolateBubbles(geometry_msgs::PoseStamped start_center, geometry_msgs::PoseStamped end_center,
                            geometry_msgs::PoseStamped& interpolated_center);

    /**
     * @brief this checks whether two bubbles overlap
     * @param band on which we want to check
     * @param iterator to first bubble
     * @param iterator to second bubble
     * @return true if bubbles overlap
     */
    bool checkOverlap(Bubble bubble1, Bubble bubble2);

    /**
     * @brief This calculates the distance between two bubbles [depends kinematics, shape]
     * @param pose of the center of first bubbles
     * @param pose of the center of second bubbles
     * @param refernce to variable to pass distance to caller function
     * @return true if distance was successfully calculated
     */
    bool calcBubbleDistance(geometry_msgs::Pose start_center_pose, geometry_msgs::Pose end_center_pose,
                            double& distance);

    /**
     * @brief Calculates the difference between the pose of the center of two bubbles and outputs it as Twist (pointing
     * from first to second bubble) [depends kinematic]
     * @param pose of the first bubble
     * @param pose of the second bubble
     * @param reference to variable in wich the difference is stored as twist
     * @return true if difference was successfully calculated
     */
    bool calcBubbleDifference(geometry_msgs::Pose start_center_pose, geometry_msgs::Pose end_center_pose,
                              geometry_msgs::Twist& difference);

    /**
     * @brief Calculates the distance between the center of a bubble and the closest obstacle [depends kinematics,
     * shape, environment]
     * @param center of bubble as Pose
     * @param reference to distance variable
     * @return True if successfully calculated distance false otherwise
     */
    bool calcObstacleKinematicDistance(geometry_msgs::Pose center_pose, double& distance);

    /**
     * @brief Calculates all forces for a certain bubble at a specific position in the band [depends kinematic]
     * @param position in band for which internal forces shall be calculated
     * @param band for which forces shall be calculated
     * @param Bubble for which internal forces shall be calculated
     * @param reference to variable via which forces and torques are given back
     * @return true if forces were calculated successfully
     */
    bool getForcesAt(int bubble_num, std::vector<Bubble> band, Bubble curr_bubble,
                     geometry_msgs::WrenchStamped& forces);

    /**
     * @brief Calculates internal forces for bubbles along the band [depends kinematic]
     * @param position in band for which internal forces shall be calculated
     * @param band for which forces shall be calculated
     * @param Bubble for which internal forces shall be calculated
     * @param reference to variable via which forces and torques are given back
     * @return true if forces were calculated successfully
     */
    bool calcInternalForces(int bubble_num, std::vector<Bubble> band, Bubble curr_bubble,
                            geometry_msgs::WrenchStamped& forces);

    /**
     * @brief Calculates external forces for bubbles along the band [depends shape, environment]
     * @param position in band for which internal forces shall be calculated
     * @param Bubble for which external forces shall be calculated
     * @param reference to variable via which forces and torques are given back
     * @return true if forces were calculated successfully
     */
    bool calcExternalForces(int bubble_num, Bubble curr_bubble, geometry_msgs::WrenchStamped& forces);

    /**
     * @brief Calculates tangential portion of forces
     * @param number of bubble for which forces shall be recalculated
     * @param reference to variable via which forces and torques are given back
     * @return true if forces were calculated successfully
     */
    bool suppressTangentialForces(int bubble_num, std::vector<Bubble> band, geometry_msgs::WrenchStamped& forces);

    /**
     * @brief This converts a plan from a sequence of stamped poses into a band, a sequence of bubbles. Therefore it
     * calculates distances to the surrounding obstacles and derives the expansion or radius of the bubble
     * @param plan is the sequence of stamped Poses which shall be converted
     * @param band is the resulting sequence of bubbles
     * @return true if path was successfully converted - band did not break
     */
    bool convertPlanToBand(std::vector<geometry_msgs::PoseStamped> plan, std::vector<Bubble>& band);

    /**
     * @brief This converts a band from a sequence of bubbles into a plan from, a sequence of stamped poses
     * @param band is the sequence of bubbles which shall be converted
     * @param plan is the resulting sequence of stamped Poses
     * @return true if path was successfully converted - band did not break
     */
    bool convertBandToPlan(std::vector<geometry_msgs::PoseStamped>& plan, std::vector<Bubble> band);
};

}  // namespace eband_local_planner

#endif  // EBAND_LOCAL_PLANNER_EBAND_LOCAL_PLANNER_H
