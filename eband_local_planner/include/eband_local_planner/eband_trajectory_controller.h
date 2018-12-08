#ifndef EBAND_LOCAL_PLANNER_EBAND_TRAJECTORY_CONTROLLER_H
#define EBAND_LOCAL_PLANNER_EBAND_TRAJECTORY_CONTROLLER_H

#include <ros/assert.h>
#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

#include <eband_local_planner/conversions_and_types.h>
#include <eband_local_planner/eband_visualization.h>
#include <eband_local_planner/pid.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>

namespace eband_local_planner
{

class EBandTrajectoryCtrl
{
  public:
    EBandTrajectoryCtrl(costmap_2d::Costmap2DROS* costmap_ros, const double max_vel_lin, const double max_vel_th,
                        const double min_vel_lin, const double min_vel_th, const double min_in_place_vel_th,
                        const double in_place_trans_vel, const double xy_goal_tolerance,
                        const double yaw_goal_tolerance, const double k_prop, const double k_damp,
                        const double ctrl_rate, const double max_acceleration, const double virtual_mass,
                        const double max_translational_acceleration, const double max_rotational_acceleration,
                        const double rotation_correction_threshold);
    ~EBandTrajectoryCtrl();

    void setVisualization(std::shared_ptr<EBandVisualization> target_visual);

    bool setBand(const std::vector<Bubble>& elastic_band);

    bool setOdometry(const nav_msgs::Odometry& odometry);

    bool getTwist(geometry_msgs::Twist& twist_cmd, bool& goal_reached);

  private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    std::shared_ptr<EBandVisualization> target_visual_;

    Pid pid_;

    // flags
    bool band_set_;
    bool visualization_;

    // parameters
    const double max_vel_lin_;
    const double max_vel_th_;
    const double min_vel_lin_;
    const double min_vel_th_;
    const double min_in_place_vel_th_;
    const double in_place_trans_vel_;
    const double xy_goal_tolerance_;
    const double yaw_goal_tolerance_;
    const double k_prop_;
    const double k_damp_;
    const double ctrl_rate_;
    const double max_acceleration_;
    const double virtual_mass_;
    const double max_translational_acceleration_;
    const double max_rotational_acceleration_;
    const double rotation_correction_threshold_;

    std::vector<Bubble> elastic_band_;
    geometry_msgs::Twist odom_vel_;
    geometry_msgs::Twist last_vel_;
    geometry_msgs::Pose ref_frame_band_;

    inline double sign(double n)
    {
        return n < 0.0 ? -1.0 : 1.0;
    }

    /**
     * @brief Transforms Pose of frame 1 and 2 into reference frame and gets difference of frame 1 and 2
     * @param reference to pose of frame1
     * @param reference to pose of frame2
     * @param reference to pose of reference frame
     * @return vector from frame1 to frame2 in coordinates of the reference frame
     */
    geometry_msgs::Twist getFrame1ToFrame2InRefFrame(const geometry_msgs::Pose& frame1,
                                                     const geometry_msgs::Pose& frame2,
                                                     const geometry_msgs::Pose& ref_frame);
    geometry_msgs::Twist getFrame1ToFrame2InRefFrameNew(const geometry_msgs::Pose& frame1,
                                                        const geometry_msgs::Pose& frame2,
                                                        const geometry_msgs::Pose& ref_frame);

    /**
     * @param Transforms twist into a given reference frame
     * @param Twist that shall be transformed
     * @param refernce to pose of frame1
     * @param reference to pose of frame2
     * @return transformed twist
     */
    geometry_msgs::Twist transformTwistFromFrame1ToFrame2(const geometry_msgs::Twist& curr_twist,
                                                          const geometry_msgs::Pose& frame1,
                                                          const geometry_msgs::Pose& frame2);

    /**
     * @brief limits the twist to the allowed range
     * @param reference to unconstrained twist
     * @return twist in allowed range
     */
    geometry_msgs::Twist limitTwist(const geometry_msgs::Twist& twist);

    /**
     * @brief gets the max velocity allowed within this bubble depending on size of the bubble and pose and size of the
     * following bubble
     * @param number of the bubble of interest within the band
     * @param band in which the bubble is in
     * @return absolute value of maximum allowed velocity within this bubble
     */
    double getBubbleTargetVel(const int target_bub_num, const std::vector<Bubble>& band, geometry_msgs::Twist& VelDir);
};

}  // namespace eband_local_planner

#endif  // EBAND_LOCAL_PLANNER_EBAND_TRAJECTORY_CONTROLLER_H
