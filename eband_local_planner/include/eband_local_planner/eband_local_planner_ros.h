// Copyright Boeing 2017
#ifndef EBAND_LOCAL_PLANNER_EBAND_LOCAL_PLANNER_ROS_H
#define EBAND_LOCAL_PLANNER_EBAND_LOCAL_PLANNER_ROS_H

#include <ros/ros.h>

#include <string>
#include <vector>

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

// classes wich are parts of this pkg
#include <eband_local_planner/conversions_and_types.h>
#include <eband_local_planner/eband_local_planner.h>
#include <eband_local_planner/eband_trajectory_controller.h>
#include <eband_local_planner/eband_visualization.h>

// local planner specific classes which provide some macros
#include <base_local_planner/goal_functions.h>

// msgs
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// transforms
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>

// boost classes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>


namespace eband_local_planner
{

class EBandPlannerROS : public nav_core::BaseLocalPlanner
{
  public:
    EBandPlannerROS();
    EBandPlannerROS(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
    ~EBandPlannerROS();

    void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    bool isGoalReached();

  private:
    // pointer to external objects (do NOT delete object)
    costmap_2d::Costmap2DROS* costmap_ros_;
    tf::TransformListener* tf_;

    // flags
    bool initialized_;

    // parameters
    double yaw_goal_tolerance_, xy_goal_tolerance_;
    double rot_stopped_vel_, trans_stopped_vel_;

    // Topics & Services
    ros::Publisher plan_pub_;
    ros::Subscriber odom_sub_;

    // data
    nav_msgs::Odometry base_odom_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    std::vector<geometry_msgs::PoseStamped> transformed_plan_;
    std::vector<int> plan_start_end_counter_;

    // pointer to locally created objects (delete)
    boost::shared_ptr<EBandPlanner> eband_;
    boost::shared_ptr<EBandVisualization> eband_visual_;
    boost::shared_ptr<EBandTrajectoryCtrl> eband_trj_ctrl_;

    bool goal_reached_;

    boost::mutex odom_mutex_;  // mutex to lock odometry-callback while data is read from topic

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

}  // namespace eband_local_planner

#endif  // EBAND_LOCAL_PLANNER_EBAND_LOCAL_PLANNER_ROS_H
