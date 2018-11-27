#ifndef EBAND_LOCAL_PLANNER_EBAND_LOCAL_PLANNER_ROS_H
#define EBAND_LOCAL_PLANNER_EBAND_LOCAL_PLANNER_ROS_H

#include <ros/ros.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nav_core/base_local_planner.h>

#include <eband_local_planner/conversions_and_types.h>
#include <eband_local_planner/eband_local_planner.h>
#include <eband_local_planner/eband_trajectory_controller.h>
#include <eband_local_planner/eband_visualization.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

namespace eband_local_planner
{

class EBandPlannerROS : public nav_core::BaseLocalPlanner
{
  public:
    EBandPlannerROS();
    ~EBandPlannerROS();

    void initialize(std::string name, tf2_ros::Buffer* tf_buffer, costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    bool isGoalReached();

  private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    tf2_ros::Buffer* tf_buffer_;

    double yaw_goal_tolerance_;
    double xy_goal_tolerance_;

    ros::Publisher plan_pub_;
    ros::Subscriber odom_sub_;

    nav_msgs::Odometry base_odom_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    std::vector<geometry_msgs::PoseStamped> transformed_plan_;
    std::vector<int> plan_start_end_counter_;

    std::shared_ptr<EBandPlanner> eband_;
    std::shared_ptr<EBandVisualization> eband_visual_;
    std::shared_ptr<EBandTrajectoryCtrl> eband_trj_ctrl_;

    bool goal_reached_;

    std::mutex odom_mutex_;
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

}  // namespace eband_local_planner

#endif  // EBAND_LOCAL_PLANNER_EBAND_LOCAL_PLANNER_ROS_H
