#ifndef MOVE_BASE_MOVE_BASE_H
#define MOVE_BASE_MOVE_BASE_H

#include <string>
#include <vector>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_core/base_global_planner.h>
#include <nav_core/base_local_planner.h>
#include <nav_core/recovery_behavior.h>

#include <geometry_msgs/PoseStamped.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

#include <nav_msgs/GetPlan.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <condition_variable>
#include <mutex>
#include <thread>

namespace move_base
{

template <typename T> T get_param_with_default_warn(const std::string& param_name, const T& default_val)
{
    T param_val;
    if (ros::param::has(param_name))
    {
        if (ros::param::get(param_name, param_val))
        {
            return param_val;
        }
    }
    ROS_WARN_STREAM("Using default value for " << param_name << "=" << default_val);
    return default_val;
}

template <typename T> T get_param_or_throw(const std::string& param_name)
{
    T param_val;
    if (ros::param::has(param_name))
    {
        if (ros::param::get(param_name, param_val))
        {
            return param_val;
        }
    }
    throw std::runtime_error("Must specify: " + param_name);
}

enum class MoveBaseState
{
    PLANNING,
    CONTROLLING,
    RECOVERING,
    GOAL_COMPLETE,
    GOAL_FAILED
};

inline std::string toString(const MoveBaseState& state)
{
    if (state == MoveBaseState::PLANNING)
    {
        return "PLANNING";
    }
    else if (state == MoveBaseState::CONTROLLING)
    {
        return "CONTROLLING";
    }
    else if (state == MoveBaseState::RECOVERING)
    {
        return "RECOVERING";
    }
    else if (state == MoveBaseState::GOAL_COMPLETE)
    {
        return "GOAL_COMPLETE";
    }
    else if (state == MoveBaseState::GOAL_FAILED)
    {
        return "GOAL_FAILED";
    }
    return "Unknown State";
}

inline std::ostream& operator<<(std::ostream& os, const MoveBaseState& state)
{
    os << toString(state);
    return os;
}

class MoveBase
{
  public:
    MoveBase();
    virtual ~MoveBase();

  private:
    bool clearCostmapsCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool planCallback(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res);

    void publishZeroVelocity();

    bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    bool goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg, geometry_msgs::PoseStamped& global_goal);

    void wakePlanner(const ros::TimerEvent& event);
    void planThread();

    void executeCallback(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal);
    MoveBaseState executeState(const MoveBaseState state);

    bool loadRecoveryBehaviors(ros::NodeHandle node);

    ros::NodeHandle nh_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_;

    boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
    boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;

    costmap_2d::Costmap2DROS planner_costmap_ros_;
//    costmap_2d::Costmap2DROS controller_costmap_ros_;

    std::vector<boost::shared_ptr<nav_core::RecoveryBehavior>> recovery_behaviors_;
    unsigned int recovery_index_;

    ros::Publisher current_goal_pub_;
    ros::Publisher vel_pub_;

    ros::Subscriber goal_sub_;

    ros::ServiceServer clear_costmaps_service_;
    ros::ServiceServer plan_service_;

    ros::Time last_valid_plan_;
    ros::Time last_valid_control_;

    pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
    pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
    pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

    std::mutex planner_mutex_;

    // The following variables should be protected by the planner_mutex_
    std::condition_variable planner_cond_;
    geometry_msgs::PoseStamped planner_goal_;
    std::vector<geometry_msgs::PoseStamped> planner_plan_;
    bool new_global_plan_;
    MoveBaseState state_;

    std::thread planner_thread_;

    // Configuration
    const double planner_frequency_;
    const double controller_frequency_;
    const double planner_patience_;
    const double controller_patience_;
};

}  // namespace move_base

#endif  // MOVE_BASE_MOVE_BASE_H
