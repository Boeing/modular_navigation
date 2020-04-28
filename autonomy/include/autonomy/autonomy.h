#ifndef AUTONOMY_H
#define AUTONOMY_H

#include <actionlib/server/simple_action_server.h>
#include <autonomy/DriveAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <gridmap/layered_map.h>
#include <gridmap/robot_tracker.h>
#include <gridmap/urdf_tree.h>
#include <nav_msgs/Odometry.h>
#include <navigation_interface/controller.h>
#include <navigation_interface/path_planner.h>
#include <navigation_interface/trajectory_planner.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace autonomy
{

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

struct ControlTrajectory
{
    bool goal_trajectory;
    navigation_interface::Trajectory trajectory;
};

struct TrackingPath
{
    // transformed goal in map frame
    Eigen::Isometry2d goal;

    ros::SteadyTime start_time;
    double start_cost;

    // re-calculation of cost
    ros::SteadyTime last_successful_time;
    double last_successful_cost;

    navigation_interface::Path path;
};

class Autonomy
{
  public:
    typedef actionlib::ServerGoalHandle<autonomy::DriveAction> GoalHandle;

    Autonomy();
    virtual ~Autonomy();

  private:
    void activeMapCallback(const hd_map::MapInfo::ConstPtr& map);

    void executionThread();
    void executeGoal(GoalHandle& goal);

    void goalCallback(GoalHandle goal);
    void cancelCallback(GoalHandle goal);

    void pathPlannerThread(const Eigen::Isometry2d& goal,
                           const navigation_interface::PathPlanner::GoalSampleSettings goal_sample_settings);
    void trajectoryPlannerThread();
    void controllerThread();

    ros::NodeHandle nh_;

    std::mutex goal_mutex_;
    std::unique_ptr<GoalHandle> goal_;
    std::condition_variable execution_condition_;
    std::thread execution_thread_;
    actionlib::ActionServer<autonomy::DriveAction> as_;

    pluginlib::ClassLoader<gridmap::Layer> layer_loader_;
    pluginlib::ClassLoader<navigation_interface::PathPlanner> pp_loader_;
    pluginlib::ClassLoader<navigation_interface::TrajectoryPlanner> tp_loader_;
    pluginlib::ClassLoader<navigation_interface::Controller> c_loader_;

    std::shared_ptr<navigation_interface::PathPlanner> path_planner_;
    std::shared_ptr<navigation_interface::TrajectoryPlanner> trajectory_planner_;
    std::shared_ptr<navigation_interface::Controller> controller_;

    std::shared_ptr<gridmap::LayeredMap> layered_map_;

    ros::Subscriber active_map_sub_;

    ros::Publisher costmap_publisher_;
    ros::Publisher costmap_updates_publisher_;

    ros::Publisher current_goal_pub_;
    ros::Publisher vel_pub_;

    ros::Publisher path_pub_;
    ros::Publisher trajectory_pub_;

    std::atomic<bool> running_;
    std::atomic<bool> execution_thread_running_;

    std::unique_ptr<std::thread> path_planner_thread_;
    std::unique_ptr<std::thread> trajectory_planner_thread_;
    std::unique_ptr<std::thread> controller_thread_;

    std::atomic<bool> controller_done_;

    std::unique_ptr<TrackingPath> current_path_;
    std::unique_ptr<ControlTrajectory> current_trajectory_;

    std::mutex path_mutex_;
    std::mutex trajectory_mutex_;

    // Configuration
    const std::string global_frame_ = "map";

    double map_publish_frequency_;
    double clear_radius_;
    double path_planner_frequency_;
    double trajectory_planner_frequency_;
    double controller_frequency_;
    double path_swap_fraction_;
    const double path_persistence_time_ = 6.0;

    std::shared_ptr<gridmap::URDFTree> urdf_tree_;
    std::shared_ptr<gridmap::RobotTracker> robot_tracker_;

    ros::Subscriber odom_sub_;
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    ros::Subscriber mapper_status_sub_;
    void mapperCallback(const cartographer_ros_msgs::SystemState::ConstPtr& msg);
};

}  // namespace autonomy

#endif
