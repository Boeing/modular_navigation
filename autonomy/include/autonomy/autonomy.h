#ifndef AUTONOMY_H
#define AUTONOMY_H

//#include <actionlib/server/simple_action_server.h>
#include "rclcpp_action/rclcpp_action.hpp"
//#include <autonomy/DriveAction.h>
#include <autonomy_interface/action/drive.hpp>
#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/twist.hpp>
#include <gridmap/layered_map.h>
#include <gridmap/robot_tracker.h>
#include <gridmap/urdf_tree.h>
#include <map_manager/srv/get_map_info.hpp>
#include <map_manager/srv/get_occupancy_grid.hpp>
#include <map_manager/srv/get_zones.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <nav_msgs/msg/odometry.h>
#include <nav_msgs/msg/path.hpp>
#include <navigation_interface/controller.h>
#include <navigation_interface/path_planner.h>
#include <navigation_interface/trajectory_planner.h>
#include <pluginlib/class_loader.hpp>
#include <std_msgs/msg/float64.h>

//#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcpputils/asserts.hpp"
#include "std_msgs/msg/float64.hpp"

namespace autonomy
{

struct ControlTrajectory
{
    bool goal_trajectory;
    navigation_interface::Trajectory trajectory;
};

struct TrackingPath
{
    // transformed goal in map frame
    Eigen::Isometry2d goal;

    rclcpp::Time start_time;
    double start_cost;

    // re-calculation of cost
    rclcpp::Time last_successful_time;
    double last_successful_cost;

    navigation_interface::Path path;
};

class Autonomy : public rclcpp::Node
{
  public:
    using Drive = autonomy_interface::action::Drive;
    using GoalHandleDrive = rclcpp_action::ServerGoalHandle<Drive>;

    Autonomy(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~Autonomy();

    void init();  // node must be initialized out-of-constructor to enable access to this->shared_from_this()

    rclcpp::Node::SharedPtr service_node_;
    rclcpp::Node::SharedPtr param_client_node;

  private:
    void activeMapCallback(const map_manager::msg::MapInfo& map);
    void updateCostmapCallback();
    void executeGoal();

    void waitForExecution(bool preempt = false);

    rclcpp_action::GoalResponse goalCallback(const rclcpp_action::GoalUUID& uuid,
                                             std::shared_ptr<const Drive::Goal> goal);
    rclcpp_action::CancelResponse cancelCallback(const std::shared_ptr<GoalHandleDrive> goal_handle);
    void acceptedCallback(const std::shared_ptr<GoalHandleDrive> goal_handle);

    void pathPlannerThread();
    void trajectoryPlannerThread();
    void controllerThread();

    std::shared_ptr<GoalHandleDrive> goalHandle()
    {
        std::lock_guard<std::mutex> lock(goal_handle_mutex_);
        return goal_handle_ptr_;
    }
    void setGoalHandle(std::shared_ptr<GoalHandleDrive> goal_handle)
    {
        std::lock_guard<std::mutex> lock(goal_handle_mutex_);
        goal_handle_ptr_ = goal_handle;
    }

    mutable std::mutex goal_handle_mutex_;
    std::shared_ptr<GoalHandleDrive> goal_handle_ptr_;

    rclcpp::TimerBase::SharedPtr costmap_timer_;

    mutable std::condition_variable map_updated_conditional_;
    mutable std::mutex map_updated_mutex_;
    bool map_updated_;

    geometry_msgs::msg::PoseStamped transformed_goal_pose_;
    rclcpp_action::Server<autonomy_interface::action::Drive>::SharedPtr action_server_;

//    std::thread execution_thread_;
//    std::thread goal_exec_thread_;
    std::shared_future<void> execution_future_;

    pluginlib::ClassLoader<gridmap::Layer> layer_loader_;
    pluginlib::ClassLoader<navigation_interface::PathPlanner> pp_loader_;
    pluginlib::ClassLoader<navigation_interface::TrajectoryPlanner> tp_loader_;
    pluginlib::ClassLoader<navigation_interface::Controller> c_loader_;

    std::shared_ptr<gridmap::MapData> path_planner_map_data_;
    std::shared_ptr<gridmap::MapData> trajectory_planner_map_data_;
    std::shared_ptr<gridmap::MapData> controller_map_data_;

    std::shared_ptr<navigation_interface::PathPlanner> path_planner_;
    std::shared_ptr<navigation_interface::TrajectoryPlanner> trajectory_planner_;
    std::shared_ptr<navigation_interface::Controller> controller_;

    std::shared_ptr<gridmap::LayeredMap> layered_map_;

    rclcpp::Subscription<map_manager::msg::MapInfo>::SharedPtr active_map_sub_;

    rclcpp::Client<map_manager::srv::GetMapInfo>::SharedPtr get_map_info_client_;
    rclcpp::Client<map_manager::srv::GetOccupancyGrid>::SharedPtr get_occupancy_grid_client_;
    rclcpp::Client<map_manager::srv::GetZones>::SharedPtr get_zones_client_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_publisher_;

    rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr costmap_updates_publisher_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr path_goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr planner_map_update_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr trajectory_map_update_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_map_update_pub_;

    // callback groups
    rclcpp::CallbackGroup::SharedPtr callback_group_action_srv_;
    rclcpp::CallbackGroup::SharedPtr umbrella_callback_group_;
    rclcpp::CallbackGroup::SharedPtr srv_callback_group_;
    rclcpp::CallbackGroup::SharedPtr costmap_timer_callback_group_;

    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

    std::atomic<bool> running_;
    std::atomic<bool> execution_thread_running_;

    std::unique_ptr<std::thread> path_planner_thread_;
    std::unique_ptr<std::thread> trajectory_planner_thread_;
    std::unique_ptr<std::thread> controller_thread_;

    std::atomic<bool> controller_done_;

    std::unique_ptr<TrackingPath> current_path_;
    std::unique_ptr<ControlTrajectory> current_trajectory_;

    mutable std::mutex path_mutex_;
    mutable std::mutex trajectory_mutex_;

    // Configuration
    const std::string global_frame_ = "map";

    double max_planning_distance_;
    double clear_radius_;
    double path_planner_frequency_;
    double trajectory_planner_frequency_;
    double controller_frequency_;
    double path_swap_fraction_;
    const double path_persistence_time_ = 6.0;
    // Delay for the spin_until_future_complete for async srv calls
    const double maximum_map_server_delay = 1.0;

    std::shared_ptr<gridmap::URDFTree> urdf_tree_;
    std::shared_ptr<gridmap::RobotTracker> robot_tracker_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    void odomCallback(const nav_msgs::msg::Odometry& msg) const;

    rclcpp::Subscription<cartographer_ros_msgs::msg::SystemState>::SharedPtr mapper_status_sub_;
    void mapperCallback(const cartographer_ros_msgs::msg::SystemState& msg) const;
};

}  // namespace autonomy

#endif
