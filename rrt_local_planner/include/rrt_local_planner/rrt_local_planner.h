#ifndef RRT_LOCAL_PLANNER_H
#define RRT_LOCAL_PLANNER_H

#include <nav_core/base_local_planner.h>

#include <rrt_local_planner/pid.h>

#include <costmap_2d/costmap_2d.h>

#include <Eigen/Geometry>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <ros/ros.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

namespace rrt_local_planner
{

struct KinodynamicState
{
    geometry_msgs::Pose pose;
    geometry_msgs::Twist velocity;
};

struct LocalTrajectory
{
    std::vector<KinodynamicState> states;
};

struct ControlData
{
    LocalTrajectory trajectory;
    std::size_t execution_index = 1;
    std::atomic<bool> execution_complete;

    ros::SteadyTime last_time_step = ros::SteadyTime::now();
};

struct TrajectoryPlanResult
{
    bool success;

    double cost;
    double length;

    ompl::base::PlannerPtr planner;
    std::shared_ptr<ompl::base::PlannerData> pd;
    std::shared_ptr<ompl::geometric::PathGeometric> trajectory;

    geometry_msgs::PoseStamped global_waypoint;
    std::size_t global_index;

    Eigen::Isometry2d start;
    Eigen::Isometry2d goal;
    Eigen::Isometry3d global_to_local;
};

struct PlannerState
{
    bool path_exists;
    bool path_waypoint_changed;
    bool path_last_waypoint;
    bool global_to_local_changed;

    geometry_msgs::PoseStamped global_waypoint;
    std::size_t global_index;

    Eigen::Isometry3d goal;        // in local_frame
    Eigen::Isometry3d robot_pose;  // in local_frame
    Eigen::Isometry3d global_to_local;

    Eigen::Isometry2d stard_2d;
    Eigen::Isometry2d goal_2d;
};

struct GlobalPath
{
    std::vector<geometry_msgs::PoseStamped> path;
    std::size_t execution_index;
};

unsigned char getCost(const costmap_2d::Costmap2D& costmap, const double x, const double y);
double getDistanceToCollision(const costmap_2d::Costmap2D& costmap, const double x, const double y,
                              const double inflation_weight);
double getDistanceToCollision(const unsigned char cost, const double inflation_weight);

class RRTLocalPlanner : public nav_core::BaseLocalPlanner
{
  public:
    RRTLocalPlanner();
    virtual ~RRTLocalPlanner() override;

    //
    // BaseLocalPlanner
    //
    virtual nav_core::Control computeControl(const ros::SteadyTime& steady_time, const ros::Time& ros_time,
                                             const nav_msgs::Odometry& odom) override;
    virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
    virtual bool clearPlan() override;
    virtual void initialize(const std::string& name, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                            const std::shared_ptr<costmap_2d::Costmap2DROS>& local_costmap) override;

    TrajectoryPlanResult planLocalTrajectory(const Eigen::Isometry2d& start, const Eigen::Isometry2d& goal,
                                             const double threshold);

    // Visualisation
    void visualisePlannerData(const ompl::base::PlannerData& pd);
    void visualisePathGeometric(const ompl::geometric::PathGeometric& path);

    void updateTrajectory(const TrajectoryPlanResult& result);

    double getRemainingTrajectoryCost();

    PlannerState getPlannerState();
    void trajectoryPlanningThread();

  private:
    tf2_ros::Buffer* tf_buffer_;
    costmap_2d::Costmap2DROS* costmap_ros_;

    // This is the execution of the current local trajectory in "odom" frame
    std::mutex control_mutex_;
    std::unique_ptr<ControlData> control_data_;
    rrt_local_planner::PID goal_x_pid_ = rrt_local_planner::PID(1, 0.5, 0.0, -0, 0, true);
    rrt_local_planner::PID goal_y_pid_ = rrt_local_planner::PID(1, 0.5, 0.0, -0, 0, true);
    rrt_local_planner::PID tracking_x_pid_ = rrt_local_planner::PID(4, 2.0, 0.0, -0, 0, true);
    rrt_local_planner::PID tracking_y_pid_ = rrt_local_planner::PID(4, 2.0, 0.0, -0, 0, true);
    rrt_local_planner::PID rotation_pid_ = rrt_local_planner::PID(1, 0.1, 0.0, -0, 0, true);

    // This is the current local trajectory in "odom" frame
    std::mutex trajectory_mutex_;
    std::unique_ptr<TrajectoryPlanResult> trajectory_result_;

    // This is the requested global path in "map" frame
    std::mutex global_mutex_;
    std::unique_ptr<GlobalPath> global_path_;

    std::unique_ptr<rviz_visual_tools::RvizVisualTools> rrt_viz_;
    std::unique_ptr<rviz_visual_tools::RvizVisualTools> trajectory_viz_;

    const double control_p_gain_ = 10.0;
    const double control_d_gain_ = 2.0;

    const double goal_linear_threshold = 0.005;
    const double goal_angular_threshold = 0.0175;

    const double max_velocity_x_ = 0.15;
    const double max_acceleration_x_ = 0.01;

    const double max_velocity_y_ = 0.10;
    const double max_acceleration_y_ = 0.01;

    const double max_velocity_w_ = 0.5;
    const double max_acceleration_w_ = 0.1;

    const double goal_radius_ = 0.04;

    const double max_linear_tracking_error_ = 0.10;
    const double max_angular_tracking_error_ = 0.6;

    const double costmap_weight_ = 2.0;

    std::thread planning_thread_;

    // OMPL data structures
    ompl::base::StateSpacePtr se2_space_;
    ompl::base::StateSpacePtr velocity_space_;
    ompl::base::StateSpacePtr space_;
    ompl::control::ControlSpacePtr cspace_;
    ompl::base::SpaceInformationPtr si_;
    ompl::base::OptimizationObjectivePtr objective_;
};

class ValidityChecker : public ompl::base::StateValidityChecker
{
  public:
    ValidityChecker(const ompl::base::SpaceInformationPtr& si, const costmap_2d::Costmap2D* costmap,
                    const double inflation_weight)
        : ompl::base::StateValidityChecker(si), costmap_(costmap), inflation_weight_(inflation_weight)
    {
    }

    bool isValid(const ompl::base::State* state) const override
    {
        return clearance(state) > 0.0;
    }

    double clearance(const ompl::base::State* state) const override
    {
        return getDistanceToCollision(cost(state), inflation_weight_);
    }

    unsigned char cost(const ompl::base::State* state) const
    {
        const auto* se2state = state->as<ompl::base::SE2StateSpace::StateType>();
        const double x = se2state->getX();
        const double y = se2state->getY();
        return getCost(*costmap_, x, y);
    }

  private:
    const costmap_2d::Costmap2D* costmap_;
    const double inflation_weight_;
};

class CostMapObjective : public ompl::base::StateCostIntegralObjective
{
  public:
    CostMapObjective(const ompl::base::SpaceInformationPtr& si, const costmap_2d::Costmap2D* costmap)
        : ompl::base::StateCostIntegralObjective(si, true), costmap_(costmap)
    {
    }

    virtual ompl::base::Cost stateCost(const ompl::base::State* s) const
    {
        return ompl::base::Cost(static_cast<double>(cost(s)));
    }

    unsigned char cost(const ompl::base::State* state) const
    {
        const auto* se2state = state->as<ompl::base::SE2StateSpace::StateType>();
        const double x = se2state->getX();
        const double y = se2state->getY();
        return getCost(*costmap_, x, y);
    }

  private:
    const costmap_2d::Costmap2D* costmap_;
};
}

#endif
