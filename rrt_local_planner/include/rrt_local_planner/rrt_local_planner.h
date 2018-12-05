#ifndef RRT_LOCAL_PLANNER_H
#define RRT_LOCAL_PLANNER_H

#include <nav_core/base_local_planner.h>

#include <costmap_2d/costmap_2d.h>

#include <Eigen/Geometry>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <atomic>
#include <mutex>
#include <thread>
#include <memory>

#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

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
    std::size_t execution_index;
    std::atomic<bool> execution_complete;
};

struct TrajectoryPlanResult
{
    bool success;
    std::shared_ptr<ompl::base::PlannerData> pd;  // manages State memory
    std::shared_ptr<ompl::geometric::PathGeometric> trajectory;  // references State memory in PlannerData
};

struct GlobalPath
{
    std::vector<geometry_msgs::PoseStamped> path;
    std::size_t execution_index;
};

class RRTLocalPlanner : public nav_core::BaseLocalPlanner
{
  public:
    RRTLocalPlanner();
    virtual ~RRTLocalPlanner() override;

    //
    // BaseLocalPlanner
    //
    virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
    virtual bool isGoalReached() override;
    virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
    virtual void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;

    TrajectoryPlanResult planLocalTrajectory(const Eigen::Isometry2d& start, const Eigen::Isometry2d& goal, const double threshold);

    // OMPL functions
    unsigned char stateCost(const ompl::base::State*);
    bool isStateValid(const ompl::control::SpaceInformation* si, const ompl::base::State* state);
    double costToDistance(const unsigned char cost) const;
    void propagate(const ompl::base::State* start, const ompl::control::Control* control, const double duration, ompl::base::State* result);

    // Visualisation
    void visualisePlannerData(const ompl::base::PlannerData& pd);
    void visualisePathGeometric(const ompl::geometric::PathGeometric& path);

    void updateTrajectory(const TrajectoryPlanResult& result);
    double remainingTrajectoryCost();

    void trajectoryPlanningThread();

  private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    tf2_ros::Buffer* tf_buffer_;

    // This is the execution of the current local trajectory in "odom" frame
    std::mutex control_mutex_;
    std::unique_ptr<ControlData> control_data_;

    // This is the current local trajectory in "map" frame
    std::mutex trajectory_mutex_;
    std::unique_ptr<TrajectoryPlanResult> trajectory_result_;

    // This is the requested global path in "map" frame
    std::mutex global_mutex_;
    std::unique_ptr<GlobalPath> global_path_;

    std::unique_ptr<rviz_visual_tools::RvizVisualTools> rrt_viz_;
    std::unique_ptr<rviz_visual_tools::RvizVisualTools> trajectory_viz_;

    const double costmap_weight_ = 2.0;

    std::thread planning_thread_;

    // OMPL data structures
    ompl::base::StateSpacePtr se2_space_;
    ompl::base::StateSpacePtr velocity_space_;
    ompl::base::StateSpacePtr space_;
    ompl::control::ControlSpacePtr cspace_;
    ompl::control::SpaceInformationPtr si_;
    ompl::base::OptimizationObjectivePtr objective_;

    nav_msgs::Odometry base_odom_;
    std::mutex odom_mutex_;
    ros::Subscriber odom_sub_;
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

class CostMapObjective : public ompl::base::StateCostIntegralObjective
{
  public:
    CostMapObjective(RRTLocalPlanner& op, const ompl::base::SpaceInformationPtr& si)
        : ompl::base::StateCostIntegralObjective(si, true), _ompl_planner(op)
    {
    }

    virtual ompl::base::Cost stateCost(const ompl::base::State* s) const
    {
        return ompl::base::Cost(static_cast<double>(_ompl_planner.stateCost(s)));
    }

  private:
    RRTLocalPlanner& _ompl_planner;
};

}

#endif
