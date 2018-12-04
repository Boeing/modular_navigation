#ifndef _OMPL_GLOBAL_PLANNER_H
#define _OMPL_GLOBAL_PLANNER_H

#include <nav_core/base_local_planner.h>

#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <atomic>
#include <mutex>

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
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

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
    std::size_t execution_index = 1;
};

class RRTLocalPlanner : public nav_core::BaseLocalPlanner
{
  public:
    RRTLocalPlanner();
    ~RRTLocalPlanner();

    //
    // BaseLocalPlanner
    //
    virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
    virtual bool isGoalReached() override;
    virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
    virtual void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;

    bool planLocalTrajectory();

    unsigned char calc_cost(const ob::State*);
    double motion_cost(const ob::State* s1, const ob::State* s2);

    bool isStateValid(const oc::SpaceInformation* si, const ob::State* state);

    double costToDistance(const unsigned char cost) const;

    void propagate(const ob::State* start, const oc::Control* control, const double duration, ob::State* result);

  private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    tf2_ros::Buffer* tf_buffer_;

    std::atomic<bool> trajectory_complete_;
    std::mutex trajectory_mutex_;
    std::unique_ptr<LocalTrajectory> local_trajectory_;

    std::mutex global_plan_mutex_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;

    std::unique_ptr<rviz_visual_tools::RvizVisualTools> rrt_viz_;
    std::unique_ptr<rviz_visual_tools::RvizVisualTools> trajectory_viz_;

    const double costmap_weight_ = 2.0;

    // State spaces:
    ob::StateSpacePtr se2_space_;
    ob::StateSpacePtr velocity_space_;
    ob::StateSpacePtr space_;

    nav_msgs::Odometry base_odom_;
    std::mutex odom_mutex_;
    ros::Subscriber odom_sub_;
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

class CostMapObjective : public ob::StateCostIntegralObjective
{
  public:
    CostMapObjective(RRTLocalPlanner& op, const ob::SpaceInformationPtr& si)
        : ob::StateCostIntegralObjective(si, true), _ompl_planner(op)
    {
    }

    virtual ob::Cost stateCost(const ob::State* s) const
    {
        return ob::Cost(_ompl_planner.calc_cost(s));
    }


  private:
    RRTLocalPlanner& _ompl_planner;
};

class CostMapWorkObjective : public ob::MechanicalWorkOptimizationObjective
{
  public:
    CostMapWorkObjective(RRTLocalPlanner& op, const ob::SpaceInformationPtr& si)
        : ob::MechanicalWorkOptimizationObjective(si), _ompl_planner(op)
    {
    }

    ob::Cost stateCost(const ob::State* s) const
    {
        return ob::Cost(_ompl_planner.calc_cost(s));
    }

    /*
    ob::Cost motionCost(const ob::State* s1, const ob::State* s2) const
    {
        return ob::Cost(_ompl_planner.motion_cost(s1, s2));
    }
    */

  private:
    RRTLocalPlanner& _ompl_planner;
};

}

#endif
