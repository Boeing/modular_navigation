#ifndef OMNI_RRT_PLANNER_H
#define OMNI_RRT_PLANNER_H

#include <nav_core/base_global_planner.h>

#include <costmap_2d/costmap_2d.h>

#include <Eigen/Geometry>

#include <atomic>
#include <memory>
#include <mutex>

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

namespace omni_rrt_planner
{

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

unsigned char getCost(const costmap_2d::Costmap2D& costmap, const double x, const double y);
double getDistanceToCollision(const costmap_2d::Costmap2D& costmap, const double x, const double y,
                              const double inflation_weight);
double getDistanceToCollision(const unsigned char cost, const double inflation_weight);

class OmniRRTPlanner : public nav_core::BaseGlobalPlanner
{
  public:
    OmniRRTPlanner();
    virtual ~OmniRRTPlanner() override;

    //
    // BaseLocalPlanner
    //
    virtual nav_core::PlanResult makePlan(const geometry_msgs::PoseStamped& start,
                                          const geometry_msgs::PoseStamped& goal) override;
    virtual double cost(const std::vector<geometry_msgs::PoseStamped>& plan) override;

    virtual void initialize(const std::string& name, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                            const std::shared_ptr<costmap_2d::Costmap2DROS>& global_costmap,
                            const std::shared_ptr<costmap_2d::Costmap2DROS>& local_costmap) override;

    // Visualisation
    void visualisePlannerData(const ompl::base::PlannerData& pd);
    void visualisePathGeometric(const ompl::geometric::PathGeometric& path);

  private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<costmap_2d::Costmap2DROS> global_costmap_;
    std::shared_ptr<costmap_2d::Costmap2DROS> local_costmap_;

    std::unique_ptr<rviz_visual_tools::RvizVisualTools> rrt_viz_;
    std::unique_ptr<rviz_visual_tools::RvizVisualTools> trajectory_viz_;

    const double costmap_weight_ = 2.0;

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
