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
    virtual ~EBandPlannerROS() override;

    virtual nav_core::Control computeControl(const ros::SteadyTime& steady_time, const ros::Time& ros_time,
                                             const nav_msgs::Odometry& odom) override;

    virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
    virtual bool clearPlan() override;

    virtual void initialize(const std::string& name, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                            const std::shared_ptr<costmap_2d::Costmap2DROS>& local_costmap) override;

  private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<costmap_2d::Costmap2DROS> local_costmap_;

    ros::Publisher plan_pub_;

    std::vector<geometry_msgs::PoseStamped> global_plan_;
    std::vector<geometry_msgs::PoseStamped> transformed_plan_;
    std::vector<int> plan_start_end_counter_;

    std::shared_ptr<EBandPlanner> eband_;
    std::shared_ptr<EBandVisualization> eband_visual_;
    std::shared_ptr<EBandTrajectoryCtrl> eband_trj_ctrl_;

    bool goal_reached_;
};

}

#endif
