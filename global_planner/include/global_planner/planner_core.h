#ifndef GLOBAL_PLANNER_PLANNERCORE_H
#define GLOBAL_PLANNER_PLANNERCORE_H

#include <costmap_2d/costmap_2d.h>

#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include <global_planner/GlobalPlannerConfig.h>

#include <global_planner/expander.h>
#include <global_planner/orientation_filter.h>
#include <global_planner/potential_calculator.h>
#include <global_planner/traceback.h>

#include <nav_core/base_global_planner.h>

#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>

#include <ros/ros.h>

#include <mutex>
#include <vector>

namespace global_planner
{

class GlobalPlanner : public nav_core::BaseGlobalPlanner
{
  public:
    GlobalPlanner();
    ~GlobalPlanner();

    void initialize(std::string name,
                            std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                            std::shared_ptr<costmap_2d::Costmap2DROS> global_costmap,
                            std::shared_ptr<costmap_2d::Costmap2DROS> local_costmap) override;

    nav_core::PlanResult makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal) override;

  private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<costmap_2d::Costmap2DROS> global_costmap_;
    std::shared_ptr<costmap_2d::Costmap2DROS> local_costmap_;

    ros::Publisher plan_pub_;
    ros::Publisher potential_pub_;

    bool allow_unknown_;
    bool visualize_potential_;
    bool publish_potential_;
    int publish_scale_;

    std::mutex mutex_;

    std::shared_ptr<PotentialCalculator> p_calc_;
    std::shared_ptr<Expander> planner_;
    std::shared_ptr<Traceback> path_maker_;
    std::shared_ptr<OrientationFilter> orientation_filter_;

    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
    void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);

    std::unique_ptr<dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>> dsrv_;
    void reconfigureCB(global_planner::GlobalPlannerConfig& config, uint32_t level);
};

}  // end namespace global_planner

#endif
