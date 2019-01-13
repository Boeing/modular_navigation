#ifndef CLEAR_COSTMAP_RECOVERY_H
#define CLEAR_COSTMAP_RECOVERY_H

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_layer.h>

#include <nav_core/recovery_behavior.h>
#include <ros/ros.h>

namespace clear_costmap_recovery
{

class ClearCostmapRecovery : public nav_core::RecoveryBehavior
{
  public:
    ClearCostmapRecovery();

    void initialize(const std::string& name, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                    const std::shared_ptr<costmap_2d::Costmap2DROS>& global_costmap,
                    const std::shared_ptr<costmap_2d::Costmap2DROS>& local_costmap) override;

    void runBehavior() override;

  private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<costmap_2d::Costmap2DROS> local_costmap_;
    std::shared_ptr<costmap_2d::Costmap2DROS> global_costmap_;

    void clear(costmap_2d::Costmap2DROS& costmap);

    std::set<std::string> clearable_layers_;
};
}

#endif
