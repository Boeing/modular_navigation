#ifndef NAV_CORE_RECOVERY_BEHAVIOR_H
#define NAV_CORE_RECOVERY_BEHAVIOR_H

#include <costmap_2d/costmap_2d_ros.h>
#include <memory>

#include <tf2_ros/buffer.h>

namespace nav_core
{

class RecoveryBehavior
{
  public:
    virtual void initialize(const std::string& name, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                            const std::shared_ptr<costmap_2d::Costmap2DROS>& global_costmap,
                            const std::shared_ptr<costmap_2d::Costmap2DROS>& local_costmap) = 0;

    virtual void runBehavior() = 0;

    virtual ~RecoveryBehavior()
    {
    }

  protected:
    RecoveryBehavior()
    {
    }
};
}

#endif
