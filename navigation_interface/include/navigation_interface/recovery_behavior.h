#ifndef NAVIGATION_INTERFACE_BASE_LOCAL_PLANNER_H_RECOVERY_BEHAVIOR_H
#define NAVIGATION_INTERFACE_BASE_LOCAL_PLANNER_H_RECOVERY_BEHAVIOR_H

#include <costmap_2d/costmap_2d.h>

#include <xmlrpcpp/XmlRpc.h>

#include <memory>

namespace navigation_interface
{

class RecoveryBehavior
{
  public:
    virtual void initialize(const std::shared_ptr<const costmap_2d::Costmap2D>& costmap) = 0;

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
