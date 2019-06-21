#ifndef NAVIGATION_INTERFACE_RECOVERY_BEHAVIOR_H
#define NAVIGATION_INTERFACE_RECOVERY_BEHAVIOR_H

#include <gridmap/map_data.h>

#include <xmlrpcpp/XmlRpc.h>

#include <memory>

namespace navigation_interface
{

class RecoveryBehavior
{
  public:
    virtual void initialize(const std::shared_ptr<const gridmap::MapData>& map_data) = 0;

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
