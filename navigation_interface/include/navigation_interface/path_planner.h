#ifndef NAVIGATION_INTERFACE_PATH_PLANNER_H
#define NAVIGATION_INTERFACE_PATH_PLANNER_H

#include <navigation_interface/types/path.h>

#include <gridmap/map_data.h>

#include <xmlrpcpp/XmlRpc.h>

#include <memory>

namespace navigation_interface
{

class PathPlanner
{
  public:
    enum class Outcome
    {
        FAILED,
        SUCCESSFUL,
        PARTIAL
    };

    struct Result
    {
        Outcome outcome;
        double cost;
        Path path;
    };

    PathPlanner() = default;
    virtual ~PathPlanner() = default;

    virtual Result plan(const Eigen::Isometry2d& start, const Eigen::Isometry2d& goal) = 0;

    virtual bool valid(const Path& path) const = 0;
    virtual double cost(const Path& path) const = 0;

    virtual void onInitialize(const XmlRpc::XmlRpcValue& parameters) = 0;
    virtual void onMapDataChanged() = 0;

    void initialize(const XmlRpc::XmlRpcValue& parameters, const std::shared_ptr<const gridmap::MapData>& map_data)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        map_data_ = map_data;
        onInitialize(parameters);
    }

    void setMapData(const std::shared_ptr<const gridmap::MapData>& map_data)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        map_data_ = map_data;
        onMapDataChanged();
    }

  protected:
    std::mutex mutex_;
    std::shared_ptr<const gridmap::MapData> map_data_;
};
}  // namespace navigation_interface

#endif
