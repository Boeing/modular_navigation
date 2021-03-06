#ifndef GRIDMAP_BASE_MAP_LAYER_H
#define GRIDMAP_BASE_MAP_LAYER_H

#include <gridmap/grids/occupancy_grid.h>
#include <gridmap/layers/layer.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

namespace gridmap
{

class BaseMapLayer : public Layer
{
  public:
    BaseMapLayer() = default;
    virtual ~BaseMapLayer() = default;

    virtual bool draw(OccupancyGrid& grid) const override;
    virtual bool draw(OccupancyGrid& grid, const AABB& bb) const override;

    virtual bool update(OccupancyGrid& grid) const override;
    virtual bool update(OccupancyGrid& grid, const AABB& bb) const override;

    virtual void onInitialize(const YAML::Node& parameters) override;
    virtual void onMapChanged(const nav_msgs::OccupancyGrid& map_data) override;

    virtual bool clear() override
    {
        std::lock_guard<std::timed_mutex> g(map_mutex_);
        return bool(map_);
    }
    virtual bool clearRadius(const Eigen::Vector2i&, const int) override
    {
        std::lock_guard<std::timed_mutex> g(map_mutex_);
        return bool(map_);
    }

  private:
    int lethal_threshold_;
    std::shared_ptr<OccupancyGrid> map_;
};
}  // namespace gridmap

#endif
