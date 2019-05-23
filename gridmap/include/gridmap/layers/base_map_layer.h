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

    virtual void draw(OccupancyGrid& grid) override;
    virtual void draw(OccupancyGrid& grid, const AABB& bb) override;

    virtual void update(OccupancyGrid& grid) override;
    virtual void update(OccupancyGrid& grid, const AABB& b) override;

    virtual void onInitialize(const XmlRpc::XmlRpcValue& parameters) override;
    virtual void onMapChanged(const nav_msgs::OccupancyGrid& map_data) override;

    virtual void clearRadius(const Eigen::Vector2i&, const int) override {}

  private:
    int lethal_threshold_;
    std::shared_ptr<OccupancyGrid> map_;
};
}

#endif
