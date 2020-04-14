#ifndef GRIDMAP_MAP_UPDATER_H
#define GRIDMAP_MAP_UPDATER_H

#include <gridmap/layers/base_map_layer.h>
#include <gridmap/layers/layer.h>
#include <gridmap/map_data.h>
#include <hd_map/Map.h>

#include <memory>
#include <vector>

namespace gridmap
{

class LayeredMap
{
  public:
    LayeredMap(const std::shared_ptr<BaseMapLayer>& base_map_layer, const std::vector<std::shared_ptr<Layer>>& layers);

    bool update();

    bool update(const AABB& bb);

    void clear();

    void clearRadius(const Eigen::Vector2d& pose, const double radius);

    void setMap(const hd_map::Map& hd_map, const nav_msgs::OccupancyGrid& map_data);

    std::shared_ptr<const MapData> map() const
    {
        return map_data_;
    }

  private:
    std::shared_ptr<MapData> map_data_;

    // static map layer
    std::shared_ptr<BaseMapLayer> base_map_layer_;

    // additional layers
    std::vector<std::shared_ptr<Layer>> layers_;
};
}  // namespace gridmap

#endif
