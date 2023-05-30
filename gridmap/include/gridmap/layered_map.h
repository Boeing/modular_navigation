#ifndef GRIDMAP_MAP_UPDATER_H
#define GRIDMAP_MAP_UPDATER_H

#include <gridmap/layers/base_map_layer.h>
#include <gridmap/layers/layer.h>
#include <gridmap/map_data.h>
#include <map_manager/msg/map_info.hpp>

#include <memory>
#include <vector>

namespace gridmap {

class LayeredMap {
public:
  LayeredMap(const std::shared_ptr<BaseMapLayer> &base_map_layer,
             const std::vector<std::shared_ptr<Layer>> &layers);

  bool update();
  bool update(OccupancyGrid &grid);

  bool update(const AABB &bb);
  bool update(OccupancyGrid &grid, const AABB &bb);

  void clear();

  void clearRadius(const Eigen::Vector2d &pose, const double radius);
  void clearRadius(OccupancyGrid &grid, const Eigen::Vector2d &pose,
                   const double radius);

  void setMap(const map_manager::msg::MapInfo &map_info,
              const nav_msgs::msg::OccupancyGrid &map_data,
              const std::vector<graph_map::msg::Zone> &zones);

  std::shared_ptr<const MapData> map() const { return map_data_; }

private:
  std::shared_ptr<MapData> map_data_;

  // static map layer
  std::shared_ptr<BaseMapLayer> base_map_layer_;

  // additional layers
  std::vector<std::shared_ptr<Layer>> layers_;
};
} // namespace gridmap

#endif
