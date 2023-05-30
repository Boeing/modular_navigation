#ifndef GRIDMAP_MAP_DATA_H
#define GRIDMAP_MAP_DATA_H

#include <graph_map/msg/zone.hpp>
#include <gridmap/grids/grid_2d.h>
#include <gridmap/grids/occupancy_grid.h>
#include <map_manager/msg/map_info.hpp>

namespace gridmap {

struct MapData {
  MapData(const map_manager::msg::MapInfo &_map_info,
          const MapDimensions &map_dims,
          const std::vector<graph_map::msg::Zone> &_zones)
      : map_info(_map_info), grid(map_dims), zones(_zones) {}

  map_manager::msg::MapInfo map_info;
  OccupancyGrid grid;
  std::vector<graph_map::msg::Zone> zones;
};
} // namespace gridmap

#endif
