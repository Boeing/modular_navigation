#ifndef GRIDMAP_MAP_DATA_H
#define GRIDMAP_MAP_DATA_H

#include <graph_map/Zone.h>
#include <gridmap/grids/grid_2d.h>
#include <gridmap/grids/occupancy_grid.h>
#include <map_manager/MapInfo.h>

namespace gridmap
{

struct MapData
{
    MapData(const map_manager::MapInfo& _map_info, const MapDimensions& map_dims,
            const std::vector<graph_map::Zone>& _zones)
        : map_info(_map_info), grid(map_dims), zones(_zones)
    {
    }

    map_manager::MapInfo map_info;
    OccupancyGrid grid;
    std::vector<graph_map::Zone> zones;
};
}  // namespace gridmap

#endif
