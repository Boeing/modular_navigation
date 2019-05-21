#ifndef GRIDMAP_MAP_DATA_H
#define GRIDMAP_MAP_DATA_H

#include <gridmap/grids/grid_2d.h>
#include <gridmap/grids/occupancy_grid.h>

#include <hd_map/Map.h>

namespace gridmap
{

struct MapData
{
    MapData(const hd_map::Map& _hd_map, const MapDimensions& map_dims)
        : hd_map(_hd_map), grid(map_dims)
    {
    }

    hd_map::Map hd_map;
    OccupancyGrid grid;
};

}

#endif
