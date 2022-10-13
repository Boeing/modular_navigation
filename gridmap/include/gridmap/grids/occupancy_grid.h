#ifndef GRIDMAP_OCCUPANCY_GRID_H
#define GRIDMAP_OCCUPANCY_GRID_H

#include <Eigen/Geometry>

#include <gridmap/grids/grid_2d.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <cmath>
#include <mutex>
#include <vector>

namespace gridmap
{

class OccupancyGrid : public Grid2D<uint8_t>
{
  public:
    static const uint8_t FREE;
    static const uint8_t UNKNOWN;
    static const uint8_t CONFLICT;
    static const uint8_t OCCUPIED;

    explicit OccupancyGrid(const MapDimensions& map_dims) : Grid2D<uint8_t>(map_dims)
    {
    }

    OccupancyGrid(const OccupancyGrid& grid, const AABB& bb);

    virtual ~OccupancyGrid() = default;

    inline bool occupied(const std::size_t& cell_index) const
    {
        return cells_[cell_index] == OCCUPIED;
    }

    inline bool occupied(const Eigen::Array2i& cell_index) const
    {
        return cells_[static_cast<std::size_t>(index(cell_index))] == OCCUPIED;
    }

    void setFree(const std::size_t cell_index)
    {
        cells_[cell_index] = FREE;
    }

    void setFree(const Eigen::Array2i& cell_index)
    {
        cells_[static_cast<std::size_t>(index(cell_index))] = FREE;
    }

    void setUnknown(const std::size_t cell_index)
    {
        cells_[cell_index] = UNKNOWN;
    }

    void setUnknown(const Eigen::Array2i& cell_index)
    {
        cells_[static_cast<std::size_t>(index(cell_index))] = UNKNOWN;
    }

    void setConflict(const std::size_t cell_index)
    {
        cells_[cell_index] = CONFLICT;
    }

    void setConflict(const Eigen::Array2i& cell_index)
    {
        cells_[static_cast<std::size_t>(index(cell_index))] = CONFLICT;
    }

    void setOccupied(const std::size_t cell_index)
    {
        cells_[cell_index] = OCCUPIED;
    }

    void setOccupied(const Eigen::Array2i& cell_index)
    {
        cells_[static_cast<std::size_t>(index(cell_index))] = OCCUPIED;
    }

    void merge(const OccupancyGrid& map);

    void merge(const OccupancyGrid& map, const AABB& bb);

    nav_msgs::msg::OccupancyGrid toMsg() const;

    nav_msgs::msg::OccupancyGrid toMsg(const AABB& bb) const;
};
}  // namespace gridmap

#endif
