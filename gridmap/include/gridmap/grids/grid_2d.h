#ifndef GRIDMAP_OGMAP_H
#define GRIDMAP_OGMAP_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/assert.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <mutex>
#include <vector>

namespace gridmap
{

class MapDimensions
{
  public:
    MapDimensions(const double resolution, const Eigen::Vector2d& origin, const Eigen::Array2i size)
        : resolution_(resolution), origin_(origin), size_(size)
    {
        ROS_ASSERT(resolution_ > 0.0);
        ROS_ASSERT(size.x() > 0);
        ROS_ASSERT(size.y() > 0);
    }

    double resolution() const
    {
        return resolution_;
    }

    const Eigen::Vector2d& origin() const
    {
        return origin_;
    }

    const Eigen::Array2i& size() const
    {
        return size_;
    }

    int cells() const
    {
        return size_.x() * size_.y();
    }

    Eigen::Array2i getCellIndex(const Eigen::Vector2d& point) const
    {
        return Eigen::Array2i(std::round((point.x() - origin_.x()) / resolution_),
                              std::round((point.y() - origin_.y()) / resolution_));
    }

    Eigen::Vector2d getCellCenter(const Eigen::Array2i cell_index) const
    {
        return {origin_.x() + resolution() * static_cast<double>(cell_index.x()),
                origin_.y() + resolution() * static_cast<double>(cell_index.y())};
    }

    bool contains(const Eigen::Array2i& cell_index) const
    {
        return (Eigen::Array2i(0, 0) <= cell_index).all() && (cell_index < size_).all();
    }

  private:
    double resolution_;
    Eigen::Vector2d origin_;
    Eigen::Array2i size_;
};

struct AABB
{
    Eigen::Array2i roi_start;
    Eigen::Array2i roi_size;
};

template <class CellType> class Grid2D
{
  public:
    explicit Grid2D(const MapDimensions& map_dims);

    Grid2D(const Grid2D& grid, const AABB& bb);

    const MapDimensions& dimensions() const
    {
        return map_dimensions_;
    }

    CellType& cell(const std::size_t cell_index)
    {
        return cells_[cell_index];
    }

    CellType& cell(const Eigen::Array2i& cell_index)
    {
        return cells_[static_cast<std::size_t>(index(cell_index))];
    }

    CellType cell(const std::size_t cell_index) const
    {
        return cells_[cell_index];
    }

    CellType cell(const Eigen::Array2i& cell_index) const
    {
        return cells_[static_cast<std::size_t>(index(cell_index))];
    }

    std::vector<CellType>& cells()
    {
        return cells_;
    }

    const std::vector<CellType>& cells() const
    {
        return cells_;
    }

    void copyTo(Grid2D<CellType>& grid) const;

    void copyTo(Grid2D<CellType>& grid, const AABB& bb) const;

    inline int index(const Eigen::Array2i& cell_index) const
    {
        return map_dimensions_.size().x() * cell_index.y() + cell_index.x();
    }

    std::unique_lock<std::recursive_mutex> getLock() const
    {
        return std::unique_lock<std::recursive_mutex>(mutex_);
    }

  protected:
    MapDimensions map_dimensions_;

    mutable std::recursive_mutex mutex_;
    std::vector<CellType> cells_;
};
}  // namespace gridmap

#endif
