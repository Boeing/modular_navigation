#ifndef GRIDMAP_OGMAP_H
#define GRIDMAP_OGMAP_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>
#include <vector>

#include "rcpputils/asserts.hpp"

namespace gridmap
{

class MapDimensions
{
  public:
    MapDimensions(const double resolution, const Eigen::Vector2d& origin, const Eigen::Array2i size)
        : resolution_(resolution), origin_(origin), size_(size)
    {
        rcpputils::assert_true(resolution_ > 0.0);
        rcpputils::assert_true(size.x() > 0);
        rcpputils::assert_true(size.y() > 0);
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

    boost::shared_lock<boost::shared_mutex> getReadLock() const
    {
        return boost::shared_lock<boost::shared_mutex>(shared_mutex_);
    }

    // By default, the unique_lock (write lock) has priority over shared_lock. This means while the writer is trying
    // to get a lock, new readers cannot grab the lock. This effectively means one reader can block another reader
    // if a writer is trying to get the lock.
    // To get around this, we use a non-blocking try_to_lock every 5ms and during the sleeps, new readers can lock.
    // This means readers have priority and writers have to wait for windows where there are no reader.
    boost::unique_lock<boost::shared_mutex> getWriteLock() const
    {
        boost::unique_lock<boost::shared_mutex> lock(shared_mutex_, boost::try_to_lock_t());
        while (!lock.owns_lock())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            lock.try_lock();
        }
        return lock;
    }

  protected:
    MapDimensions map_dimensions_;

    mutable boost::shared_mutex shared_mutex_;
    std::vector<CellType> cells_;
};
}  // namespace gridmap

#endif
