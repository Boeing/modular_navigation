#include <gridmap/grids/grid_2d.h>

namespace gridmap
{

template <class CellType>
Grid2D<CellType>::Grid2D(const MapDimensions& map_dims) : map_dimensions_(map_dims), cells_(map_dims.cells(), 0)
{
}

template <class CellType>
Grid2D<CellType>::Grid2D(const Grid2D& grid, const AABB& bb)
    : map_dimensions_(grid.dimensions().resolution(),
                      {grid.dimensions().origin().x() + bb.roi_start.x() * grid.dimensions().resolution(),
                       grid.dimensions().origin().y() + bb.roi_start.y() * grid.dimensions().resolution()},
                      bb.roi_size)
{
    ROS_ASSERT(((bb.roi_start + bb.roi_size) <= grid.dimensions().size()).all());

    cells_.resize(bb.roi_size.x() * bb.roi_size.y());

    const int src_index = grid.dimensions().size().x() * bb.roi_start.y() + bb.roi_start.x();
    const int src_index_step = grid.dimensions().size().x();
    const size_t n_bytes = bb.roi_size.x() * sizeof(CellType);

    CellType* dest = &cells_[0];
    const CellType* src = &grid.cells()[src_index];

    for (int i = 0; i < bb.roi_size.y(); ++i)
    {
        std::memcpy(dest, src, n_bytes);
        std::advance(dest, bb.roi_size.x());
        std::advance(src, src_index_step);
    }
}

template <class CellType> void Grid2D<CellType>::copyTo(Grid2D<CellType>& grid) const
{
    ROS_ASSERT((grid.dimensions().size() == map_dimensions_.size()).all());
    std::copy(cells_.begin(), cells_.end(), grid.cells().begin());
}

template <class CellType> void Grid2D<CellType>::copyTo(Grid2D<CellType>& grid, const AABB& bb) const
{
    ROS_ASSERT((grid.dimensions().size() == map_dimensions_.size()).all());

    const int index = grid.dimensions().size().x() * bb.roi_start.y() + bb.roi_start.x();
    const int index_step = grid.dimensions().size().x();
    const size_t n_bytes = bb.roi_size.x() * sizeof(CellType);

    const CellType* src = &cells_[index];
    CellType* dest = &grid.cells()[index];

    for (int i = 0; i < bb.roi_size.y(); ++i)
    {
        std::memcpy(dest, src, n_bytes);
        std::advance(dest, index_step);
        std::advance(src, index_step);
    }
}

template class Grid2D<uint8_t>;
template class Grid2D<double>;
}  // namespace gridmap
