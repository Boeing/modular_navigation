#include <gridmap/grids/occupancy_grid.h>

namespace gridmap
{

const uint8_t OccupancyGrid::FREE = 0;
const uint8_t OccupancyGrid::UNKNOWN = 1;
const uint8_t OccupancyGrid::CONFLICT = 2;
const uint8_t OccupancyGrid::OCCUPIED = 255;

void OccupancyGrid::merge(const OccupancyGrid& map)
{
    const int size = dimensions().cells();
    for (int index = 0; index < size; ++index)
    {
        if (map.cells()[index] == OCCUPIED)
            cells_[index] = OCCUPIED;
    }
}

void OccupancyGrid::merge(const OccupancyGrid& map, const AABB& bb)
{
    const int y_size = bb.roi_start.y() + bb.roi_size.y();
    for (int y = bb.roi_start.y(); y < y_size; y++)
    {
        const int index_start = map_dimensions_.size().x() * y + bb.roi_start.x();
        const int index_end = index_start + bb.roi_size.x();
        for (int index = index_start; index < index_end; ++index)
        {
            if (map.cells()[index] == OCCUPIED)
                cells_[index] = OCCUPIED;
        }
    }
}

// cppcheck-suppress unusedFunction
nav_msgs::OccupancyGrid OccupancyGrid::toMsg() const
{
    nav_msgs::OccupancyGrid grid;
    grid.info.resolution = map_dimensions_.resolution();
    grid.info.width = map_dimensions_.size().x();
    grid.info.height = map_dimensions_.size().y();
    grid.info.origin.position.x = map_dimensions_.origin().x();
    grid.info.origin.position.y = map_dimensions_.origin().y();
    grid.info.origin.orientation.w = 1.0;
    const int size = map_dimensions_.cells();
    grid.data.resize(size);
    for (int i = 0; i < size; ++i)
        grid.data[i] = (cells_[i] == OCCUPIED) ? 100 : 0;
    return grid;
}

nav_msgs::OccupancyGrid OccupancyGrid::toMsg(const AABB& bb) const
{
    nav_msgs::OccupancyGrid grid;
    grid.info.resolution = map_dimensions_.resolution();
    grid.info.width = bb.roi_size.x();
    grid.info.height = bb.roi_size.y();
    grid.info.origin.position.x = map_dimensions_.origin().x() + bb.roi_start.x() * map_dimensions_.resolution();
    grid.info.origin.position.y = map_dimensions_.origin().y() + bb.roi_start.y() * map_dimensions_.resolution();
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(bb.roi_size.x() * bb.roi_size.y());
    int roi_index = 0;
    const int y_size = bb.roi_start.y() + bb.roi_size.y();
    for (int y = bb.roi_start.y(); y < y_size; y++)
    {
        const int index_start = map_dimensions_.size().x() * y + bb.roi_start.x();
        const int index_end = index_start + bb.roi_size.x();
        for (int index = index_start; index < index_end; ++index)
        {
            grid.data[roi_index] = (cells_[index] == OCCUPIED) ? 100 : 0;
            ++roi_index;
        }
    }
    return grid;
}
}  // namespace gridmap
