#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <graph_map/Zone.h>
#include <gridmap/layers/base_map_layer.h>
#include <gridmap/operations/rasterize.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(gridmap::BaseMapLayer, gridmap::Layer)

namespace gridmap
{

bool BaseMapLayer::draw(OccupancyGrid& grid) const
{
    // cppcheck-suppress unreadVariable
    const auto layer_lock = getReadLock();
    if (!map_)
        return false;
    // cppcheck-suppress unreadVariable
    const auto src_lock = map_->getReadLock();
    // cppcheck-suppress unreadVariable
    const auto dst_lock = grid.getWriteLock();
    map_->copyTo(grid);
    return true;
}

bool BaseMapLayer::draw(OccupancyGrid& grid, const AABB& bb) const
{
    // cppcheck-suppress unreadVariable
    const auto layer_lock = getReadLock();
    if (!map_)
        return false;
    // cppcheck-suppress unreadVariable
    const auto src_lock = map_->getReadLock();
    // cppcheck-suppress unreadVariable
    const auto dst_lock = grid.getWriteLock();
    map_->copyTo(grid, bb);
    return true;
}

bool BaseMapLayer::update(OccupancyGrid& grid) const
{
    // cppcheck-suppress unreadVariable
    const auto layer_lock = getReadLock();
    if (!map_)
        return false;
    // cppcheck-suppress unreadVariable
    const auto src_lock = map_->getReadLock();
    // cppcheck-suppress unreadVariable
    const auto dst_lock = grid.getWriteLock();
    grid.merge(*map_);
    return true;
}

bool BaseMapLayer::update(OccupancyGrid& grid, const AABB& bb) const
{
    // cppcheck-suppress unreadVariable
    const auto layer_lock = getReadLock();
    if (!map_)
        return false;
    // cppcheck-suppress unreadVariable
    const auto src_lock = map_->getReadLock();
    // cppcheck-suppress unreadVariable
    const auto dst_lock = grid.getWriteLock();
    grid.merge(*map_, bb);
    return true;
}

void BaseMapLayer::onInitialize(const YAML::Node& parameters)
{
    lethal_threshold_ = parameters["lethal_threshold"].as<int>(50);
}

void BaseMapLayer::onMapChanged(const nav_msgs::OccupancyGrid& new_map)
{
    map_ = std::make_shared<OccupancyGrid>(dimensions());

    uint8_t default_value = OccupancyGrid::UNKNOWN;

    //
    // Copy the occupancy grid into the costmap
    //
    const std::size_t size = dimensions().cells();
    for (std::size_t index = 0; index < size; ++index)
    {
        // OG mapping: 0 for free, 100 for obstacle and -1 for unknown
        map_->cells()[index] =
            ((int)new_map.data[index]) >= ((int)lethal_threshold_) ? OccupancyGrid::OCCUPIED : default_value;
    }

    //
    // Raster zones
    //
    for (const graph_map::Zone& zone : zones())
    {
        for (const graph_map::Region& region : zone.regions)
        {
            const geometry_msgs::Polygon polygon = region.polygon;
            int min_x = std::numeric_limits<int>::max();
            int max_x = 0;

            int min_y = std::numeric_limits<int>::max();
            int max_y = 0;

            std::vector<Eigen::Array2i> map_polygon;
            for (const geometry_msgs::Point32& p : polygon.points)
            {
                const Eigen::Array2i map_point = map_->dimensions().getCellIndex({p.x, p.y});
                min_x = std::min(map_point.x(), min_x);
                max_x = std::max(map_point.x(), max_x);
                min_y = std::min(map_point.y(), min_y);
                max_y = std::max(map_point.y(), max_y);
                map_polygon.push_back(map_point);
            }
            if (!map_polygon.empty())
                map_polygon.push_back(map_polygon.front());

            const std::vector<Eigen::Array2i> connected = connectPolygon(map_polygon);

            std::vector<Eigen::Array2i> raster;
            auto append_raster = [&raster](const int x, const int y) { raster.push_back({x, y}); };

            rasterPolygonFill(append_raster, connected, min_x, max_x, min_y, max_y);

            const auto grid_lock = map_->getWriteLock();
            for (const Eigen::Array2i& p : raster)
            {
                if (map_->dimensions().contains(p))
                {
                    const int index = map_->index(p);
                    const bool obstacle = ((int)new_map.data[index]) >= ((int)lethal_threshold_);

                    if (!obstacle)
                    {
                        if (zone.drivable)
                        {
                            map_->setFree(p);
                        }
                        else
                        {
                            map_->setOccupied(p);
                        }
                    }
                }
            }
        }
    }
}
}  // namespace gridmap
