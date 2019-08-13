#include <gridmap/layers/base_map_layer.h>
#include <gridmap/operations/rasterize.h>
#include <gridmap/params.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(gridmap::BaseMapLayer, gridmap::Layer)

namespace gridmap
{

bool BaseMapLayer::draw(OccupancyGrid& grid)
{
    std::lock_guard<std::mutex> g(map_mutex_);
    if (!map_)
        return false;
    // cppcheck-suppress unreadVariable
    const auto lock = map_->getLock();
    map_->copyTo(grid);
    return true;
}

bool BaseMapLayer::draw(OccupancyGrid& grid, const AABB& bb)
{
    std::lock_guard<std::mutex> g(map_mutex_);
    if (!map_)
        return false;
    // cppcheck-suppress unreadVariable
    const auto lock = map_->getLock();
    map_->copyTo(grid, bb);
    return true;
}

bool BaseMapLayer::update(OccupancyGrid& grid)
{
    std::lock_guard<std::mutex> g(map_mutex_);
    if (!map_)
        return false;
    // cppcheck-suppress unreadVariable
    const auto lock = map_->getLock();
    map_->merge(grid);
    return true;
}

bool BaseMapLayer::update(OccupancyGrid& grid, const AABB& bb)
{
    std::lock_guard<std::mutex> g(map_mutex_);
    if (!map_)
        return false;
    // cppcheck-suppress unreadVariable
    const auto lock = map_->getLock();
    map_->merge(grid, bb);
    return true;
}

void BaseMapLayer::onInitialize(const XmlRpc::XmlRpcValue& parameters)
{
    lethal_threshold_ =
        get_config_with_default_warn<int>(parameters, "lethal_threshold", 50, XmlRpc::XmlRpcValue::TypeInt);
}

void BaseMapLayer::onMapChanged(const nav_msgs::OccupancyGrid& new_map)
{
    map_ = std::make_shared<OccupancyGrid>(dimensions());

    //
    // Copy the occupancy grid into the costmap
    //
    const std::size_t size = dimensions().cells();
    for (std::size_t index = 0; index < size; ++index)
    {
        map_->cells()[index] =
            ((int)new_map.data[index]) >= ((int)lethal_threshold_) ? OccupancyGrid::OCCUPIED : OccupancyGrid::FREE;
    }

    //
    // Raster no-go zones
    //
    for (const hd_map::Zone& zone : hdMap().zones)
    {
        // Only consider EXCLUSION_ZONE
        if (zone.zone_type == hd_map::Zone::EXCLUSION_ZONE)
        {
            int min_x = std::numeric_limits<int>::max();
            int max_x = 0;

            int min_y = std::numeric_limits<int>::max();
            int max_y = 0;

            std::vector<Eigen::Array2i> map_polygon;
            for (const geometry_msgs::Point32& p : zone.polygon.points)
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

            for (const Eigen::Array2i& p : raster)
            {
                if (map_->dimensions().contains(p))
                    map_->setOccupied(p);
            }
        }
    }
}
}
