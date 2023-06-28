#include <gridmap/layered_map.h>

#include "rcpputils/asserts.hpp"

namespace gridmap
{

LayeredMap::LayeredMap(const std::shared_ptr<BaseMapLayer>& base_map_layer,
                       const std::vector<std::shared_ptr<Layer>>& layers)
    : base_map_layer_(base_map_layer), layers_(layers)
{
}

bool LayeredMap::update()
{
    rcpputils::assert_true(map_data_ != NULL);  // This used to be ROS_ASSERT(map_data_); now just checks for null

    // copy from base map
    bool success = base_map_layer_->draw(map_data_->grid);

    // update from layers
    auto& grid = map_data_->grid;
    success &= std::all_of(layers_.begin(), layers_.end(), [&grid](const auto& layer) { return layer->update(grid); });

    return success;
}

// Create the combined map but write it to a provided grid instead of the
// internal one
bool LayeredMap::update(OccupancyGrid& grid)
{
    rcpputils::assert_true(map_data_ != NULL);  // This used to be ROS_ASSERT(map_data_); now just checks for null
    rcpputils::assert_true((map_data_->grid.dimensions().size() == grid.dimensions().size()).all());

    // copy from base map
    bool success = base_map_layer_->draw(grid);

    // update from layers
    success &= std::all_of(layers_.begin(), layers_.end(), [&grid](const auto& layer) { return layer->update(grid); });

    return success;
}

bool LayeredMap::update(const AABB& bb)
{
    rcpputils::assert_true(map_data_ != NULL);  // This used to be ROS_ASSERT(map_data_); now just checks for null
    rcpputils::assert_true(((bb.roi_start + bb.roi_size) <= map_data_->grid.dimensions().size()).all());

    // copy from base map
    bool success = base_map_layer_->draw(map_data_->grid, bb);

    // update from layers
    auto& grid = map_data_->grid;
    success &= std::all_of(layers_.begin(), layers_.end(),
                           [&grid, &bb](const auto& layer) { return layer->update(grid, bb); });

    return success;
}

bool LayeredMap::update(OccupancyGrid& grid, const AABB& bb)
{
    rcpputils::assert_true(((bb.roi_start + bb.roi_size) <= grid.dimensions().size()).all());

    // copy from base map
    bool success = base_map_layer_->draw(grid, bb);

    // update from layers
    success &= std::all_of(layers_.begin(), layers_.end(),
                           [&grid, &bb](const auto& layer) { return layer->update(grid, bb); });

    return success;
}

void LayeredMap::clear()
{
    rcpputils::assert_true(map_data_ != NULL);  // This used to be ROS_ASSERT(map_data_); now just checks for null
    for (const auto& layer : layers_)
        layer->clear();
}

void LayeredMap::clearRadius(const Eigen::Vector2d& pose, const double radius)
{
    rcpputils::assert_true(map_data_ != NULL);  // This used to be ROS_ASSERT(map_data_); now just checks for null

    const Eigen::Vector2i cell_index = map_data_->grid.dimensions().getCellIndex(pose);
    const int cell_radius = static_cast<int>(radius / map_data_->grid.dimensions().resolution());

    for (const auto& layer : layers_)
        layer->clearRadius(cell_index, cell_radius);
}

void LayeredMap::clearRadius(OccupancyGrid& grid, const Eigen::Vector2d& pose, const double radius)
{
    const Eigen::Vector2i cell_index = grid.dimensions().getCellIndex(pose);
    const int cell_radius = static_cast<int>(radius / grid.dimensions().resolution());

    for (const auto& layer : layers_)
        layer->clearRadius(cell_index, cell_radius);
}

void LayeredMap::setMap(const map_manager::msg::MapInfo& map_info, const nav_msgs::msg::OccupancyGrid& map_data,
                        const std::vector<graph_map::msg::Zone>& zones)
{
    rcpputils::assert_true(map_info.meta_data.width == map_data.info.width);
    rcpputils::assert_true(map_info.meta_data.height == map_data.info.height);
    rcpputils::assert_true(std::abs(map_info.meta_data.resolution - map_data.info.resolution) <
                           std::numeric_limits<float>::epsilon());

    base_map_layer_->setMap(map_info, map_data, zones);
    for (const auto& layer : layers_)
    {
        layer->setMap(map_info, map_data, zones);
    }
    map_data_ = std::make_shared<MapData>(map_info, base_map_layer_->dimensions(), zones);
    update();
}
}  // namespace gridmap
