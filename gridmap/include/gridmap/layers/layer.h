#ifndef GRIDMAP_LAYER_H
#define GRIDMAP_LAYER_H

#include <gridmap/grids/grid_2d.h>
#include <gridmap/grids/occupancy_grid.h>
#include <gridmap/robot_tracker.h>
#include <gridmap/urdf_tree.h>
#include <hd_map/Map.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>

#include <memory>

namespace gridmap
{

class Layer
{
  public:
    Layer(){};
    virtual ~Layer(){};

    virtual bool draw(OccupancyGrid& grid) const = 0;
    virtual bool draw(OccupancyGrid& grid, const AABB& bb) const = 0;

    virtual bool update(OccupancyGrid& grid) const = 0;
    virtual bool update(OccupancyGrid& grid, const AABB& bb) const = 0;

    virtual void onInitialize(const YAML::Node& parameters) = 0;
    virtual void onMapChanged(const nav_msgs::OccupancyGrid& map_data) = 0;

    virtual bool clear() = 0;
    virtual bool clearRadius(const Eigen::Vector2i& cell_index, const int cell_radius) = 0;

    void setMap(const hd_map::Map& hd_map, const nav_msgs::OccupancyGrid& map_data)
    {
        ROS_INFO_STREAM("Updating map: " << name());
        std::lock_guard<std::timed_mutex> lock(map_mutex_);
        hd_map_ = std::make_shared<hd_map::Map>(hd_map);
        map_dimensions_.reset(
            new MapDimensions(hd_map.info.meta_data.resolution,
                              {hd_map.info.meta_data.origin.position.x, hd_map.info.meta_data.origin.position.y},
                              {hd_map.info.meta_data.width, hd_map.info.meta_data.height}));
        onMapChanged(map_data);
        ROS_INFO_STREAM("Updating map: " << name() << " DONE");
    }

    void initialize(const std::string& name, const YAML::Node& parameters,
                    const std::vector<Eigen::Vector2d>& robot_footprint,
                    const std::shared_ptr<RobotTracker>& robot_tracker, const std::shared_ptr<URDFTree>& urdf_tree)
    {
        name_ = name;
        robot_tracker_ = robot_tracker;
        urdf_tree_ = urdf_tree;
        robot_footprint_ = robot_footprint;
        onInitialize(parameters);
    }

    const hd_map::Map& hdMap() const
    {
        return *hd_map_;
    }

    const MapDimensions& dimensions() const
    {
        return *map_dimensions_;
    }

    const std::string& name() const
    {
        return name_;
    }

    const std::string& globalFrame() const
    {
        return global_frame_;
    }

  protected:
    mutable std::timed_mutex map_mutex_;

    std::shared_ptr<RobotTracker> robot_tracker_;
    std::shared_ptr<URDFTree> urdf_tree_;
    std::vector<Eigen::Vector2d> robot_footprint_;

  private:
    std::shared_ptr<hd_map::Map> hd_map_;
    std::shared_ptr<MapDimensions> map_dimensions_;
    std::string name_;

    std::string global_frame_ = "map";
};
}  // namespace gridmap

#endif
