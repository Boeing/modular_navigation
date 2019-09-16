#ifndef GRIDMAP_LAYER_H
#define GRIDMAP_LAYER_H

#include <gridmap/grids/grid_2d.h>
#include <gridmap/grids/occupancy_grid.h>

#include <tf2_ros/buffer.h>

#include <hd_map/Map.h>
#include <nav_msgs/OccupancyGrid.h>

#include <memory>

namespace gridmap
{

class Layer
{
  public:
    Layer() : hd_map_(nullptr), map_dimensions_(nullptr), tf_buffer_(nullptr){};
    virtual ~Layer(){};

    virtual bool draw(OccupancyGrid& grid) = 0;
    virtual bool draw(OccupancyGrid& grid, const AABB& bb) = 0;

    virtual bool update(OccupancyGrid& grid) = 0;
    virtual bool update(OccupancyGrid& grid, const AABB& bb) = 0;

    virtual void onInitialize(const XmlRpc::XmlRpcValue& parameters) = 0;
    virtual void onMapChanged(const nav_msgs::OccupancyGrid& map_data) = 0;

    virtual bool clear() = 0;
    virtual bool clearRadius(const Eigen::Vector2i& cell_index, const int cell_radius) = 0;

    void setMap(const hd_map::Map& hd_map, const nav_msgs::OccupancyGrid& map_data)
    {
        std::lock_guard<std::timed_mutex> lock(map_mutex_);
        ROS_INFO_STREAM("Updating map for: " << name_);
        hd_map_ = std::make_shared<hd_map::Map>(hd_map);
        map_dimensions_.reset(
            new MapDimensions(hd_map.info.meta_data.resolution,
                              {hd_map.info.meta_data.origin.position.x, hd_map.info.meta_data.origin.position.y},
                              {hd_map.info.meta_data.width, hd_map.info.meta_data.height}));
        onMapChanged(map_data);
        ROS_INFO_STREAM("Updating map for: " << name_ << " DONE");
    }

    void initialize(const std::string& name, const std::string& global_frame, const XmlRpc::XmlRpcValue& parameters,
                    const std::vector<Eigen::Vector2d>& robot_footprint,
                    const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
    {
        name_ = name;
        global_frame_ = global_frame;
        tf_buffer_ = tf_buffer;
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

    const std::shared_ptr<tf2_ros::Buffer> tfBuffer() const
    {
        return tf_buffer_;
    }

    const std::vector<Eigen::Vector2d> robotFootprint() const
    {
        return robot_footprint_;
    }

  protected:
    mutable std::timed_mutex map_mutex_;

  private:
    std::shared_ptr<hd_map::Map> hd_map_;
    std::shared_ptr<MapDimensions> map_dimensions_;

    std::string name_;
    std::string global_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::vector<Eigen::Vector2d> robot_footprint_;
};
}  // namespace gridmap

#endif
