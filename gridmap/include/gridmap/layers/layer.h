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

    virtual void draw(OccupancyGrid& grid) = 0;
    virtual void draw(OccupancyGrid& grid, const AABB& bb) = 0;

    virtual void update(OccupancyGrid& grid) = 0;
    virtual void update(OccupancyGrid& grid, const AABB& bb) = 0;

    virtual void onInitialize(const XmlRpc::XmlRpcValue& parameters) = 0;
    virtual void onMapChanged(const nav_msgs::OccupancyGrid& map_data) = 0;

    void setMap(const hd_map::Map& hd_map, const nav_msgs::OccupancyGrid& map_data)
    {
        hd_map_ = std::make_shared<hd_map::Map>(hd_map);
        map_dimensions_.reset(new MapDimensions(hd_map.map_info.resolution,
                                                {hd_map.map_info.origin.position.x, hd_map.map_info.origin.position.y},
                                                {hd_map.map_info.width, hd_map.map_info.height}));
        onMapChanged(map_data);
    }

    void initialize(const std::string& name, const std::string& global_frame, const XmlRpc::XmlRpcValue& parameters,
                    const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
    {
        name_ = name;
        global_frame_ = global_frame;
        tf_buffer_ = tf_buffer;
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

  private:
    std::shared_ptr<hd_map::Map> hd_map_;
    std::shared_ptr<MapDimensions> map_dimensions_;

    std::string name_;
    std::string global_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};
}

#endif
