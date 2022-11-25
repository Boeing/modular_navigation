#ifndef GRIDMAP_LAYER_H
#define GRIDMAP_LAYER_H

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <graph_map/msg/zone.hpp>
#include <gridmap/grids/grid_2d.h>
#include <gridmap/grids/occupancy_grid.h>
#include <gridmap/robot_tracker.h>
#include <gridmap/urdf_tree.h>
#include <map_manager/msg/map_info.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>

// For logging reasons
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"

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
    virtual void onMapChanged(const nav_msgs::msg::OccupancyGrid& map_data) = 0;

    virtual bool clear() = 0;
    virtual bool clearRadius(const Eigen::Vector2i& cell_index, const int cell_radius) = 0;

    void setMap(const map_manager::msg::MapInfo& map_info, const nav_msgs::msg::OccupancyGrid& map_data,
                const std::vector<graph_map::msg::Zone>& zones)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger(""), "Updating map: " << name());
        // cppcheck-suppress unreadVariable
        const auto lock = getWriteLock();
        map_info_ = std::make_shared<map_manager::msg::MapInfo>(map_info);
        zones_ = std::make_shared<std::vector<graph_map::msg::Zone>>(zones);
        map_dimensions_.reset(new MapDimensions(
            map_info.meta_data.resolution, {map_info.meta_data.origin.position.x, map_info.meta_data.origin.position.y},
            {map_info.meta_data.width, map_info.meta_data.height}));
        onMapChanged(map_data);
        RCLCPP_INFO_STREAM(rclcpp::get_logger(""), "Updating map: " << name() << " DONE");
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

    const map_manager::msg::MapInfo& mapInfo() const
    {
        return *map_info_;
    }

    const std::vector<graph_map::msg::Zone>& zones() const
    {
        return *zones_;
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

    boost::shared_lock<boost::shared_mutex> getReadLock() const
    {
        return boost::shared_lock<boost::shared_mutex>(layer_mutex_);
    }

    // By default, the unique_lock (write lock) has priority over shared_lock. This means while the writer is trying
    // to get a lock, new readers cannot grab the lock. This effectively means one reader can block another reader
    // if a writer is trying to get the lock.
    // To get around this, we use a non-blocking try_to_lock every 5ms and during the sleeps, new readers can lock.
    // This means readers have priority and writers have to wait for windows where there are no reader.
    boost::unique_lock<boost::shared_mutex> getWriteLock() const
    {
        boost::unique_lock<boost::shared_mutex> lock(layer_mutex_, boost::try_to_lock_t());
        while (!lock.owns_lock())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            lock.try_lock();
        }
        return lock;
    }

  protected:
    mutable boost::shared_timed_mutex layer_mutex_;

    std::shared_ptr<RobotTracker> robot_tracker_;
    std::shared_ptr<URDFTree> urdf_tree_;
    std::vector<Eigen::Vector2d> robot_footprint_;

  private:
    std::shared_ptr<map_manager::msg::MapInfo> map_info_;
    std::shared_ptr<std::vector<graph_map::msg::Zone>> zones_;
    std::shared_ptr<MapDimensions> map_dimensions_;
    std::string name_;

    std::string global_frame_ = "map";
};
}  // namespace gridmap

#endif
