#ifndef GRIDMAP_DATA_SOURCE_H
#define GRIDMAP_DATA_SOURCE_H

#include <memory>
#include <mutex>
#include <string>

#include <gridmap/grids/probability_grid.h>
#include <gridmap/operations/rasterize.h>

#include <tf2_ros/buffer.h>

namespace gridmap
{

inline uint64_t IndexToKey(const Eigen::Array2i& index)
{
    uint64_t k_0(static_cast<uint32_t>(index[0]));
    uint64_t k_1(static_cast<uint32_t>(index[1]));
    return (k_0 << 32) | k_1;
}

inline Eigen::Array2i KeyToIndex(const uint64_t& key)
{
    return Eigen::Array2i(static_cast<int32_t>((key >> 32) & 0xFFFFFFFF), static_cast<int32_t>(key & 0xFFFFFFFF));
}

inline Eigen::Isometry2d convert(const geometry_msgs::Transform& tr)
{
    const double yaw = std::atan2(2.0 * (tr.rotation.z * tr.rotation.w + tr.rotation.x * tr.rotation.y),
                                  -1.0 + 2.0 * (tr.rotation.w * tr.rotation.w + tr.rotation.x * tr.rotation.x));
    return Eigen::Translation2d(tr.translation.x, tr.translation.y) * Eigen::Rotation2Dd(yaw);
}

inline std::set<uint64_t> buildFootprintSet(const MapDimensions& dimensions, const Eigen::Isometry2d& robot_pose,
                                            const std::vector<Eigen::Vector2d>& footprint)
{
    int min_x = std::numeric_limits<int>::max();
    int max_x = 0;

    int min_y = std::numeric_limits<int>::max();
    int max_y = 0;

    std::vector<Eigen::Array2i> map_footprint;
    for (const Eigen::Vector2d& p : footprint)
    {
        const Eigen::Vector2d world_point = robot_pose.translation() + robot_pose.rotation() * p;
        const auto map_point = dimensions.getCellIndex(world_point);
        map_footprint.push_back(map_point);
        min_x = std::min(map_point.x(), min_x);
        max_x = std::max(map_point.x(), max_x);
        min_y = std::min(map_point.y(), min_y);
        max_y = std::max(map_point.y(), max_y);
    }

    const std::vector<Eigen::Array2i> connected_poly = connectPolygon(map_footprint);

    std::set<uint64_t> footprint_set;
    auto insert_set = [&footprint_set](const int x, const int y) { footprint_set.insert(IndexToKey({x, y})); };

    gridmap::rasterPolygonFill(insert_set, connected_poly, min_x, max_x, min_y, max_y);

    return footprint_set;
}

class DataSource
{
  public:
    DataSource() = default;
    virtual ~DataSource() = default;

    void initialize(const std::string& name, const std::string& global_frame, const XmlRpc::XmlRpcValue& parameters,
                    const std::vector<Eigen::Vector2d>& robot_footprint,
                    const std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const double maximum_sensor_delay)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        name_ = name;
        global_frame_ = global_frame;
        map_data_ = nullptr;
        robot_footprint_ = robot_footprint;
        tf_buffer_ = tf_buffer;
        maximum_sensor_delay_ = maximum_sensor_delay;
        last_updated_ = ros::Time::now();
        onInitialize(parameters);
    }

    void setMapData(const std::shared_ptr<ProbabilityGrid>& map_data)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        map_data_ = map_data;
        onMapDataChanged();
    }

    std::string name() const
    {
        return name_;
    }

    struct DataStatus
    {
        bool ok;
        double seconds_since_update;
    };

    DataStatus status() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const double delay = (ros::Time::now() - last_updated_).toSec();
        return {delay < maximum_sensor_delay_, delay};
    }

  protected:
    virtual void onInitialize(const XmlRpc::XmlRpcValue& parameters) = 0;
    virtual void onMapDataChanged() = 0;

    void setLastUpdatedTime(const ros::Time& time)
    {
        last_updated_ = time;
    }

    mutable std::mutex mutex_;

    std::string name_;
    std::string global_frame_;
    std::shared_ptr<ProbabilityGrid> map_data_;
    std::vector<Eigen::Vector2d> robot_footprint_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    ros::Time last_updated_;
    double maximum_sensor_delay_;
};
}

#endif
