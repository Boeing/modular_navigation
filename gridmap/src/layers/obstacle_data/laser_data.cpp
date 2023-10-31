#include <gridmap/layers/obstacle_data/laser_data.h>
#include <gridmap/operations/clip_line.h>
#include <pluginlib/class_list_macros.hpp>

// For logging reasons
#include <chrono>

#include "rclcpp/rclcpp.hpp"

PLUGINLIB_EXPORT_CLASS(gridmap::LaserData, gridmap::DataSource)

namespace gridmap
{

LaserData::LaserData()
    : TopicDataSource<sensor_msgs::msg::LaserScan>("scan"), hit_probability_log_(0), miss_probability_log_(0),
      min_obstacle_height_(0), max_obstacle_height_(0), obstacle_range_(0), raytrace_range_(0)
{
}

void LaserData::onInitialize(const YAML::Node& parameters)
{
    hit_probability_log_ = logodds(parameters["hit_probability_log"].as<double>(0.8));
    miss_probability_log_ = logodds(parameters["miss_probability_log"].as<double>(0.4));
    min_obstacle_height_ = parameters["min_obstacle_height"].as<double>(0.0);
    max_obstacle_height_ = parameters["max_obstacle_height"].as<double>(2.0);
    obstacle_range_ = parameters["obstacle_range"].as<double>(3.5);
    raytrace_range_ = parameters["raytrace_range"].as<double>(4.0);
    initChronoTime_ = std::chrono::system_clock::now();
}

void LaserData::onMapDataChanged()
{
}

bool LaserData::processData(const sensor_msgs::msg::LaserScan::SharedPtr msg, const Eigen::Isometry2d& robot_pose,
                            const Eigen::Isometry3d& sensor_transform)
{
    const std::shared_ptr<ProbabilityGrid> map_data = get_map_data();

    const Eigen::Vector3d sensor_pt = sensor_transform.translation();
    const Eigen::Vector2d sensor_pt_2d(sensor_pt.x(), sensor_pt.y());
    const Eigen::Vector2i sensor_pt_map = map_data->dimensions().getCellIndex(sensor_pt_2d);

    // Check sensor is on map
    if (sensor_pt_map.x() < 0 || sensor_pt_map.x() >= map_data->dimensions().size().x() || sensor_pt_map.y() < 0 ||
        sensor_pt_map.y() >= map_data->dimensions().size().y())
    {
        if (std::chrono::system_clock::now() - initChronoTime_ >  std::chrono::seconds(INIT_PRINT_DELAY));
        { // Only log once everything is up and running
            RCLCPP_WARN(rclcpp::get_logger(""), "Laser sensor is not on gridmap");
        }
        return false;
    }

    if (laser_directions_.size() != msg->ranges.size())
    {
        laser_directions_.resize(msg->ranges.size());
        double angle = static_cast<double>(msg->angle_min);
        for (size_t i = 0; i < msg->ranges.size(); i++)
        {
            laser_directions_[i] = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d::UnitX();
            angle += static_cast<double>(msg->angle_increment);
        }
    }

    const auto footprint = buildFootprintSet(map_data->dimensions(), robot_pose, robot_footprint_, 1.00);

    const unsigned int cell_raytrace_range =
        static_cast<unsigned int>(raytrace_range_ / map_data->dimensions().resolution());

    {
        auto _lock = map_data->getWriteLock();
        AddLogCost marker(map_data->cells().data(), miss_probability_log_, map_data->clampingThresMinLog(),
                          map_data->clampingThresMaxLog());
        for (size_t i = 0; i < msg->ranges.size(); i++)
        {
            double range = static_cast<double>(msg->ranges[i]);
            if (!std::isfinite(range) && range > 0)
            {
                range = static_cast<double>(msg->range_max);
            }

            const Eigen::Vector3d pt = sensor_transform * (range * laser_directions_[i]);
            if (pt.z() < min_obstacle_height_ || pt.z() > max_obstacle_height_)
            {
                continue;
            }

            const Eigen::Vector2d pt_2d(pt.x(), pt.y());
            Eigen::Array2i ray_end = map_data->dimensions().getCellIndex(pt_2d);
            cohenSutherlandLineClipEnd(sensor_pt_map.x(), sensor_pt_map.y(), ray_end.x(), ray_end.y(),
                                       map_data->dimensions().size().x() - 1, map_data->dimensions().size().y() - 1);
            raytraceLine(marker, sensor_pt_map.x(), sensor_pt_map.y(), ray_end.x(), ray_end.y(),
                         map_data->dimensions().size().x(), cell_raytrace_range);
            if (range < static_cast<double>(msg->range_max) && range < obstacle_range_)
            {
                map_data->update(ray_end, -miss_probability_log_);
                map_data->update(ray_end, hit_probability_log_);
            }

            for (auto elem : footprint)
            {
                const Eigen::Array2i index = KeyToIndex(elem);
                if (map_data->dimensions().contains(index))
                    map_data->setMinThres(index);
            }
        }
    }
    return true;
}

}  // namespace gridmap
