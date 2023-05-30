#include <gridmap/layers/obstacle_data/point_cloud_data.h>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <chrono>
#include <unordered_map>
#include <unordered_set>

// For logging reasons
#include "rclcpp/rclcpp.hpp"

PLUGINLIB_EXPORT_CLASS(gridmap::PointCloudData, gridmap::DataSource)

namespace gridmap {

PointCloudData::PointCloudData()
    : TopicDataSource<sensor_msgs::msg::PointCloud2>("points"),
      hit_probability_log_(0), miss_probability_log_(0), obstacle_height_(0),
      max_range_(0) {}

void PointCloudData::onInitialize(const YAML::Node &parameters) {
  miss_probability_log_ =
      logodds(parameters["miss_probability_log"].as<double>(0.4));
  obstacle_height_ = parameters["obstacle_height"].as<double>(0.03);
  max_range_ = parameters["max_range"].as<float>(2.0);
}

void PointCloudData::onMapDataChanged() {
  const std::shared_ptr<ProbabilityGrid> map_data = get_map_data();

  const double cost_gradient = (-miss_probability_log_ / obstacle_height_);
  const double max_height = 10.0 / cost_gradient;

  const unsigned int max_height_cells =
      static_cast<unsigned int>(max_height /
                                map_data->dimensions().resolution()) +
      1;
  log_cost_lookup_.resize(max_height_cells);
  for (unsigned int i = 0; i < max_height_cells; ++i) {
    const double x = i * map_data->dimensions().resolution();
    log_cost_lookup_[i] =
        x * (-miss_probability_log_ / obstacle_height_) + miss_probability_log_;
  }
}

PointCloudData::~PointCloudData() {}

bool PointCloudData::processData(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    const Eigen::Isometry2d &robot_pose,
    const Eigen::Isometry3d &sensor_transform) {
  const std::shared_ptr<ProbabilityGrid> map_data = get_map_data();

  const Eigen::Isometry3f t_f = sensor_transform.cast<float>();

  const Eigen::Vector3d sensor_pt = sensor_transform.translation();
  const Eigen::Vector2d sensor_pt_2d(sensor_pt.x(), sensor_pt.y());
  const Eigen::Vector2i sensor_pt_map =
      map_data->dimensions().getCellIndex(sensor_pt_2d);

  // Check sensor is on map
  if (sensor_pt_map.x() < 0 ||
      sensor_pt_map.x() >= map_data->dimensions().size().x() ||
      sensor_pt_map.y() < 0 ||
      sensor_pt_map.y() >= map_data->dimensions().size().y()) {
    RCLCPP_WARN(rclcpp::get_logger(""), "Sensor is not on gridmap");
    return false;
  }

  // add a 5% buffer
  const auto footprint = buildFootprintSet(map_data->dimensions(), robot_pose,
                                           robot_footprint_, 1.05);

  {
    auto _lock = map_data->getWriteLock();

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");

    std::unordered_map<uint64_t, float> height_voxels;

    for (; iter_x != iter_x.end(); ++iter_x) {
      if (!std::isfinite(iter_x[2])) {
        continue;
      }

      const Eigen::Vector3f reading(iter_x[0], iter_x[1], iter_x[2]);

      if (reading.norm() > max_range_) {
        continue;
      }

      const Eigen::Vector3f pt = t_f * reading;
      const Eigen::Array2i pt_map =
          map_data->dimensions().getCellIndex(pt.head<2>().cast<double>());

      const auto key = IndexToKey(pt_map);

      if (footprint.count(key) > 0) {
        continue;
      }

      const auto insert_ret = height_voxels.find(key);

      if (insert_ret == height_voxels.end()) {
        height_voxels.insert(std::make_pair(key, pt.z()));
      } else {
        if (pt.z() > insert_ret->second)
          height_voxels[key] = pt.z();
      }
    }
    for (auto elem : height_voxels) {
      const size_t height_in_cells = static_cast<size_t>(
          std::abs(elem.second) /
          static_cast<float>(map_data->dimensions().resolution()));
      const double log_odds = log_cost_lookup_[std::min(
          height_in_cells, log_cost_lookup_.size() - 1)];
      const Eigen::Array2i index = KeyToIndex(elem.first);
      if (map_data->dimensions().contains(index))
        map_data->update(index, log_odds);
    }
  }

  return true;
}

} // namespace gridmap
