#ifndef GRIDMAP_OBSTACLE_LAYER_H
#define GRIDMAP_OBSTACLE_LAYER_H

#include <gridmap/grids/probability_grid.h>
#include <gridmap/layers/layer.h>
#include <gridmap/layers/obstacle_data/data_source.h>
#include <pluginlib/class_loader.hpp>

#include <atomic>
#include <thread>
#include <unordered_map>
#include <vector>

namespace gridmap {

class ObstacleLayer : public Layer {
public:
  ObstacleLayer();
  virtual ~ObstacleLayer();

  virtual bool draw(OccupancyGrid &grid) const override;
  virtual bool draw(OccupancyGrid &grid, const AABB &bb) const override;

  virtual bool update(OccupancyGrid &grid) const override;
  virtual bool update(OccupancyGrid &grid, const AABB &bb) const override;

  virtual void onInitialize(const YAML::Node &parameters) override;
  virtual void
  onMapChanged(const nav_msgs::msg::OccupancyGrid &map_data) override;

  virtual bool clear() override;
  virtual bool clearRadius(const Eigen::Vector2i &cell_index,
                           const int cell_radius) override;

private:
  bool isDataOk() const;

  std::shared_ptr<ProbabilityGrid> probability_grid_;

  pluginlib::ClassLoader<gridmap::DataSource> ds_loader_;
  std::unordered_map<std::string, std::shared_ptr<gridmap::DataSource>>
      data_sources_;

  double clamping_thres_min_ = 0.1192;
  double clamping_thres_max_ = 0.971;
  double occ_prob_thres_ = 0.8;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_viz_pub_;

  bool debug_viz_ = true;
  std::atomic<bool> debug_viz_running_;
  double debug_viz_frequency_ = 4.0;
  std::thread debug_viz_thread_;
  void debugVizThread(const double frequency);

  std::atomic<bool> clear_footprint_running_;
  double clear_footprint_frequency_ = 10.0;
  std::thread clear_footprint_thread_;
  void clearFootprintThread(const double frequency);

  bool time_decay_ = true;
  std::atomic<bool> time_decay_running_;
  double time_decay_frequency_ = 1.0;
  double alpha_decay_ = 1.0 - std::pow(0.001, 1.0 / 10.0);
  std::thread time_decay_thread_;
  void timeDecayThread(const double frequency, const double alpha_decay);
};
} // namespace gridmap

#endif
