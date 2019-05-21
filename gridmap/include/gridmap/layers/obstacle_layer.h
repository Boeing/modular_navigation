#ifndef GRIDMAP_OBSTACLE_LAYER_H
#define GRIDMAP_OBSTACLE_LAYER_H

#include <gridmap/grids/probability_grid.h>
#include <gridmap/layers/layer.h>
#include <gridmap/layers/obstacle_data/data_source.h>

#include <pluginlib/class_loader.h>

#include <unordered_map>
#include <vector>
#include <atomic>
#include <thread>

namespace gridmap
{

class ObstacleLayer : public Layer
{
  public:
    ObstacleLayer();
    virtual ~ObstacleLayer();

    virtual void draw(OccupancyGrid& grid) override;
    virtual void draw(OccupancyGrid& grid, const AABB& bb) override;

    virtual void update(OccupancyGrid& grid) override;
    virtual void update(OccupancyGrid& grid, const AABB& bb) override;

    virtual void onInitialize(const XmlRpc::XmlRpcValue& parameters) override;
    virtual void onMapChanged(const nav_msgs::OccupancyGrid& map_data) override;

  private:
    std::shared_ptr<ProbabilityGrid> probability_grid_;

    pluginlib::ClassLoader<gridmap::DataSource> ds_loader_;
    std::unordered_map<std::string, std::shared_ptr<gridmap::DataSource>> data_sources_;

    bool debug_viz_;
    std::atomic<bool> debug_viz_running_;
    double debug_viz_rate_;
    std::thread debug_viz_thread_;
    ros::Publisher debug_viz_pub_;
    void debugVizThread(const double frequency);

    double clamping_thres_min_;
    double clamping_thres_max_;
    double occ_prob_thres_;

    bool time_decay_;
    std::atomic<bool> time_decay_running_;
    double time_decay_frequency_;
    double time_decay_step_;
    std::thread time_decay_thread_;
    void timeDecayThread(const double frequency, const double log_odds_decay);
};
}

#endif
