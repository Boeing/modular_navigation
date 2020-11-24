#include <gridmap/layers/obstacle_data/range_data.h>
#include <gridmap/operations/clip_line.h>
#include <gridmap/operations/rasterize.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_list_macros.h>

#include <chrono>
#include <fstream>
#include <iostream>

PLUGINLIB_EXPORT_CLASS(gridmap::RangeData, gridmap::DataSource)

namespace gridmap
{

RangeData::RangeData()
    : TopicDataSource<sensor_msgs::Range>("scan"), hit_probability_(0), miss_probability_(0), std_deviation_(0),
      max_range_(0), obstacle_range_(0), raytrace_range_(0)
{
}

RangeData::~RangeData()
{
}

void RangeData::onInitialize(const YAML::Node& parameters)
{
    hit_probability_ = parameters["hit_probability_log"].as<double>(0.99);
    miss_probability_ = parameters["miss_probability_log"].as<double>(0.4);

    std_deviation_ = parameters["std_deviation"].as<double>(0.06);

    max_range_ = parameters["max_range"].as<double>(1.5);

    obstacle_range_ = parameters["obstacle_range"].as<double>(1.2);
    raytrace_range_ = parameters["raytrace_range"].as<double>(1.2);

    ROS_ASSERT(obstacle_range_ <= max_range_);
    ROS_ASSERT(raytrace_range_ <= max_range_);
}

void RangeData::onMapDataChanged()
{
    const int obstacle_cells = static_cast<int>(obstacle_range_ / map_data_->dimensions().resolution());
    const int raytrace_cells = static_cast<int>(raytrace_range_ / map_data_->dimensions().resolution());
    const int max_range_cells = static_cast<int>(max_range_ / map_data_->dimensions().resolution() + 1);

    log_cost_lookup_.resize(static_cast<size_t>(max_range_cells));
    for (int c = 0; c < max_range_cells; ++c)
    {
        log_cost_lookup_[c].resize(static_cast<size_t>(max_range_cells));
        const double range = c * map_data_->dimensions().resolution();

        for (int i = 0; i < max_range_cells; ++i)
        {
            const double x = i * map_data_->dimensions().resolution();

            if (x < range + std_deviation_)
            {
                const double pdf_gaussian =
                    hit_probability_ * std::exp(-0.5 * std::pow((x - range) / std_deviation_, 2.0));
                const double r_dist_scale = std::max(0.0, static_cast<double>(raytrace_cells - i) / raytrace_cells);
                const double o_dist_scale = std::max(0.0, static_cast<double>(obstacle_cells - i) / obstacle_cells);

                const double hit = i < obstacle_cells ? pdf_gaussian : 0.5;
                const double miss = i < raytrace_cells ? miss_probability_ : 0.5;

                log_cost_lookup_[c][i] = o_dist_scale * logodds(hit) + r_dist_scale * r_dist_scale * logodds(miss);
            }
            else
            {
                log_cost_lookup_[c][i] = 0;
            }
        }
    }
}

bool RangeData::processData(const sensor_msgs::Range::ConstPtr& msg, const Eigen::Isometry2d&,
                            const Eigen::Isometry3d& sensor_transform)
{
    ROS_ASSERT(std::abs(msg->max_range - max_range_) < std::numeric_limits<double>::epsilon());

    const Eigen::Vector3d sensor_pt = sensor_transform.translation();
    const Eigen::Vector2d sensor_pt_2d(sensor_pt.x(), sensor_pt.y());
    const Eigen::Vector2i sensor_pt_map = map_data_->dimensions().getCellIndex(sensor_pt_2d);

    // Check sensor is on map
    if (sensor_pt_map.x() < 0 || sensor_pt_map.x() >= map_data_->dimensions().size().x() || sensor_pt_map.y() < 0 ||
        sensor_pt_map.y() >= map_data_->dimensions().size().y())
    {
        ROS_WARN("Range sensor is not on gridmap");
        return false;
    }

    const double half_fov = static_cast<double>(msg->field_of_view / 2.0f);
    const double range = static_cast<double>(std::max(msg->min_range, std::min(msg->max_range, msg->range)));

    const Eigen::Vector3d centre_pt = sensor_transform * Eigen::Vector3d((1.0 / std::cos(half_fov)) * range, 0.0, 0.0);
    const Eigen::Vector2d centre_pt_2d(centre_pt.x(), centre_pt.y());

    const Eigen::Vector2d sensor_vec = (centre_pt_2d - sensor_pt_2d);

    const Eigen::Vector2d left_pt_2d = sensor_pt_2d + (Eigen::Rotation2Dd(-half_fov) * sensor_vec);
    const Eigen::Vector2d right_pt_2d = sensor_pt_2d + (Eigen::Rotation2Dd(half_fov) * sensor_vec);

    Eigen::Vector2i left_pt_map = map_data_->dimensions().getCellIndex(left_pt_2d);
    Eigen::Vector2i right_pt_map = map_data_->dimensions().getCellIndex(right_pt_2d);

    cohenSutherlandLineClipEnd(sensor_pt_map.x(), sensor_pt_map.y(), left_pt_map.x(), left_pt_map.y(),
                               map_data_->dimensions().size().x() - 1, map_data_->dimensions().size().y() - 1);

    cohenSutherlandLineClipEnd(sensor_pt_map.x(), sensor_pt_map.y(), right_pt_map.x(), right_pt_map.y(),
                               map_data_->dimensions().size().x() - 1, map_data_->dimensions().size().y() - 1);

    const std::size_t cell_range = static_cast<std::size_t>(range / map_data_->dimensions().resolution());

    auto shader = [this, sensor_pt_map, cell_range](const int x, const int y, const int w0, const int w1,
                                                    const int w2) {
        const double _w0 = static_cast<double>(w0) / static_cast<double>(w0 + w1 + w2);
        const double _w1 = static_cast<double>(w1) / static_cast<double>(w0 + w1 + w2);
        const double _w2 = static_cast<double>(w2) / static_cast<double>(w0 + w1 + w2);

        const std::size_t dist = static_cast<std::size_t>(cell_range * (1 - _w0));

        if (_w2 > 0.85)
            return;

        if (_w1 > 0.85)
            return;

        ROS_ASSERT(dist < log_cost_lookup_[cell_range].size());

        map_data_->update({x, y}, log_cost_lookup_[cell_range][dist]);
    };

    {
        auto _lock = map_data_->getLock();
        drawTri(shader, {sensor_pt_map.x(), sensor_pt_map.y()}, {left_pt_map.x(), left_pt_map.y()},
                {right_pt_map.x(), right_pt_map.y()});
        map_data_->setMinThres(sensor_pt_map);
    }

    return true;
}

}  // namespace gridmap
