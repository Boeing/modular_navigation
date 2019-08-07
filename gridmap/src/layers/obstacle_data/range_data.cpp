#include <gridmap/layers/obstacle_data/range_data.h>
#include <gridmap/params.h>

#include <chrono>
#include <fstream>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <gridmap/operations/clip_line.h>
#include <gridmap/operations/rasterize.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(gridmap::RangeData, gridmap::DataSource)

namespace gridmap
{

RangeData::RangeData()
    : hit_probability_(0), miss_probability_(0), std_deviation_(0), max_range_(0), obstacle_range_(0),
      raytrace_range_(0), sub_sample_(0), sub_sample_count_(0)
{
}

RangeData::~RangeData()
{
}

void RangeData::onInitialize(const XmlRpc::XmlRpcValue& parameters)
{
    ros::NodeHandle nh("~/" + name_);
    ros::NodeHandle g_nh;

    const std::string topic = get_config_with_default_warn<std::string>(parameters, "topic", name_ + "/range",
                                                                        XmlRpc::XmlRpcValue::TypeString);

    hit_probability_ =
        get_config_with_default_warn<double>(parameters, "hit_probability", 0.99, XmlRpc::XmlRpcValue::TypeDouble);
    miss_probability_ =
        get_config_with_default_warn<double>(parameters, "miss_probability", 0.4, XmlRpc::XmlRpcValue::TypeDouble);

    std_deviation_ =
        get_config_with_default_warn<double>(parameters, "std_deviation", 0.06, XmlRpc::XmlRpcValue::TypeDouble);

    max_range_ = get_config_with_default_warn<double>(parameters, "max_range", 1.5, XmlRpc::XmlRpcValue::TypeDouble);

    obstacle_range_ =
        get_config_with_default_warn<double>(parameters, "obstacle_range", 1.2, XmlRpc::XmlRpcValue::TypeDouble);
    raytrace_range_ =
        get_config_with_default_warn<double>(parameters, "raytrace_range", 1.2, XmlRpc::XmlRpcValue::TypeDouble);

    sub_sample_ = get_config_with_default_warn<int>(parameters, "sub_sample", 0, XmlRpc::XmlRpcValue::TypeInt);

    ROS_ASSERT(obstacle_range_ <= max_range_);
    ROS_ASSERT(raytrace_range_ <= max_range_);

    ROS_INFO_STREAM("Subscribing to range sensor: " << topic);

    subscriber_.reset(new message_filters::Subscriber<sensor_msgs::Range>(g_nh, topic, 50));
    message_filter_.reset(
        new tf2_ros::MessageFilter<sensor_msgs::Range>(*subscriber_, *tf_buffer_, global_frame_, 50, g_nh));
    message_filter_->registerCallback(boost::bind(&RangeData::rangeCallback, this, _1));
}

void RangeData::onMapDataChanged()
{
    const int obstacle_cells = obstacle_range_ / map_data_->dimensions().resolution();
    const int raytrace_cells = raytrace_range_ / map_data_->dimensions().resolution();
    const int max_range_cells = max_range_ / map_data_->dimensions().resolution() + 1;

    log_cost_lookup_.resize(max_range_cells);
    for (int c = 0; c < max_range_cells; ++c)
    {
        log_cost_lookup_[c].resize(max_range_cells);
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

void RangeData::rangeCallback(const sensor_msgs::RangeConstPtr& message)
{
    if (sub_sample_ == 0 || (sub_sample_ > 0 && sub_sample_count_ > sub_sample_))
    {
        sub_sample_count_ = 0;

        std::lock_guard<std::mutex> lock(mutex_);
        if (!map_data_)
            return;

        ROS_ASSERT(std::abs(message->max_range - max_range_) < std::numeric_limits<double>::epsilon());

        const auto tr = tf_buffer_->lookupTransform(global_frame_, message->header.frame_id, message->header.stamp);

        const Eigen::Isometry3d t =
            Eigen::Translation3d(tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z) *
            Eigen::Quaterniond(tr.transform.rotation.w, tr.transform.rotation.x, tr.transform.rotation.y,
                               tr.transform.rotation.z);

        const Eigen::Vector3d sensor_pt = t.translation();
        const Eigen::Vector2d sensor_pt_2d(sensor_pt.x(), sensor_pt.y());
        const Eigen::Vector2i sensor_pt_map = map_data_->dimensions().getCellIndex(sensor_pt_2d);

        // Check sensor is on map
        if (sensor_pt_map.x() < 0 || sensor_pt_map.x() >= map_data_->dimensions().size().x() || sensor_pt_map.y() < 0 ||
            sensor_pt_map.y() >= map_data_->dimensions().size().y())
        {
            ROS_WARN("Range sensor is not on gridmap");
            return;
        }

        const double half_fov = static_cast<double>(message->field_of_view / 2.0f);
        const double range = std::max(message->min_range, std::min(message->max_range, message->range));

        const Eigen::Vector3d centre_pt = t * Eigen::Vector3d((1.0 / std::cos(half_fov)) * range, 0.0, 0.0);
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

        const int cell_range = range / map_data_->dimensions().resolution();

        auto shader = [this, sensor_pt_map, cell_range](const int x, const int y, const int w0, const int w1,
                                                        const int w2) {
            const double _w0 = static_cast<double>(w0) / static_cast<double>(w0 + w1 + w2);
            const double _w1 = static_cast<double>(w1) / static_cast<double>(w0 + w1 + w2);
            const double _w2 = static_cast<double>(w2) / static_cast<double>(w0 + w1 + w2);

            int dist = cell_range * (1 - _w0);

            if (_w2 > 0.85)
                return;

            if (_w1 > 0.85)
                return;

            ROS_ASSERT(dist < log_cost_lookup_[cell_range].size());

            map_data_->update({x, y}, log_cost_lookup_[cell_range][dist]);
        };

        {
            auto lock = map_data_->getLock();

            drawTri(shader, {sensor_pt_map.x(), sensor_pt_map.y()}, {left_pt_map.x(), left_pt_map.y()},
                    {right_pt_map.x(), right_pt_map.y()});

            map_data_->setMinThres(sensor_pt_map);
        }

        setLastUpdatedTime(message->header.stamp);
    }
    else
        ++sub_sample_count_;
}
}
