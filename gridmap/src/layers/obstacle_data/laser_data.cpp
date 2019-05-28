#include <gridmap/layers/obstacle_data/laser_data.h>
#include <gridmap/params.h>

#include <pluginlib/class_list_macros.h>

#include <chrono>

PLUGINLIB_EXPORT_CLASS(gridmap::LaserData, gridmap::DataSource)

namespace gridmap
{

LaserData::LaserData()
{
}

void LaserData::onInitialize(const XmlRpc::XmlRpcValue& parameters)
{
    ros::NodeHandle nh("~/" + name_);
    ros::NodeHandle g_nh;

    const std::string topic = get_config_with_default_warn<std::string>(parameters, "topic", name_ + "/scan",
                                                                        XmlRpc::XmlRpcValue::TypeString);
    hit_probability_log_ = logodds(
        get_config_with_default_warn<double>(parameters, "hit_probability", 0.8, XmlRpc::XmlRpcValue::TypeDouble));
    miss_probability_log_ = logodds(
        get_config_with_default_warn<double>(parameters, "miss_probability", 0.4, XmlRpc::XmlRpcValue::TypeDouble));
    min_obstacle_height_ =
        get_config_with_default_warn<double>(parameters, "min_obstacle_height", 0.0, XmlRpc::XmlRpcValue::TypeDouble);
    max_obstacle_height_ =
        get_config_with_default_warn<double>(parameters, "max_obstacle_height", 2.0, XmlRpc::XmlRpcValue::TypeDouble);
    obstacle_range_ =
        get_config_with_default_warn<double>(parameters, "obstacle_range", 2.5, XmlRpc::XmlRpcValue::TypeDouble);
    raytrace_range_ =
        get_config_with_default_warn<double>(parameters, "raytrace_range", 3.0, XmlRpc::XmlRpcValue::TypeDouble);
    sub_sample_ = get_config_with_default_warn<int>(parameters, "sub_sample", 10, XmlRpc::XmlRpcValue::TypeInt);

    ROS_INFO_STREAM("Subscribing to laser: " << topic);

    subscriber_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));
    message_filter_.reset(
        new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*subscriber_, *tf_buffer_, global_frame_, 50, g_nh));
    message_filter_->registerCallback(boost::bind(&LaserData::laserScanCallback, this, _1));
}

void LaserData::onMapDataChanged()
{
}

LaserData::~LaserData()
{
}

void LaserData::laserScanCallback(const sensor_msgs::LaserScanConstPtr& message)
{
    if (sub_sample_ == 0 || (sub_sample_ > 0 && sub_sample_count_ > sub_sample_))
    {
        sub_sample_count_ = 0;

        std::lock_guard<std::mutex> lock(mutex_);
        if (!map_data_)
            return;

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
            ROS_WARN("Laser sensor is not on gridmap");
            return;
        }

        if (laser_directions_.size() != message->ranges.size())
        {
            laser_directions_.resize(message->ranges.size());
            double angle = static_cast<double>(message->angle_min);
            for (size_t i = 0; i < message->ranges.size(); i++)
            {
                laser_directions_[i] = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()) * Eigen::Vector3d::UnitX();
                angle += static_cast<double>(message->angle_increment);
            }
        }

        const unsigned int cell_raytrace_range = raytrace_range_ / map_data_->dimensions().resolution();

        {
            auto lock = map_data_->getLock();
            AddLogCost marker(map_data_->cells().data(), miss_probability_log_, map_data_->clampingThresMinLog(),
                              map_data_->clampingThresMaxLog());
            for (size_t i = 0; i < message->ranges.size(); i++)
            {
                double range = static_cast<double>(message->ranges[i]);
                if (!std::isfinite(range) && range > 0)
                {
                    range = static_cast<double>(message->range_max);
                }

                const Eigen::Vector3d pt = t * (range * laser_directions_[i]);
                if (pt.z() < min_obstacle_height_ || pt.z() > max_obstacle_height_)
                {
                    continue;
                }

                const Eigen::Vector2d pt_2d(pt.x(), pt.y());
                Eigen::Array2i ray_end = map_data_->dimensions().getCellIndex(pt_2d);
                clipRayEnd(sensor_pt_map, ray_end, map_data_->dimensions().size());
                raytraceLine(marker, sensor_pt_map.x(), sensor_pt_map.y(), ray_end.x(), ray_end.y(),
                             map_data_->dimensions().size().x(), cell_raytrace_range);
                if (range < static_cast<double>(message->range_max) && range < obstacle_range_)
                {
                    map_data_->update(ray_end, -miss_probability_log_);
                    map_data_->update(ray_end, hit_probability_log_);
                }
            }
        }
    }
    else
        ++sub_sample_count_;
}
}
