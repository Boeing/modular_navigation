#include <gridmap/layers/obstacle_data/point_cloud_data.h>
#include <gridmap/params.h>

#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/point_cloud2_iterator.h>

#include <chrono>
#include <unordered_map>
#include <unordered_set>

PLUGINLIB_EXPORT_CLASS(gridmap::PointCloudData, gridmap::DataSource)

namespace gridmap
{

PointCloudData::PointCloudData()
    : hit_probability_log_(0), miss_probability_log_(0), obstacle_height_(0), max_range_(0), sub_sample_(0),
      sub_sample_count_(0)
{
}

void PointCloudData::onInitialize(const XmlRpc::XmlRpcValue& parameters)
{
    ros::NodeHandle nh("~/" + name_);
    ros::NodeHandle g_nh;

    const std::string topic = get_config_with_default_warn<std::string>(parameters, "topic", name_ + "/points",
                                                                        XmlRpc::XmlRpcValue::TypeString);
    miss_probability_log_ = logodds(get_config_with_default_warn<double>(parameters, "miss_probability", 0.4, XmlRpc::XmlRpcValue::TypeDouble));
    obstacle_height_ = get_config_with_default_warn<double>(parameters, "max_obstacle_height", 0.03, XmlRpc::XmlRpcValue::TypeDouble);
    max_range_ = get_config_with_default_warn<double>(parameters, "max_range", 2.0, XmlRpc::XmlRpcValue::TypeDouble);
    sub_sample_ = get_config_with_default_warn<int>(parameters, "sub_sample", 0, XmlRpc::XmlRpcValue::TypeInt);

    ROS_INFO_STREAM("Subscribing to point cloud: " << topic);

    subscriber_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));
    message_filter_.reset(
        new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*subscriber_, *tf_buffer_, global_frame_, 50, g_nh));
    message_filter_->registerCallback(boost::bind(&PointCloudData::pointCloudCallback, this, _1));
}

void PointCloudData::onMapDataChanged()
{
    const double cost_gradient = (-miss_probability_log_ / obstacle_height_);
    const double max_height = 10.0 / cost_gradient;

    const unsigned int max_height_cells =
        static_cast<unsigned int>(max_height / map_data_->dimensions().resolution()) + 1;
    log_cost_lookup_.resize(max_height_cells);
    for (unsigned int i = 0; i < max_height_cells; ++i)
    {
        const double x = i * map_data_->dimensions().resolution();
        log_cost_lookup_[i] = x * (-miss_probability_log_ / obstacle_height_) + miss_probability_log_;
    }
}

PointCloudData::~PointCloudData()
{
}

void PointCloudData::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& message)
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
        const Eigen::Isometry3f t_f = t.cast<float>();

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

        const auto robot_tr = tf_buffer_->lookupTransform(global_frame_, "base_link", message->header.stamp);
        const Eigen::Isometry2d robot_t = convert(robot_tr.transform);
        const auto footprint = buildFootprintSet(map_data_->dimensions(), robot_t, robot_footprint_);

        {
            auto _lock = map_data_->getLock();

            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*message, "x");

            std::unordered_map<uint64_t, float> height_voxels;

            for (; iter_x != iter_x.end(); ++iter_x)
            {
                if (!std::isfinite(iter_x[2]))
                {
                    continue;
                }

                const Eigen::Vector3f reading(iter_x[0], iter_x[1], iter_x[2]);

                if (reading.norm() > max_range_)
                {
                    continue;
                }

                const Eigen::Vector3f pt = t_f * reading;
                const Eigen::Array2i pt_map = map_data_->dimensions().getCellIndex(pt.head<2>().cast<double>());

                const auto key = IndexToKey(pt_map);

                if (footprint.count(key) > 0)
                {
                    continue;
                }

                const auto insert_ret = height_voxels.find(key);

                if (insert_ret == height_voxels.end())
                {
                    height_voxels.insert(std::make_pair(key, pt.z()));
                }
                else
                {
                    if (pt.z() > insert_ret->second)
                        height_voxels[key] = pt.z();
                }
            }
            for (auto elem : height_voxels)
            {
                const size_t height_in_cells =
                    static_cast<size_t>(std::abs(elem.second) / map_data_->dimensions().resolution());
                const double log_odds = log_cost_lookup_[std::min(height_in_cells, log_cost_lookup_.size() - 1)];
                const Eigen::Array2i index = KeyToIndex(elem.first);
                if (map_data_->dimensions().contains(index))
                    map_data_->update(index, log_odds);
            }
            for (auto elem : footprint)
            {
                const Eigen::Array2i index = KeyToIndex(elem);
                if (map_data_->dimensions().contains(index))
                    map_data_->setMinThres(index);
            }
        }

        setLastUpdatedTime(message->header.stamp);
    }
    else
        ++sub_sample_count_;
}
}
