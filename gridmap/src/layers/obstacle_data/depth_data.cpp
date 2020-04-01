#include <gridmap/layers/obstacle_data/depth_data.h>
#include <gridmap/params.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <chrono>
#include <unordered_set>

PLUGINLIB_EXPORT_CLASS(gridmap::DepthData, gridmap::DataSource)

namespace gridmap
{

DepthData::DepthData()
    : TopicDataSource<sensor_msgs::Image>("depth/image_rect_raw"), hit_probability_log_(0), miss_probability_log_(0),
      obstacle_height_(0), max_range_(0)
{
}

void DepthData::onInitialize(const XmlRpc::XmlRpcValue& parameters)
{
    miss_probability_log_ = logodds(
        get_config_with_default_warn<double>(parameters, "miss_probability", 0.4, XmlRpc::XmlRpcValue::TypeDouble));
    obstacle_height_ =
        get_config_with_default_warn<double>(parameters, "max_obstacle_height", 0.10, XmlRpc::XmlRpcValue::TypeDouble);
    max_range_ = static_cast<float>(
        get_config_with_default_warn<double>(parameters, "max_range", 1.5, XmlRpc::XmlRpcValue::TypeDouble));

    camera_info_topic_ = get_config_with_default_warn<std::string>(
        parameters, "camera_info_topic", std::string(name_ + "/depth/camera_info"), XmlRpc::XmlRpcValue::TypeString);

    ROS_INFO_STREAM("camera_info_topic: " << camera_info_topic_);

    const auto camera_info =
        ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_, ros::Duration(10.0));
    if (!camera_info)
    {
        ROS_ERROR("NO CAM INFO");
        throw std::runtime_error("No camera info for '" + name_ + "' on topic '" + camera_info_topic_ + "'");
    }
    camera_model_.fromCameraInfo(*camera_info);
}

void DepthData::onMapDataChanged()
{
}

DepthData::~DepthData()
{
}

bool DepthData::processData(const sensor_msgs::Image::ConstPtr& msg, const Eigen::Isometry3d& sensor_transform)
{
    const Eigen::Isometry3f t_f = sensor_transform.cast<float>();

    const Eigen::Vector3d sensor_pt = sensor_transform.translation();
    const Eigen::Vector2d sensor_pt_2d(sensor_pt.x(), sensor_pt.y());
    const Eigen::Vector2i sensor_pt_map = map_data_->dimensions().getCellIndex(sensor_pt_2d);

    // Check sensor is on map
    if (sensor_pt_map.x() < 0 || sensor_pt_map.x() >= map_data_->dimensions().size().x() || sensor_pt_map.y() < 0 ||
        sensor_pt_map.y() >= map_data_->dimensions().size().y())
    {
        ROS_WARN("Sensor is not on gridmap");
        return false;
    }

    const auto robot_tr = tf_buffer_->lookupTransform(global_frame_, "base_link", msg->header.stamp);
    const Eigen::Isometry2d robot_t = convert(robot_tr.transform);
    // add a 5% buffer
    const auto footprint = buildFootprintSet(map_data_->dimensions(), robot_t, robot_footprint_, 1.05);

    {
        // cppcheck-suppress unreadVariable
        auto _lock = map_data_->getLock();

        std::unordered_map<uint64_t, float> height_voxels;

        if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
            process<uint16_t>(height_voxels, t_f, footprint, msg);
        else
            ROS_ASSERT("Unsupported depth image format");

        for (auto elem : height_voxels)
        {
            const double h = std::max(-0.1f, std::min(0.3f, elem.second));
            const double log_odds = h * (-miss_probability_log_ / 0.1) + miss_probability_log_;

            const Eigen::Array2i index = KeyToIndex(elem.first);
            if (map_data_->dimensions().contains(index))
                map_data_->update(index, log_odds);
        }
    }

    return true;
}

template <typename T>
void DepthData::process(std::unordered_map<uint64_t, float>& height_voxels, const Eigen::Isometry3f& sensor_transform,
                        const std::set<uint64_t>& footprint, const sensor_msgs::Image::ConstPtr& msg)
{
    // Use correct principal point from calibration
    const float center_x = camera_model_.cx();
    const float center_y = camera_model_.cy();

    const double unit_scaling = 0.001;
    const float constant_x = unit_scaling / camera_model_.fx();
    const float constant_y = unit_scaling / camera_model_.fy();

    const T* depth_row = reinterpret_cast<const T*>(&msg->data[0]);
    int row_step = msg->step / sizeof(T);
    for (int v = 0; v < (int)msg->height; ++v, depth_row += row_step)
    {
        for (int u = 0; u < (int)msg->width; ++u)
        {
            T depth = depth_row[u];

            // Missing points denoted by NaNs
            if (depth == 0)
            {
                if (max_range_ != 0.0)
                {
                    depth = (max_range_ * 1000.0f) + 0.5f;
                }
                else
                {
                    continue;
                }
            }

            const Eigen::Vector3f reading((u - center_x) * depth * constant_x, (v - center_y) * depth * constant_y,
                                          depth * 0.001f);

            ROS_ASSERT(reading.allFinite());

            if (reading.norm() > max_range_)
            {
                continue;
            }

            const Eigen::Vector3f pt = sensor_transform * reading;
            const Eigen::Array2i pt_map = map_data_->dimensions().getCellIndex(pt.head<2>().cast<double>());

            const auto key = IndexToKey(pt_map);

            if (footprint.count(key) > 0)
            {
                if (pt.z() < 0.40)
                {
                    height_voxels[key] = 0;
                    continue;
                }
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
    }
}

}  // namespace gridmap
