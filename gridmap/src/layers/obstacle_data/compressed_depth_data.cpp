#include <compressed_depth_image_transport/codec.h>
#include <compressed_depth_image_transport/compression_common.h>
#include <cv_bridge/cv_bridge.h>
#include <gridmap/layers/obstacle_data/compressed_depth_data.h>
#include <gridmap/layers/obstacle_data/depth.h>
#include <image_transport/subscriber.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <chrono>
#include <cstdint>
#include <unordered_set>

PLUGINLIB_EXPORT_CLASS(gridmap::CompressedDepthData, gridmap::DataSource)

namespace gridmap
{

CompressedDepthData::CompressedDepthData()
    : TopicDataSource<sensor_msgs::CompressedImage>("depth/compressedDepth"), got_camera_info_(false),
      hit_probability_log_(0), miss_probability_log_(0), obstacle_height_(0), min_range_(0), max_range_(0)
{
}

void CompressedDepthData::onInitialize(const YAML::Node& parameters)
{
    miss_probability_log_ = logodds(parameters["miss_probability"].as<double>(0.4));
    obstacle_height_ = parameters["max_obstacle_height"].as<double>(0.10);
    min_range_ = parameters["min_range"].as<float>(0.15);
    max_range_ = parameters["max_range"].as<float>(1.5);
    image_mask_ = parameters["image_mask"].as<std::string>(std::string());
    camera_info_topic_ = parameters["camera_info_topic"].as<std::string>(std::string(name_ + "/camera_info"));

    if (!image_mask_.empty())
        cv_image_mask_ = std::make_unique<cv::Mat>(cv::imread(image_mask_, cv::IMREAD_GRAYSCALE));

    ros::NodeHandle g_nh;
    camera_info_sub_ = g_nh.subscribe<sensor_msgs::CameraInfo>(camera_info_topic_, 1000,
                                                               &CompressedDepthData::cameraInfoCallback, this);
}

void CompressedDepthData::onMapDataChanged()
{
}

CompressedDepthData::~CompressedDepthData()
{
}

bool CompressedDepthData::isDataOk() const
{
    return got_camera_info_ && TopicDataSource<sensor_msgs::CompressedImage>::isDataOk();
}

void CompressedDepthData::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    got_camera_info_ = true;
    camera_model_.fromCameraInfo(*msg);
}

bool CompressedDepthData::processData(const sensor_msgs::CompressedImage::ConstPtr& msg,
                                      const Eigen::Isometry2d& robot_pose, const Eigen::Isometry3d& sensor_transform)
{
    const Eigen::Isometry3f t_f = sensor_transform.cast<float>();

    const Eigen::Vector3d sensor_pt = sensor_transform.translation();
    const Eigen::Vector2d sensor_pt_2d(sensor_pt.x(), sensor_pt.y());
    const Eigen::Vector2i sensor_pt_map = map_data_->dimensions().getCellIndex(sensor_pt_2d);

    if (!got_camera_info_)
    {
        ROS_ERROR_STREAM("No camera info for: " << name());
        return false;
    }

    // Check sensor is on map
    if (sensor_pt_map.x() < 0 || sensor_pt_map.x() >= map_data_->dimensions().size().x() || sensor_pt_map.y() < 0 ||
        sensor_pt_map.y() >= map_data_->dimensions().size().y())
    {
        ROS_WARN_STREAM("Sensor is not on gridmap for: " << name());
        return false;
    }

    const sensor_msgs::Image::Ptr image = compressed_depth_image_transport::decodeCompressedDepthImage(*msg);
    const cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(image);

    if (cv_image_mask_)
        cv::bitwise_and(*cv_image_mask_, cv_image->image, cv_image->image);

    // add a 5% buffer
    const auto footprint = buildFootprintSet(map_data_->dimensions(), robot_pose, robot_footprint_, 1.05);

    {
        // cppcheck-suppress unreadVariable
        auto _lock = map_data_->getLock();
        std::lock_guard<std::mutex> lock(camera_info_mutex_);

        std::unordered_map<uint64_t, float> height_voxels;

        if (image->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
            projectDepth<uint16_t>(height_voxels, min_range_, max_range_, obstacle_height_, t_f, footprint,
                                   cv_image->image, camera_model_, map_data_->dimensions());
        else if (image->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
            projectDepth<float>(height_voxels, min_range_, max_range_, obstacle_height_, t_f, footprint,
                                cv_image->image, camera_model_, map_data_->dimensions());
        else
            ROS_ASSERT_MSG(false, "Unsupported depth image format");

        for (auto elem : height_voxels)
        {
            const double h = static_cast<double>(std::max(-0.1f, std::min(0.3f, elem.second)));
            const double log_odds = h * (-miss_probability_log_ / obstacle_height_) + miss_probability_log_;

            const Eigen::Array2i index = KeyToIndex(elem.first);
            if (map_data_->dimensions().contains(index))
                map_data_->update(index, log_odds);
        }
    }

    return true;
}

}  // namespace gridmap
