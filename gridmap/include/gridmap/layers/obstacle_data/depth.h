#ifndef GRIDMAP_DEPTH_H
#define GRIDMAP_DEPTH_H

#include <gridmap/layers/obstacle_data/data_source.h>
#include <gridmap/operations/raytrace.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core/core.hpp>
#include "rclcpp/rclcpp.hpp"
//#include <rospack/rospack.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rcpputils/asserts.hpp"

#include <unordered_map>

namespace gridmap
{

// Encapsulate differences between processing float and uint16_t depths
template <typename T> struct DepthTraits
{
};

template <> struct DepthTraits<uint16_t>
{
    static inline bool valid(uint16_t depth)
    {
        return depth != 0;
    }
    static inline float toMeters(uint16_t depth)
    {
        return depth * 0.001f;
    }  // originally mm
};

template <> struct DepthTraits<float>
{
    static inline bool valid(float depth)
    {
        return std::isfinite(depth);
    }
    static inline float toMeters(float depth)
    {
        return depth;
    }
};

template <typename T>
void projectDepth(std::unordered_map<uint64_t, float>& height_voxels, const float min_range, const float max_range,
                  const float obstacle_height, const Eigen::Isometry3f& sensor_transform,
                  const std::set<uint64_t>& footprint, const cv::Mat& msg,
                  const image_geometry::PinholeCameraModel& camera_model, const MapDimensions& map_dimensions)
{
    // Use correct principal point from calibration
    const float center_x = static_cast<float>(camera_model.cx());
    const float center_y = static_cast<float>(camera_model.cy());

    const float unit_scaling = DepthTraits<T>::toMeters(T(1));
    const float constant_x = unit_scaling / static_cast<float>(camera_model.fx());
    const float constant_y = unit_scaling / static_cast<float>(camera_model.fy());

    for (int v = 0; v < msg.rows; ++v)
    {
        for (int u = 0; u < msg.cols; ++u)
        {
            const T d = msg.at<T>(v, u);

            if (!DepthTraits<T>::valid(d))
                continue;

            const float depth = DepthTraits<T>::toMeters(d);

            if (depth < min_range)
                continue;

            const Eigen::Vector3f reading((u - center_x) * d * constant_x, (v - center_y) * d * constant_y, depth);

            //ROS_ASSERT_MSG(reading.allFinite(), "%f %f %f", reading.x(), reading.y(), reading.z());
            std::string text = "";
            text += std::to_string(reading.x()); // TODO use streams cmon
            text += std::to_string(reading.y());
            text += std::to_string(reading.z());
            rcpputils::assert_true(reading.allFinite(), text);

            const Eigen::Vector3f pt = sensor_transform * reading;
            const Eigen::Array2i pt_map = map_dimensions.getCellIndex(pt.head<2>().cast<double>());

            // Skip points greater than max_range unless the scan to the floor and help clear obstacles
            if (reading.norm() > max_range && pt.z() >= obstacle_height)
                continue;

            const auto key = IndexToKey(pt_map);

            // Check if point is inside footprint
            // If inside footprint then we need to discount points on the robot
            // A hack at the moment is to allow points above 400mm within the robot footprint
            // This is not ideal and perhaps we need a second footprint to cover the cabinet
            if (footprint.count(key) > 0)
            {
                if (pt.z() < 0.40f)
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

template <typename T> void maskImage(cv::Mat& image, const cv::Mat& mask)
{
    //ROS_ASSERT_MSG(image.size == mask.size, "Image and mask are not the same size.");
    rcpputils::assert_true(image.size == mask.size, "Image and mask are not the same size.");

    for (unsigned int i = 0; i < image.total(); ++i)
    {
        if (!mask.at<uint8_t>(i))
        {
            image.at<T>(i) = 0;
        }
    }
}

inline const cv_bridge::CvImageConstPtr getImage(const sensor_msgs::msg::Image::ConstPtr& image,
                                                 const std::unique_ptr<cv::Mat>& cv_image_mask)
{
    if (cv_image_mask)
    {
        const cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image);
        if (image->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
            maskImage<uint16_t>(cv_image->image, *cv_image_mask);
        else if (image->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
            maskImage<float>(cv_image->image, *cv_image_mask);
        return cv_image;
    }
    else
    {
        const cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(image);
        return cv_image;
    }
}

inline std::string getPackageUriPath(const std::string& package_uri)
{
    //TODO, CHECK THESE PATHS ARE WHAT THEY SHOULD 
    // https://docs.ros2.org/latest/api/ament_index_cpp/index.html
    rcpputils::assert_true(!package_uri.empty());

    const std::string prefix = "package://";

    std::string path = package_uri;
    path.erase(0, prefix.length());
    const std::string package_name = path.substr(0, path.find("/"));
    path.erase(0, package_name.length());

    std::string package_share_dir = ament_index_cpp::get_package_share_directory(package_name);
    
    std::string share_dir = "/share";

    std::string::size_type i = package_share_dir.find(share_dir);

    if (i != std::string::npos)
        package_share_dir.erase(i, share_dir.length());

    std::string package_dir = package_share_dir;
    
    //rospack::Rospack rospack;
    //std::vector<std::string> search_path;
    //if (!rospack.getSearchPathFromEnv(search_path))
    //    return std::string();
    //rospack.crawl(search_path, true);
    //if (!rospack.find(package_name, package_dir))
    //    throw std::runtime_error("Unable to find package: " + package_name);
    
    //ROS_ASSERT_MSG(package_uri.rfind(prefix, 0) == 0, "URI does not begin with package://");
    rcpputils::assert_true(package_uri.rfind(prefix, 0) == 0, "URI does not begin with package://");

    return (package_dir + path);
}

}  // namespace gridmap

#endif
