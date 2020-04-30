#ifndef GRIDMAP_DEPTH_H
#define GRIDMAP_DEPTH_H

#include <gridmap/layers/obstacle_data/data_source.h>
#include <gridmap/operations/raytrace.h>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

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
                  const std::set<uint64_t>& footprint, const sensor_msgs::Image::ConstPtr& msg,
                  const image_geometry::PinholeCameraModel& camera_model, const MapDimensions& map_dimensions)
{
    // Use correct principal point from calibration
    const float center_x = static_cast<float>(camera_model.cx());
    const float center_y = static_cast<float>(camera_model.cy());

    const float unit_scaling = DepthTraits<T>::toMeters(T(1));
    const float constant_x = unit_scaling / static_cast<float>(camera_model.fx());
    const float constant_y = unit_scaling / static_cast<float>(camera_model.fy());

    const T* depth_row = reinterpret_cast<const T*>(&msg->data[0]);
    int row_step = msg->step / sizeof(T);
    for (int v = 0; v < (int)msg->height; ++v, depth_row += row_step)
    {
        for (int u = 0; u < (int)msg->width; ++u)
        {
            T d = depth_row[u];

            if (!DepthTraits<T>::valid(d))
                continue;

            const float depth = DepthTraits<T>::toMeters(d);

            if (depth < min_range)
                continue;

            const Eigen::Vector3f reading((u - center_x) * d * constant_x, (v - center_y) * d * constant_y, depth);

            ROS_ASSERT_MSG(reading.allFinite(), "%f %f %f", reading.x(), reading.y(), reading.z());

            const Eigen::Vector3f pt = sensor_transform * reading;
            const Eigen::Array2i pt_map = map_dimensions.getCellIndex(pt.head<2>().cast<double>());

            // Skip points greater than max_range unless the scan to the floor and help clear obstacles
            if (reading.norm() > max_range && pt.z() >= obstacle_height)
                continue;

            const auto key = IndexToKey(pt_map);

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

}  // namespace gridmap

#endif
