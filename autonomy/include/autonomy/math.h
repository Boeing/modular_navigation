#ifndef AUTONOMY_MATH
#define AUTONOMY_MATH

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>

namespace autonomy
{

inline Eigen::Isometry2d convert(const geometry_msgs::msg::Pose& pose)
{
    const double yaw =
        std::atan2(2.0 * (pose.orientation.z * pose.orientation.w + pose.orientation.x * pose.orientation.y),
                   -1.0 + 2.0 * (pose.orientation.w * pose.orientation.w + pose.orientation.x * pose.orientation.x));
    return Eigen::Translation2d(pose.position.x, pose.position.y) * Eigen::Rotation2Dd(yaw);
}

inline Eigen::Isometry2d convert(const geometry_msgs::msg::Transform& tr)
{
    const double yaw = std::atan2(2.0 * (tr.rotation.z * tr.rotation.w + tr.rotation.x * tr.rotation.y),
                                  -1.0 + 2.0 * (tr.rotation.w * tr.rotation.w + tr.rotation.x * tr.rotation.x));
    return Eigen::Translation2d(tr.translation.x, tr.translation.y) * Eigen::Rotation2Dd(yaw);
}

}  // namespace autonomy

#endif
