#include <gridmap/robot_tracker.h>

#include "rcpputils/asserts.hpp"
//#include <ros/console.h>

#include <algorithm>
#include <chrono>

// For logging reasons
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/asserts.hpp"

namespace gridmap
{

namespace
{

Eigen::Isometry2d convert(const geometry_msgs::msg::Pose& pose)
{
    const double yaw =
        std::atan2(2.0 * (pose.orientation.z * pose.orientation.w + pose.orientation.x * pose.orientation.y),
                   -1.0 + 2.0 * (pose.orientation.w * pose.orientation.w + pose.orientation.x * pose.orientation.x));
    return Eigen::Translation2d(pose.position.x, pose.position.y) * Eigen::Rotation2Dd(yaw);
}

Eigen::Isometry2d convert(const geometry_msgs::msg::Transform& tr)
{
    const double yaw = std::atan2(2.0 * (tr.rotation.z * tr.rotation.w + tr.rotation.x * tr.rotation.y),
                                  -1.0 + 2.0 * (tr.rotation.w * tr.rotation.w + tr.rotation.x * tr.rotation.x));
    return Eigen::Translation2d(tr.translation.x, tr.translation.y) * Eigen::Rotation2Dd(yaw);
}

TimedOdom interpolate(const TimedOdom& start, const TimedOdom& end, const rclcpp::Time& time)
{
    rcpputils::assert_true(start.time.seconds() <= time.seconds());
    rcpputils::assert_true(end.time.seconds() >= time.seconds());
    const double duration = end.time.seconds() - start.time.seconds();
    const double factor = (time.seconds() - start.time.seconds()) / duration;
    const Eigen::Vector2d origin =
        start.pose.translation() + (end.pose.translation() - start.pose.translation()) * factor;
    const Eigen::Rotation2Dd rotation =
        Eigen::Rotation2Dd(start.pose.linear()).slerp(factor, Eigen::Rotation2Dd(end.pose.linear()));
    Eigen::Isometry2d p(Eigen::Translation2d(origin) * rotation);
    return TimedOdom{time, p, end.velocity};
}

TimedOdom interpolate(const rclcpp::Time& time, const boost::circular_buffer<TimedOdom>& odometry_data)
{
    TimedOdom odom;
    auto it = odometry_data.begin();
    while (it != odometry_data.end() && it->time.seconds() < time.seconds())
    {
        it++;
    }

    if (it == odometry_data.begin())
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(""), "No odometry data for time: " << time.seconds() << " (earliest: "
                                                                                 << odometry_data.front().time.seconds()
                                                                                 << ")");
        odom.pose = it->pose;
        odom.velocity = it->velocity;
    }
    else if (it == odometry_data.end())
    {
        auto prev_it = it - 1;
        const double t_diff = time.seconds() - prev_it->time.seconds();

        odom.time = time;
        odom.pose = prev_it->pose;
        odom.pose.pretranslate(t_diff * Eigen::Vector2d(prev_it->velocity.topRows(2)));
        odom.pose.rotate(Eigen::Rotation2Dd(t_diff * prev_it->velocity[2]));
    }
    else
    {
        auto prev_it = it - 1;
        odom = interpolate(*prev_it, *it, time);
    }
    return odom;
};

}  // namespace

RobotTracker::RobotTracker() : localisation_{false, Eigen::Isometry2d::Identity()}, odometry_data_(1000)
{
}

// cppcheck-suppress unusedFunction
RobotState RobotTracker::waitForRobotState(const double timeout_ms) const
{
    std::unique_lock<std::mutex> lock(mutex_);
    const auto t0 = std::chrono::steady_clock::now();
    if (conditional_.wait_for(lock, std::chrono::milliseconds(static_cast<long>(timeout_ms))) ==
        std::cv_status::timeout)
    {
        const double wait_time =
            std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count();
        throw std::runtime_error("Did not receive an odom message at the desired frequency: waited: " +
                                 std::to_string(wait_time));
    }
    return RobotState{odometry_data_.back(), localisation_.localised, localisation_.map_to_odom};
}

RobotState RobotTracker::robotState() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (odometry_data_.empty())
        return RobotState{
            TimedOdom{rclcpp::Clock(RCL_ROS_TIME).now(), Eigen::Isometry2d::Identity(), Eigen::Vector3d::Zero()},
            localisation_.localised, localisation_.map_to_odom};
    else
        return RobotState{odometry_data_.back(), localisation_.localised, localisation_.map_to_odom};
}

RobotState RobotTracker::robotState(const rclcpp::Time& time) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (odometry_data_.empty())
        return RobotState{
            TimedOdom{rclcpp::Clock(RCL_ROS_TIME).now(), Eigen::Isometry2d::Identity(), Eigen::Vector3d::Zero()},
            localisation_.localised, localisation_.map_to_odom};

    // estimate odometry at the queried time
    const TimedOdom current_odom = interpolate(time, odometry_data_);

    return RobotState{current_odom, localisation_.localised, localisation_.map_to_odom};
}

// cppcheck-suppress unusedFunction
void RobotTracker::addOdometryData(const nav_msgs::msg::Odometry& odometry_data)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        TimedOdom odom{odometry_data.header.stamp, convert(odometry_data.pose.pose),
                       Eigen::Vector3d(odometry_data.twist.twist.linear.x, odometry_data.twist.twist.linear.y,
                                       odometry_data.twist.twist.angular.z)};
        odometry_data_.push_back(odom);
    }
    conditional_.notify_all();
}

// cppcheck-suppress unusedFunction
void RobotTracker::addLocalisationData(const cartographer_ros_msgs::msg::SystemState& localisation)
{
    std::lock_guard<std::mutex> lock(mutex_);
    localisation_.localised = localisation.localisation_status == cartographer_ros_msgs::msg::SystemState::LOCALISED ||
                              localisation.localisation_status == cartographer_ros_msgs::msg::SystemState::PAUSED;
    localisation_.map_to_odom = convert(localisation.localisation.transform);
}

}  // namespace gridmap
