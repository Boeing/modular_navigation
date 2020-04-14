#include <gridmap/robot_tracker.h>
#include <ros/assert.h>
#include <ros/console.h>

#include <algorithm>
#include <chrono>

namespace gridmap
{

namespace
{

Eigen::Isometry2d convert(const geometry_msgs::Pose& pose)
{
    const double yaw =
        std::atan2(2.0 * (pose.orientation.z * pose.orientation.w + pose.orientation.x * pose.orientation.y),
                   -1.0 + 2.0 * (pose.orientation.w * pose.orientation.w + pose.orientation.x * pose.orientation.x));
    return Eigen::Translation2d(pose.position.x, pose.position.y) * Eigen::Rotation2Dd(yaw);
}

Eigen::Isometry2d convert(const geometry_msgs::Transform& tr)
{
    const double yaw = std::atan2(2.0 * (tr.rotation.z * tr.rotation.w + tr.rotation.x * tr.rotation.y),
                                  -1.0 + 2.0 * (tr.rotation.w * tr.rotation.w + tr.rotation.x * tr.rotation.x));
    return Eigen::Translation2d(tr.translation.x, tr.translation.y) * Eigen::Rotation2Dd(yaw);
}

TimedOdom interpolate(const TimedOdom& start, const TimedOdom& end, const ros::Time& time)
{
    ROS_ASSERT(start.time.toSec() <= time.toSec());
    ROS_ASSERT(end.time.toSec() >= time.toSec());
    const double duration = end.time.toSec() - start.time.toSec();
    const double factor = (time.toSec() - start.time.toSec()) / duration;
    const Eigen::Vector2d origin =
        start.pose.translation() + (end.pose.translation() - start.pose.translation()) * factor;
    const Eigen::Rotation2Dd rotation =
        Eigen::Rotation2Dd(start.pose.linear()).slerp(factor, Eigen::Rotation2Dd(end.pose.linear()));
    Eigen::Isometry2d p(Eigen::Translation2d(origin) * rotation);
    return TimedOdom{time, p, end.velocity};
}

TimedOdom interpolate(const ros::Time& time, const boost::circular_buffer<TimedOdom>& odometry_data)
{
    TimedOdom odom;
    auto it = odometry_data.begin();
    while (it != odometry_data.end() && it->time.toSec() < time.toSec())
    {
        it++;
    }

    if (it == odometry_data.begin())
    {
        ROS_WARN_STREAM("No odometry data for time: " << time.toSec()
                                                      << " (earliest: " << odometry_data.front().time.toSec() << ")");
        odom.pose = it->pose;
        odom.velocity = it->velocity;
    }
    else if (it == odometry_data.end())
    {
        auto prev_it = it - 1;
        const double t_diff = time.toSec() - prev_it->time.toSec();

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
RobotState RobotTracker::waitForRobotState(const double timeout) const
{
    std::unique_lock<std::mutex> lock(mutex_);
    const auto t0 = std::chrono::steady_clock::now();
    if (conditional_.wait_for(lock, std::chrono::milliseconds(static_cast<long>(timeout))) == std::cv_status::timeout)
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
        return RobotState{TimedOdom{ros::Time::now(), Eigen::Isometry2d::Identity(), Eigen::Vector3d::Zero()},
                          localisation_.localised, localisation_.map_to_odom};
    else
        return RobotState{odometry_data_.back(), localisation_.localised, localisation_.map_to_odom};
}

RobotState RobotTracker::robotState(const ros::Time& time) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (odometry_data_.empty())
        return RobotState{TimedOdom{ros::Time::now(), Eigen::Isometry2d::Identity(), Eigen::Vector3d::Zero()},
                          localisation_.localised, localisation_.map_to_odom};

    // estimate odometry at the queried time
    const TimedOdom current_odom = interpolate(time, odometry_data_);

    return RobotState{current_odom, localisation_.localised, localisation_.map_to_odom};
}

// cppcheck-suppress unusedFunction
void RobotTracker::addOdometryData(const nav_msgs::Odometry& odometry_data)
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
void RobotTracker::addLocalisationData(const cartographer_ros_msgs::SystemState& localisation)
{
    std::lock_guard<std::mutex> lock(mutex_);
    localisation_.localised = localisation.localisation_status == cartographer_ros_msgs::SystemState::LOCALISED;
    localisation_.map_to_odom = convert(localisation.localisation.transform);
}

}  // namespace gridmap
