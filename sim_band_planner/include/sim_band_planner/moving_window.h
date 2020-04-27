#ifndef SIM_BAND_PLANNER_MOVING_WINDOW_H
#define SIM_BAND_PLANNER_MOVING_WINDOW_H

#include <navigation_interface/types/path.h>
#include <sim_band_planner/band.h>

#include <Eigen/Geometry>
#include <vector>

namespace sim_band_planner
{

navigation_interface::Path filterDuplicates(const navigation_interface::Path& path)
{
    ROS_ASSERT(!path.nodes.empty());

    navigation_interface::Path filtered;
    filtered.id = path.id;
    filtered.cost = path.cost;
    filtered.header = path.header;

    filtered.nodes.push_back(path.nodes.front());
    for (std::size_t i = 1; i < path.nodes.size(); ++i)
    {
        const auto prev = filtered.nodes.back();
        const auto p = path.nodes[i];
        const auto delta = prev.inverse() * p;
        if ((delta.translation().norm() > 0.01))
            filtered.nodes.push_back(p);
    }
    filtered.nodes.push_back(path.nodes.back());
    return filtered;
}

navigation_interface::Path interpolate(const navigation_interface::Path& path)
{
    ROS_ASSERT(!path.nodes.empty());

    if (path.nodes.size() == 1)
        return path;

    const double linear_step = 0.04;
    const double angular_step = 0.2;

    navigation_interface::Path interpolated;
    interpolated.id = path.id;
    interpolated.cost = path.cost;
    interpolated.header = path.header;

    for (std::size_t i = 0; i < path.nodes.size() - 1; ++i)
    {
        interpolated.nodes.push_back(path.nodes[i]);

        const Eigen::Vector2d linear_dir = path.nodes[i + 1].translation() - path.nodes[i].translation();
        const double linear_dist = linear_dir.norm();
        const Eigen::Rotation2Dd rot(path.nodes[i].linear());
        const Eigen::Rotation2Dd future_rot(path.nodes[i + 1].linear());
        const Eigen::Rotation2Dd rot_dir(future_rot.inverse() * rot);
        const double rot_dist = std::abs(rot_dir.smallestAngle());

        const std::size_t steps =
            static_cast<std::size_t>(std::max((linear_dist / linear_step), (rot_dist / angular_step)));
        for (std::size_t j = 1; j < steps; j++)
        {
            const double fraction = static_cast<double>(j + 1) / static_cast<double>(steps);
            const Eigen::Isometry2d pose = Eigen::Translation2d(path.nodes[i].translation() + fraction * linear_dir) *
                                           rot.slerp(fraction, future_rot);
            interpolated.nodes.push_back(pose);
        }
    }
    if (path.nodes.size() > 1)
        interpolated.nodes.push_back(path.nodes.back());

    ROS_ASSERT(!interpolated.nodes.empty());

    return interpolated;
}

struct MovingWindow
{
    const navigation_interface::Path nominal_path;

    std::size_t end_i;
    Band window;

    MovingWindow() = delete;

    MovingWindow(const navigation_interface::Path& _path, const std::vector<Eigen::Vector2d>& _radius_offsets)
        : nominal_path(interpolate(filterDuplicates(_path))), end_i(0), window(_radius_offsets)
    {
        ROS_ASSERT(!nominal_path.nodes.empty());
    }

    bool atEnd() const
    {
        return end_i == nominal_path.nodes.size();
    }

    void updateWindow(const Eigen::Isometry2d& pose, const Eigen::Vector3d& velocity, const double max_length)
    {
        if (!window.nodes.empty())
        {
            // trim the window
            const std::size_t closest = window.closestSegment(pose, velocity);
            window.nodes.erase(window.nodes.begin(), window.nodes.begin() + static_cast<int>(closest));
        }
        else
        {
            // start appending at the closest robot pose
            const std::size_t closest = findClosest(nominal_path.nodes, pose, velocity);
            end_i = closest;
            ROS_ASSERT(closest < nominal_path.nodes.size());
        }

        double distance = window.length();
        const std::size_t start_append = end_i;
        std::size_t end_append = end_i;
        while (distance < max_length && end_append != nominal_path.nodes.size())
        {
            if (end_append != 0)
                distance += ((nominal_path.nodes[end_append - 1]).translation() -
                             (nominal_path.nodes[end_append]).translation())
                                .norm();
            ++end_append;
        }

        end_i = end_append;

        for (std::size_t i = start_append; i < end_append; ++i)
        {
            window.nodes.push_back(Node(nominal_path.nodes[i], window.radius_offsets));
        }
    }
};
}  // namespace sim_band_planner

#endif
