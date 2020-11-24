#ifndef NAVIGATION_INTERFACE_PATH_H
#define NAVIGATION_INTERFACE_PATH_H

#include <Eigen/Geometry>

#include <std_msgs/Header.h>

#include <algorithm>
#include <vector>

namespace navigation_interface
{

struct Path
{
    double cost;
    std::string id;
    std_msgs::Header header;
    std::vector<Eigen::Isometry2d> nodes;

    double length() const
    {
        double distance = 0.0;
        for (std::size_t i = 0; i < nodes.size() - 1; ++i)
        {
            distance += (nodes[i].translation() - nodes[i + 1].translation()).norm();
        }
        return distance;
    }

    std::pair<std::size_t, double> closestSegment(const Eigen::Isometry2d& pose) const
    {
        std::vector<double> distances;
        std::transform(
            nodes.begin(), nodes.end(), std::back_inserter(distances),
            [&pose](const Eigen::Isometry2d& node) { return (node.translation() - pose.translation()).norm(); });
        const auto it = std::min_element(distances.begin(), distances.end());
        const long dist = std::distance(distances.begin(), it);
        if (dist > 0 && dist < nodes.size() - 1)
        {
            const double prev_seg = (nodes[dist - 1].translation() - nodes[dist].translation()).norm();
            if (distances[dist - 1] < prev_seg)
            {
                return {dist - 1, distances[dist - 1]};
            }
            else
                return {dist, *it};
        }
        else
        {
            return {dist, *it};
        }
    }
};
};  // namespace navigation_interface

#endif
