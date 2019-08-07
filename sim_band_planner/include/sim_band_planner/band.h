#ifndef FDP_LOCAL_PLANNER_PATH_H
#define FDP_LOCAL_PLANNER_PATH_H

#include <Eigen/Geometry>
#include <vector>

#include <sim_band_planner/spline/Spline.h>
#include <sim_band_planner/spline/SplineFitting.h>
#include <sim_band_planner/spline/SplineFwd.h>

#include <ros/assert.h>
#include <ros/console.h>

namespace sim_band_planner
{

struct ControlPoint
{
    explicit ControlPoint(const Eigen::Vector2d& offset) : offset(offset)
    {
    }
    Eigen::Vector2d offset;
    double distance_to_saddle = 0;
    double distance = 0;
    Eigen::Vector2f gradient = Eigen::Vector2f::Zero();
};

struct Node
{
    Node() = delete;

    Node(const Eigen::Isometry2d& pose, const std::vector<Eigen::Vector2d>& _radius_offsets)
        : pose(pose), closest_point(0)
    {
        ROS_ASSERT(!_radius_offsets.empty());
        for (const Eigen::Vector2d& offset : _radius_offsets)
        {
            control_points.push_back(ControlPoint(offset));
        }
    }

    Eigen::Isometry2d pose;
    std::size_t closest_point;
    std::vector<ControlPoint> control_points;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
};

struct Band
{
    std::vector<Node> nodes;
    std::vector<Eigen::Vector2d> radius_offsets;

    Band() = delete;
    explicit Band(const std::vector<Eigen::Vector2d>& _radius_offsets) : radius_offsets(_radius_offsets)
    {
        ROS_ASSERT(!_radius_offsets.empty());
    }
    Band(std::vector<Node>::const_iterator start, std::vector<Node>::const_iterator end,
         const std::vector<Eigen::Vector2d>& _radius_offsets)
        : nodes(start, end), radius_offsets(_radius_offsets)
    {
        ROS_ASSERT(!_radius_offsets.empty());
    }

    double length() const
    {
        double distance = 0.0;
        for (std::size_t i = 1; i < nodes.size(); ++i)
        {
            distance += (nodes[i - 1].pose.translation() - nodes[i].pose.translation()).norm();
        }
        return distance;
    }

    std::pair<std::size_t, double> closestSegment(const Eigen::Isometry2d& pose) const
    {
        std::vector<double> distances;
        std::transform(nodes.begin(), nodes.end(), std::back_inserter(distances),
                       [&pose](const Node& node) { return (node.pose.translation() - pose.translation()).norm(); });
        const auto it = std::min_element(distances.begin(), distances.end());
        const long dist = std::distance(distances.begin(), it);
        if (dist > 0 && dist < nodes.size() - 1)
        {
            const double prev_seg = (nodes[dist].pose.translation() - nodes[dist - 1].pose.translation()).norm();
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

    Eigen::Spline<double, 3> spline() const
    {
        ROS_ASSERT(nodes.size() > 2);

        Eigen::MatrixXd points(3, nodes.size());
        for (std::size_t i = 0; i < nodes.size(); ++i)
        {
            points(0, i) = nodes[i].pose.translation().x();
            points(1, i) = nodes[i].pose.translation().y();
            points(2, i) = Eigen::Rotation2Dd(nodes[i].pose.rotation()).angle();
        }

        const int degree = std::max(3, static_cast<int>(nodes.size()) - 1);
        const Eigen::Spline<double, 3> spline =
            Eigen::SplineFitting<Eigen::Spline<double, 3>>::Interpolate(points, degree);

        return spline;
    }
};
}

#endif
