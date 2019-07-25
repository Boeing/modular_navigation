#ifndef FDP_LOCAL_PLANNER_PATH_H
#define FDP_LOCAL_PLANNER_PATH_H

#include <Eigen/Geometry>
#include <vector>

#include <sim_band_planner/spline/Spline.h>
#include <sim_band_planner/spline/SplineFitting.h>
#include <sim_band_planner/spline/SplineFwd.h>

#include <ros/console.h>

namespace sim_band_planner
{

struct ControlPoint
{
    ControlPoint(const Eigen::Vector2d& offset) : offset(offset) {}
    Eigen::Vector2d offset;
    double distance_to_saddle = 0;
    double distance = 0;
    Eigen::Vector2f gradient = Eigen::Vector2f::Zero();
};

struct Node
{
    Node() = delete;

    Node(const Eigen::Isometry2d& pose)
        : pose(pose),
          control_points({
                         ControlPoint({-0.268, 0.000}),
                         ControlPoint({ 0.268, 0.000}),
                         ControlPoint({ 0.265,-0.185}),
                         ControlPoint({ 0.077,-0.185}),
                         ControlPoint({-0.077,-0.185}),
                         ControlPoint({-0.265,-0.185}),
                         ControlPoint({ 0.265, 0.185}),
                         ControlPoint({-0.265, 0.185}),
                         ControlPoint({-0.077, 0.185}),
                         ControlPoint({ 0.077, 0.185}),
                        })
    {
    }

    Eigen::Isometry2d pose;

//    double min_distance_to_saddle = 0;
//    double min_distance = 0;
    //    Eigen::Vector2f gradient = Eigen::Vector2f::Zero();

    std::size_t closest_point;

    std::vector<ControlPoint> control_points;

    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
};

struct Band
{
    std::vector<Node> nodes;

    Band() = default;
    Band(std::vector<Node>::const_iterator start, std::vector<Node>::const_iterator end) : nodes(start, end)
    {
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
        assert(nodes.size() > 2);

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
