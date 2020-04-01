#ifndef FDP_LOCAL_PLANNER_PATH_H
#define FDP_LOCAL_PLANNER_PATH_H

#include <ros/assert.h>
#include <ros/console.h>
#include <sim_band_planner/spline/Spline.h>
#include <sim_band_planner/spline/SplineFitting.h>
#include <sim_band_planner/spline/SplineFwd.h>

#include <Eigen/Geometry>
#include <vector>

namespace sim_band_planner
{

template <typename NodeType>
std::size_t findClosest(const std::vector<NodeType>& nodes, const Eigen::Isometry2d& pose,
                        const Eigen::Vector3d& velocity)
{
    // Calcuate "distance" between current pose and each node where distance is a weighted sum of the linear distance
    // and the rotational difference. Alpha controls the weighting of the angular difference

    if (nodes.size() <= 1)
        return 0;

    const double dt = 1.0;
    Eigen::Isometry2d future_state = pose;
    future_state.pretranslate(dt * Eigen::Vector2d(velocity.topRows(2)));
    future_state.rotate(Eigen::Rotation2Dd(dt * velocity[2]));

    // First state is current robot pose, so we skip that
    std::vector<double> distances;
    std::transform(
        nodes.begin() + 1, nodes.end(), std::back_inserter(distances), [&future_state](const NodeType& state) {
            const double alpha = 0.1;
            const Eigen::Rotation2Dd rot(state.linear().inverse() * future_state.linear());
            return ((state.translation() - future_state.translation()).norm()) + alpha * std::abs(rot.smallestAngle());
        });
    auto it = std::min_element(distances.begin(), distances.end());
    const long dist = std::distance(distances.begin(), it);

    return dist;
}

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
        : nominal(pose), pose(pose), closest_point(0)
    {
        ROS_ASSERT(!_radius_offsets.empty());
        std::transform(_radius_offsets.begin(), _radius_offsets.end(), std::back_inserter(control_points),
                       [](const Eigen::Vector2d& offset) { return ControlPoint(offset); });
    }

    Eigen::Isometry2d nominal;
    Eigen::Isometry2d pose;

    std::size_t closest_point;

    // TODO should be refs
    std::vector<ControlPoint> control_points;

    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();

    Eigen::Isometry2d::ConstTranslationPart translation() const
    {
        return pose.translation();
    }
    Eigen::Isometry2d::ConstLinearPart linear() const
    {
        return pose.linear();
    }
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

    std::size_t closestSegment(const Eigen::Isometry2d& pose, const Eigen::Vector3d& velocity) const
    {
        return findClosest(nodes, pose, velocity);
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
}  // namespace sim_band_planner

#endif
