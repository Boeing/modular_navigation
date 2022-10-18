#ifndef FDP_LOCAL_PLANNER_PATH_H
#define FDP_LOCAL_PLANNER_PATH_H

#include <Eigen/Geometry>

//#include <ros/assert.h>
#include "rcpputils/asserts.hpp"

//#include <ros/console.h>
#include <sim_band_planner/spline/Spline.h>
#include <sim_band_planner/spline/SplineFitting.h>
#include <sim_band_planner/spline/SplineFwd.h>

#include <vector>

namespace sim_band_planner
{

// We can treat rotation like a Z axis, so we can calculate a 3D norm between any two poses
template <typename NodeType1, typename NodeType2>
double dist(const NodeType1& pose_1, const NodeType2& pose_2, const double a_weight)
{
    const double diff_x = pose_2.translation().x() - pose_1.translation().x();
    const double diff_y = pose_2.translation().y() - pose_1.translation().y();
    const double diff_a = Eigen::Rotation2Dd(pose_1.linear().inverse() * pose_2.linear()).smallestAngle();

    return (diff_x * diff_x) + (diff_y * diff_y) + a_weight * (diff_a * diff_a);
}

template <typename NodeType> std::size_t findClosest(const std::vector<NodeType>& nodes, const Eigen::Isometry2d& pose)
{
    if (nodes.size() <= 2)
        return nodes.size() - 1;

    std::vector<double> distances;
    std::transform(nodes.begin(), nodes.end(), std::back_inserter(distances),
                   [&pose](const NodeType& state) { return dist(state, pose, 0.1); });
    const auto closest_it = std::min_element(distances.begin(), distances.end());
    const std::size_t closest_i = std::distance(distances.begin(), closest_it);

    if (closest_i == 0 || closest_i == nodes.size() - 1)
    {
        return static_cast<std::size_t>(closest_i);
    }

    // Resolve ambiguity
    // Find midpoints  -----A------M1---closest_i---M2-------B------->
    Eigen::Isometry2d m_1;
    Eigen::Isometry2d m_2;
    {
        // Calculate pose of M1
        const Eigen::Vector2d translation_vec = nodes[closest_i].translation() - nodes[closest_i - 1].translation();
        const Eigen::Rotation2Dd rot(nodes[closest_i - 1].linear());
        const Eigen::Rotation2Dd target_rot(nodes[closest_i].linear());

        m_1 = Eigen::Translation2d(nodes[closest_i - 1].translation() + 0.99 * translation_vec) *
              rot.slerp(0.99, target_rot);
    }

    {
        // Calculate pose of M2
        const Eigen::Vector2d translation_vec = nodes[closest_i + 1].translation() - nodes[closest_i].translation();
        const Eigen::Rotation2Dd rot(nodes[closest_i].linear());
        const Eigen::Rotation2Dd target_rot(nodes[closest_i + 1].linear());

        m_2 =
            Eigen::Translation2d(nodes[closest_i].translation() + 0.01 * translation_vec) * rot.slerp(0.01, target_rot);
    }

    if (dist(m_1, pose, 0.1) < dist(m_2, pose, 0.1))
    {
        return static_cast<std::size_t>(closest_i - 1);
    }
    else
    {
        return static_cast<std::size_t>(closest_i);
    }
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
        rcpputils::assert_true(!_radius_offsets.empty());
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
        rcpputils::assert_true(!_radius_offsets.empty());
    }
    Band(std::vector<Node>::const_iterator start, std::vector<Node>::const_iterator end,
         const std::vector<Eigen::Vector2d>& _radius_offsets)
        : nodes(start, end), radius_offsets(_radius_offsets)
    {
        rcpputils::assert_true(!_radius_offsets.empty());
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

    std::size_t closestSegment(const Eigen::Isometry2d& pose) const
    {
        return findClosest(nodes, pose);
    }

    Eigen::Spline<double, 3> spline() const
    {
        rcpputils::assert_true(nodes.size() > 2);

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
