#include <sim_band_planner/simulate.h>

#include <sstream>
#include <vector>

#include <ros/assert.h>

namespace sim_band_planner
{

double simulate(Band& path, const DistanceField& distance_field, const int num_iterations, const double min_overlap,
                const double min_distance, const double internal_force_gain, const double external_force_gain,
                const double rotation_factor, const double velocity_decay, const double alpha_start,
                const double alpha_decay, const double max_distance, const int max_nodes)
{
    updateDistances(path, distance_field, max_distance);
    refine(path, distance_field, min_distance, max_distance, min_overlap, max_nodes);

    const double dt = 1.0;
    double alpha = alpha_start;

    for (int it = 0; it < num_iterations; it++)
    {
        updateDistances(path, distance_field, max_distance);

        // f = ...
        std::vector<Eigen::Vector3d> forces(path.nodes.size(), Eigen::Vector3d::Zero());
        for (std::size_t i = 1; i < path.nodes.size() - 1; ++i)
        {
            forces[i] = force(path.nodes[i - 1], path.nodes[i], path.nodes[i + 1], internal_force_gain,
                              external_force_gain, rotation_factor, max_distance);
        }

        // a = f / m
        const std::vector<Eigen::Vector3d> acc = forces;

        // v = v + a * dt;
        for (std::size_t i = 1; i < path.nodes.size() - 1; ++i)
        {
            ROS_ASSERT(acc[i].allFinite());
            path.nodes[i].velocity += alpha * acc[i] * dt;
            path.nodes[i].velocity *= velocity_decay;
        }

        // s = vt
        for (std::size_t i = 1; i < path.nodes.size() - 1; ++i)
        {
            path.nodes[i].pose.pretranslate(dt * Eigen::Vector2d(path.nodes[i].velocity.topRows(2)));
            path.nodes[i].pose.rotate(Eigen::Rotation2Dd(dt * path.nodes[i].velocity[2]));
        }

        refine(path, distance_field, min_distance, max_distance, min_overlap, max_nodes);

        alpha -= alpha * alpha_decay;
    }

    updateDistances(path, distance_field, max_distance);

    return alpha;
}

void updateDistances(Node& node, const DistanceField& distance_field, const double max_distance)
{
    double min_distance = std::numeric_limits<double>::max();
    node.closest_point = 0;

    for (std::size_t i = 0; i < node.control_points.size(); ++i)
    {
        ControlPoint& control_point = node.control_points[i];
        const auto position = node.pose.translation() + node.pose.rotation() * control_point.offset;

        unsigned int mx;
        unsigned int my;
        if (distance_field.worldToMap(position.x(), position.y(), mx, my))
        {
            control_point.distance_to_saddle = distance_field.distanceToSaddle(mx, my);
            control_point.distance = distance_field.distance(mx, my) * distance_field.resolution;
            control_point.gradient = control_point.distance > 0 ? distance_field.positiveGradient(mx, my)
                                                                : distance_field.negativeGradient(mx, my);

            if (control_point.distance > 0)
            {
                control_point.distance = std::min(max_distance, control_point.distance);
            }
        }
        else
        {
            control_point.distance_to_saddle = 0;
            control_point.distance = 0;
            control_point.gradient = Eigen::Vector2f::Zero();
        }

        if (control_point.distance < min_distance)
        {
            node.closest_point = i;
            min_distance = control_point.distance;
        }
    }
}

void updateDistances(Band& path, const DistanceField& distance_field, const double max_distance)
{
    for (std::size_t i = 0; i < path.nodes.size(); ++i)
    {
        updateDistances(path.nodes[i], distance_field, max_distance);
    }
}

void refine(Band& path, const DistanceField& distance_field, const double min_distance, const double max_distance,
            const double min_overlap, const int max_nodes)
{
    //
    // add new nodes
    //
    std::vector<Node>::iterator iter = path.nodes.begin();
    while (std::distance(path.nodes.begin(), iter) < max_nodes)
    {
        const auto next = iter + 1;

        if (next == path.nodes.end())
            break;

        const double iter_exp = std::max(min_distance, std::abs(iter->control_points[iter->closest_point].distance));
        const double next_exp = std::max(min_distance, std::abs(next->control_points[next->closest_point].distance));

        const double min_radius = std::min(iter_exp, next_exp);
        const double distance_to_next = (next->pose.translation() - iter->pose.translation()).norm();
        const double combined_radius = iter_exp + next_exp;
        const bool overlaping_next = distance_to_next < combined_radius - min_radius * min_overlap;

        if (!overlaping_next)
        {
            const double fraction = iter_exp / distance_to_next;

            Node new_node(
                Eigen::Translation2d(iter->pose.translation() +
                                     fraction * (next->pose.translation() - iter->pose.translation())) *
                    Eigen::Rotation2Dd(iter->pose.linear()).slerp(fraction, Eigen::Rotation2Dd(next->pose.linear())),
                path.radius_offsets);

            updateDistances(new_node, distance_field, max_distance);

            // insert to band
            iter = path.nodes.insert(next, new_node);
        }
        else
        {
            ++iter;
        }
    }

    //
    // delete overlapping nodes
    //
    iter = path.nodes.begin();
    while (true)
    {
        const auto next = iter + 1;

        if (next == path.nodes.end())
            break;

        const double iter_exp = std::max(min_distance, std::abs(iter->control_points[iter->closest_point].distance));
        const double next_exp = std::max(min_distance, std::abs(next->control_points[next->closest_point].distance));

        const double min_radius = std::min(iter_exp, next_exp);
        const double distance_to_next = (iter->pose.translation() - next->pose.translation()).norm();
        const double combined_radius = iter_exp + next_exp;
        const bool overlaping_next = distance_to_next < combined_radius - 2 * min_radius * min_overlap;

        if (!overlaping_next)
        {
            ++iter;
        }
        else
        {
            const auto next_next = next + 1;

            if (next_next == path.nodes.end())
                break;

            const double next_next_exp =
                std::max(min_distance, std::abs(next_next->control_points[next_next->closest_point].distance));

            const double min_radius_next = std::min(iter_exp, next_next_exp);
            const double distance_to_next_next = (next_next->pose.translation() - iter->pose.translation()).norm();
            const double combined_radius_nex = iter_exp + next_next_exp;
            const bool overlaping_next_next =
                distance_to_next_next < combined_radius_nex - min_radius_next * min_overlap;

            // check if a shortcut path is possible to the next next node
            if (overlaping_next_next)
            {
                // remove the next node
                iter = path.nodes.erase(next);
                --iter;
            }
            else
            {
                // we can't shortcut so we just have to deal
                ++iter;
            }
        }
    }

    //
    // move in-cost nodes to the edge
    //
    for (std::size_t i = 1; i < path.nodes.size() - 1; ++i)
    {
        const ControlPoint& cp = path.nodes[i].control_points[path.nodes[i].closest_point];
        if (cp.distance < 0)
        {
            path.nodes[i].pose.pretranslate(
                Eigen::Vector2d(cp.gradient[0] * (-cp.distance + 2 * distance_field.resolution),
                                cp.gradient[1] * (-cp.distance + 2 * distance_field.resolution)));
        }
    }
}

Eigen::Vector3d force(const Node& prev, const Node& curr, const Node& next, const double internal_force_gain,
                      const double external_force_gain, const double rotation_factor, const double max_distance)
{
    const auto internal_force = internalForce(prev, curr, next, internal_force_gain, rotation_factor, max_distance);
    const auto external_force = externalForce(curr, external_force_gain, max_distance);
    return internal_force + external_force;
}

Eigen::Vector3d internalForce(const Node& prev, const Node& curr, const Node& next, const double internal_force_gain,
                              const double rotation_factor, const double max_distance)
{
    const Eigen::Vector2d d_1 = prev.pose.translation() - curr.pose.translation();
    const Eigen::Vector2d d_2 = next.pose.translation() - curr.pose.translation();

    const double d_1_norm = d_1.norm();
    const double d_2_norm = d_2.norm();

    const Eigen::Vector2d d_1_normalized = d_1 / d_1_norm;
    const Eigen::Vector2d d_2_normalized = d_2 / d_2_norm;

    const Eigen::Rotation2Dd rot_1 = Eigen::Rotation2Dd(prev.pose.rotation().inverse() * curr.pose.rotation());
    const Eigen::Rotation2Dd rot_2 = Eigen::Rotation2Dd(next.pose.rotation().inverse() * curr.pose.rotation());

    Eigen::Vector3d _force = Eigen::Vector3d::Zero();

    const ControlPoint& prev_min = prev.control_points[prev.closest_point];
    const ControlPoint& curr_min = curr.control_points[curr.closest_point];
    const ControlPoint& next_min = next.control_points[next.closest_point];

    const double dis_1 = curr_min.distance + prev_min.distance;
    const double dis_2 = curr_min.distance + next_min.distance;
    const double gap_1 = dis_1 - d_1.norm();
    const double gap_2 = dis_2 - d_2.norm();

    // straightening force
    if (d_1_norm > std::numeric_limits<double>::epsilon() && d_2_norm > std::numeric_limits<double>::epsilon())
    {
        _force.topRows(2) = internal_force_gain * (d_1_normalized + d_2_normalized);
    }

    if (gap_1 > 0 && d_1_norm > std::numeric_limits<double>::epsilon())
    {
        const double exp_1 = d_1.norm() < dis_1 ? 0.001 * std::exp(1.0 * (dis_1 - d_1.norm())) : 0.0;

        // repelling force
        _force.topRows(2) += -exp_1 * d_1_normalized;

        // attracting force
        _force.topRows(2) += 0.004 * (d_1 * gap_1).normalized();
    }

    if (gap_2 > 0 && d_2_norm > std::numeric_limits<double>::epsilon())
    {
        const double exp_2 = d_2.norm() < dis_2 ? 0.001 * std::exp(1.0 * (dis_2 - d_2.norm())) : 0.0;

        // repelling force
        _force.topRows(2) += -exp_2 * d_2_normalized;

        // attracting force
        _force.topRows(2) += 0.004 * (d_2 * gap_2).normalized();
    }

    // rotation equilising force
    _force[2] = -10 * internal_force_gain * (rot_1.smallestAngle() + rot_2.smallestAngle()) / 2.0;

    // when close to objects rotate to avoid them otherwise rotate to face forward
    if (curr_min.distance > 0.2)
    {
        // face forward (or backwards) force
        if (d_2_norm > std::numeric_limits<double>::epsilon())
        {
            const Eigen::Vector2d pose_dir = curr.pose.linear() * Eigen::Vector2d::UnitX();
            const double dot = pose_dir.dot(d_2_normalized);
            const double det = pose_dir.x() * d_2_normalized.y() - pose_dir.y() * d_2_normalized.x();
            double fwd_angle = std::atan2(det, dot);
            if (fwd_angle > M_PI / 2.0)
                fwd_angle = M_PI - fwd_angle;
            else if (fwd_angle < -M_PI / 2.0)
                fwd_angle = -M_PI - fwd_angle;
            _force[2] += 10 * rotation_factor * internal_force_gain * fwd_angle;
        }
    }
    else
    {
        // control point torque
        double cp_torque = 0;
        const ControlPoint& cp = curr.control_points[curr.closest_point];
        if (cp.distance < max_distance)
        {
            double scale = 0.1 * std::min(1.0, curr_min.distance_to_saddle);
            if (cp.distance < 0)
                scale *= -1;

            const Eigen::Vector3d rotation_force =
                scale * Eigen::Vector3d(static_cast<double>(cp.gradient.x()), static_cast<double>(cp.gradient.y()), 0);
            const auto rot_offset = curr.pose.rotation() * cp.offset;
            const Eigen::Vector3d offset_3d(rot_offset.x(), rot_offset.y(), 0);

            const auto tau = offset_3d.cross(rotation_force);
            cp_torque = -tau.z();
        }
        _force[2] += 10 * internal_force_gain * cp_torque;
    }

    return _force;
}

Eigen::Vector3d externalForce(const Node& curr, const double external_force_gain, const double max_distance)
{
    const ControlPoint& curr_min = curr.control_points[curr.closest_point];
    Eigen::Vector3d _force;
    const double avoid_distance = 0.06;

    double decay;
    if (curr_min.distance <= avoid_distance)
    {
        decay = 1.0;
    }
    else if (curr_min.distance > avoid_distance && curr_min.distance < max_distance)
    {
        decay = std::exp(-curr_min.distance - avoid_distance);
    }
    else
    {
        decay = 0.0;
    }
    decay *= std::min(1.0, std::max(0.1, curr_min.distance_to_saddle));

    _force[0] = -external_force_gain * curr_min.gradient.x() * decay;
    _force[1] = -external_force_gain * curr_min.gradient.y() * decay;
    _force[2] = 0.0;

    return _force;
}
}  // namespace sim_band_planner
