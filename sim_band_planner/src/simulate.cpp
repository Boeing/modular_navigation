#include <sim_band_planner/simulate.h>

#include <sstream>
#include <vector>

namespace sim_band_planner
{

void simulate(Band& path, const DistanceField& distance_field, const int num_iterations, const double min_overlap,
              const double min_distance, const double internal_force_gain, const double external_force_gain,
              const double rotation_factor, const double velocity_decay, const double alpha_start,
              const double alpha_decay, const double max_distance)
{
    updateDistances(path, distance_field, max_distance);
    refine(path, distance_field, min_distance, min_overlap);

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
                              external_force_gain, rotation_factor);
        }

        // a = f / m
        const std::vector<Eigen::Vector3d> acc = forces;

        // v = v + a * dt;
        for (std::size_t i = 1; i < path.nodes.size() - 1; ++i)
        {
            path.nodes[i].velocity += alpha * acc[i] * dt;
            path.nodes[i].velocity *= velocity_decay;
        }

        // s = vt
        for (std::size_t i = 1; i < path.nodes.size() - 1; ++i)
        {
            path.nodes[i].pose.pretranslate(dt * Eigen::Vector2d(path.nodes[i].velocity.topRows(2)));
            path.nodes[i].pose.rotate(Eigen::Rotation2Dd(dt * path.nodes[i].velocity[2]));
        }

        refine(path, distance_field, min_distance, min_overlap);

        alpha -= alpha * alpha_decay;
        ROS_INFO_STREAM("it: " << it << " alpha: " << alpha);
    }
}

void updateDistances(Band& path, const DistanceField& distance_field, const double max_distance)
{
    for (std::size_t i = 0; i < path.nodes.size(); ++i)
    {
        unsigned int mx;
        unsigned int my;
        if (distance_field.worldToMap(path.nodes[i].pose.translation().x(), path.nodes[i].pose.translation().y(), mx,
                                      my))
        {
            path.nodes[i].distance_to_saddle = distance_field.distanceToSaddle(mx, my);
            path.nodes[i].distance = distance_field.distance(mx, my) * distance_field.resolution;
            path.nodes[i].gradient = path.nodes[i].distance > 0 ? distance_field.positiveGradient(mx, my)
                                                                : distance_field.negativeGradient(mx, my);

            if (path.nodes[i].distance > 0)
            {
                path.nodes[i].distance = std::min(max_distance, path.nodes[i].distance);
            }
        }
        else
        {
            path.nodes[i].distance_to_saddle = 0;
            path.nodes[i].distance = 0;
            path.nodes[i].gradient = Eigen::Vector2f::Zero();
        }
    }
}

void refine(Band& path, const DistanceField& distance_field, const double min_distance, const double min_overlap)
{
    const std::size_t max_size = 100;

    //
    // add new nodes
    //
    std::vector<Node>::iterator iter = path.nodes.begin();
    while (std::distance(path.nodes.begin(), iter) < static_cast<int>(max_size))
    {
        const auto next = iter + 1;

        if (next == path.nodes.end())
            break;

        const double iter_exp = std::max(min_distance, std::abs(iter->distance));
        const double next_exp = std::max(min_distance, std::abs(next->distance));

        const double min_radius = std::min(iter_exp, next_exp);
        const double distance_to_next = (next->pose.translation() - iter->pose.translation()).norm();
        const double combined_radius = iter_exp + next_exp;
        const bool overlaping_next = distance_to_next < combined_radius - min_radius * min_overlap;

        if (!overlaping_next)
        {
            const double fraction = iter_exp / distance_to_next;

            Node new_node;
            new_node.pose =
                Eigen::Translation2d(iter->pose.translation() +
                                     fraction * (next->pose.translation() - iter->pose.translation())) *
                Eigen::Rotation2Dd(iter->pose.linear()).slerp(fraction, Eigen::Rotation2Dd(next->pose.linear()));

            {
                unsigned int mx;
                unsigned int my;
                if (distance_field.worldToMap(new_node.pose.translation().x(), new_node.pose.translation().y(), mx, my))
                {
                    new_node.distance = distance_field.distance(mx, my) * distance_field.resolution;
                    new_node.gradient = new_node.distance > 0 ? distance_field.positiveGradient(mx, my)
                                                              : distance_field.negativeGradient(mx, my);
                }
                else
                {
                    ++iter;
                    continue;
                }
            }

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
    while (std::distance(path.nodes.begin(), iter) < static_cast<int>(max_size))
    {
        const auto next = iter + 1;

        if (next == path.nodes.end())
            break;

        const double iter_exp = std::max(min_distance, std::abs(iter->distance));
        const double next_exp = std::max(min_distance, std::abs(next->distance));

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

            const double next_next_exp = std::max(min_distance, std::abs(next_next->distance));

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
        if (path.nodes[i].distance < 0)
        {
            path.nodes[i].pose.pretranslate(
                Eigen::Vector2d(path.nodes[i].gradient[0] * (-path.nodes[i].distance + 2 * distance_field.resolution),
                                path.nodes[i].gradient[1] * (-path.nodes[i].distance + 2 * distance_field.resolution)));
        }
    }
}

Eigen::Vector3d force(const Node& prev, const Node& curr, const Node& next, const double internal_force_gain,
                      const double external_force_gain, const double rotation_factor)
{
    const auto internal_force = internalForce(prev, curr, next, internal_force_gain, rotation_factor);
    const auto external_force = externalForce(curr, external_force_gain);
    return internal_force + external_force;
}

Eigen::Vector3d internalForce(const Node& prev, const Node& curr, const Node& next, const double internal_force_gain,
                              const double rotation_factor)
{
    const Eigen::Vector2d d_1 = prev.pose.translation() - curr.pose.translation();
    const Eigen::Vector2d d_2 = next.pose.translation() - curr.pose.translation();

    const Eigen::Vector2d d_1_norm = d_1.normalized();
    const Eigen::Vector2d d_2_norm = d_2.normalized();

    const Eigen::Rotation2Dd rot_1 = Eigen::Rotation2Dd(prev.pose.rotation().inverse() * curr.pose.rotation());
    const Eigen::Rotation2Dd rot_2 = Eigen::Rotation2Dd(next.pose.rotation().inverse() * curr.pose.rotation());

    Eigen::Vector3d force = Eigen::Vector3d::Zero();

    const double dis_1 = curr.distance + prev.distance;
    const double dis_2 = curr.distance + next.distance;
    const double gap_1 = dis_1 - d_1.norm();
    const double gap_2 = dis_2 - d_2.norm();

    // straightening force
    force.topRows(2) = internal_force_gain * (d_1_norm + d_2_norm);

    if (gap_1 > 0)
    {
        const double exp_1 = d_1.norm() < dis_1 ? 0.001 * std::exp(1.0 * (dis_1 - d_1.norm())) : 0.0;

        // repelling force
        force.topRows(2) += -exp_1 * d_1_norm;

        // attracting force
        force.topRows(2) += 0.004 * (d_1 * gap_1).normalized();
    }
    if (gap_2 > 0)
    {
        const double exp_2 = d_2.norm() < dis_2 ? 0.001 * std::exp(1.0 * (dis_2 - d_2.norm())) : 0.0;

        // repelling force
        force.topRows(2) += -exp_2 * d_2_norm;

        // attracting force
        force.topRows(2) += 0.004 * (d_2 * gap_2).normalized();
    }

    // rotation equilising force
    force[2] = -10 * internal_force_gain * (rot_1.smallestAngle() + rot_2.smallestAngle()) / 2.0;

    // face forward force
    const Eigen::Vector2d pose_dir = curr.pose.linear() * Eigen::Vector2d::UnitX();
    const double dot = pose_dir.dot(d_2_norm);
    const double det = pose_dir.x() * d_2_norm.y() - pose_dir.y() * d_2_norm.x();
    const double fwd_angle = std::atan2(det, dot);
    force[2] += 10 * rotation_factor * internal_force_gain * fwd_angle;

    return force;
}

Eigen::Vector3d externalForce(const Node& curr, const double external_force_gain)
{
    Eigen::Vector3d force;
    const double avoid_distance = 0.1;
    const double decay =
        (curr.distance > avoid_distance ? std::exp(-curr.distance - avoid_distance) : 1.0 / avoid_distance) *
        std::min(1.0, curr.distance_to_saddle);
    force[0] = -external_force_gain * curr.gradient.x() * decay;
    force[1] = -external_force_gain * curr.gradient.y() * decay;
    force[2] = 0.0;
    return force;
}
}
