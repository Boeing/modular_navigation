//#include <ros/assert.h>
#include <sim_band_planner/simulate.h>

#include <sstream>
#include <vector>

#include "rcpputils/asserts.hpp"

namespace sim_band_planner {

double simulate(Band &path, const DistanceField &distance_field,
                const int num_iterations, const double collision_distance,
                const double nominal_force_gain,
                const double internal_force_gain,
                const double external_force_gain, const double rotation_gain,
                const double velocity_decay, const double alpha_start,
                const double alpha_decay) {
  const double dt = 1.0;
  double alpha = alpha_start;

  for (int it = 0; it < num_iterations; it++) {
    updateDistances(path, distance_field);

    // f = ...
    std::vector<Eigen::Vector3d> forces(path.nodes.size(),
                                        Eigen::Vector3d::Zero());
    for (std::size_t i = 1; i < path.nodes.size() - 1; ++i) {
      const auto internal_force =
          internalForce(path.nodes[i - 1], path.nodes[i], path.nodes[i + 1],
                        internal_force_gain);
      const auto rotation_force =
          rotationForce(path.nodes[i - 1], path.nodes[i], path.nodes[i + 1],
                        rotation_gain, collision_distance);
      const auto external_force =
          externalForce(path.nodes[i], external_force_gain, collision_distance);
      const auto nominal_force =
          nominalForce(path.nodes[i], nominal_force_gain);

      forces[i] =
          (internal_force + rotation_force + external_force + nominal_force);

      rcpputils::assert_true(forces[i].allFinite());
    }

    // a = f / m
    const std::vector<Eigen::Vector3d> acc = forces;

    // v = v + a * dt;
    for (std::size_t i = 1; i < path.nodes.size() - 1; ++i) {
      rcpputils::assert_true(acc[i].allFinite());
      path.nodes[i].velocity += alpha * acc[i] * dt;
      path.nodes[i].velocity *= velocity_decay;
    }

    // s = vt
    for (std::size_t i = 1; i < path.nodes.size() - 2; ++i) {
      path.nodes[i].pose.pretranslate(
          dt * Eigen::Vector2d(path.nodes[i].velocity.topRows(2)));
      path.nodes[i].pose.rotate(
          Eigen::Rotation2Dd(dt * path.nodes[i].velocity[2]));
    }

    alpha -= alpha * alpha_decay;
  }

  updateDistances(path, distance_field);

  return alpha;
}

void updateDistances(Node &node, const DistanceField &distance_field) {
  double min_distance = std::numeric_limits<double>::max();
  node.closest_point = 0;
  for (std::size_t i = 0; i < node.control_points.size(); ++i) {
    ControlPoint &control_point = node.control_points[i];
    const Eigen::Vector2d position =
        node.pose.translation() + node.pose.linear() * control_point.offset;

    unsigned int mx;
    unsigned int my;
    if (distance_field.worldToMap(position.x(), position.y(), mx, my)) {
      control_point.distance_to_saddle =
          static_cast<double>(distance_field.distanceToSaddle(mx, my));
      control_point.distance =
          static_cast<double>(distance_field.distance(mx, my)) *
          distance_field.resolution;
      control_point.gradient = control_point.distance > 0
                                   ? distance_field.positiveGradient(mx, my)
                                   : distance_field.negativeGradient(mx, my);
    } else {
      control_point.distance_to_saddle = 0;
      control_point.distance = 0;
      control_point.gradient = Eigen::Vector2f::Zero();
    }

    if (control_point.distance < min_distance) {
      node.closest_point = i;
      min_distance = control_point.distance;
    }
  }
}

void updateDistances(Band &path, const DistanceField &distance_field) {
  for (std::size_t i = 0; i < path.nodes.size(); ++i) {
    updateDistances(path.nodes[i], distance_field);
  }
}

Eigen::Vector3d internalForce(const Node &prev, const Node &curr,
                              const Node &next, const double gain) {
  const Eigen::Vector2d d_1 = prev.pose.translation() - curr.pose.translation();
  const Eigen::Vector2d d_2 = next.pose.translation() - curr.pose.translation();

  const double d_1_norm = d_1.norm();
  const double d_2_norm = d_2.norm();

  rcpputils::assert_true(std::isfinite(d_1_norm));
  rcpputils::assert_true(std::isfinite(d_2_norm));

  const Eigen::Vector2d d_1_normalized =
      d_1_norm > 0 ? d_1 / d_1_norm : Eigen::Vector2d{0.0, 0.0};
  const Eigen::Vector2d d_2_normalized =
      d_2_norm > 0 ? d_2 / d_2_norm : Eigen::Vector2d{0.0, 0.0};

  Eigen::Vector3d _force = Eigen::Vector3d::Zero();

  // straightening force
  if (d_1_norm > std::numeric_limits<double>::epsilon() &&
      d_2_norm > std::numeric_limits<double>::epsilon()) {
    _force.topRows(2) = gain * (d_1_normalized + d_2_normalized);
  }

  if (d_1_norm > std::numeric_limits<double>::epsilon()) {
    // attracting force
    _force.topRows(2) += 2 * gain * d_1;
  }

  if (d_2_norm > std::numeric_limits<double>::epsilon()) {
    // attracting force
    _force.topRows(2) += 2 * gain * d_2;
  }

  rcpputils::assert_true(_force.allFinite(),
                         "internal force: " + std::to_string(_force[0]) + " " +
                             std::to_string(_force[1]) + " " +
                             std::to_string(_force[2]));
  return _force;
}

Eigen::Vector3d rotationForce(const Node &prev, const Node &curr,
                              const Node &next, const double rotation_gain,
                              const double collision_distance) {
  const Eigen::Rotation2Dd rot_1 =
      Eigen::Rotation2Dd(prev.pose.linear().inverse() * curr.pose.linear());
  const Eigen::Rotation2Dd rot_2 =
      Eigen::Rotation2Dd(next.pose.linear().inverse() * curr.pose.linear());

  Eigen::Vector3d _force = Eigen::Vector3d::Zero();

  // rotation equalising force
  _force[2] = -2 * rotation_gain *
              (rot_1.smallestAngle() + rot_2.smallestAngle()) / 2.0;

  // when close to objects rotate to avoid them otherwise rotate to face forward
  const ControlPoint &curr_min = curr.control_points[curr.closest_point];
  //    if (curr_min.distance > collision_distance)
  //    {
  //        // face forward (or backwards) force
  //        if (d_2_norm > std::numeric_limits<double>::epsilon())
  //        {
  //            const Eigen::Vector2d pose_dir = curr.pose.linear() *
  //            Eigen::Vector2d::UnitX(); const double dot =
  //            pose_dir.dot(d_2_normalized); const double det = pose_dir.x() *
  //            d_2_normalized.y() - pose_dir.y() * d_2_normalized.x(); double
  //            fwd_angle = std::atan2(det, dot); if (fwd_angle > M_PI / 2.0)
  //                fwd_angle = M_PI - fwd_angle;
  //            else if (fwd_angle < -M_PI / 2.0)
  //                fwd_angle = -M_PI - fwd_angle;
  //            _force[2] += rotation_gain * fwd_angle;
  //        }
  //    }
  //    else
  if (curr_min.distance < collision_distance) {
    // control point torque
    double cp_torque = 0;
    double scale = 0.1 * std::min(1.0, curr_min.distance_to_saddle);
    if (curr_min.distance < 0)
      scale *= -1;

    const Eigen::Vector3d rotation_force =
        scale * Eigen::Vector3d(static_cast<double>(curr_min.gradient.x()),
                                static_cast<double>(curr_min.gradient.y()), 0);
    const auto rot_offset = curr.pose.linear() * curr_min.offset;
    const Eigen::Vector3d offset_3d(rot_offset.x(), rot_offset.y(), 0);
    const auto tau = offset_3d.cross(rotation_force);
    cp_torque = -tau.z();
    _force[2] += rotation_gain * cp_torque;
  }

  rcpputils::assert_true(_force.allFinite(),
                         "internal force: " + std::to_string(_force[0]) + " " +
                             std::to_string(_force[1]) + " " +
                             std::to_string(_force[2]));
  return _force;
}

Eigen::Vector3d externalForce(const Node &curr, const double gain,
                              const double collision_distance) {
  const ControlPoint &curr_min = curr.control_points[curr.closest_point];
  Eigen::Vector3d _force;

  double decay = curr_min.distance < collision_distance
                     ? std::min(4.0, collision_distance / curr_min.distance)
                     : 0.0;

  decay *= std::min(1.0, std::max(0.1, curr_min.distance_to_saddle));

  _force[0] = -gain * static_cast<double>(curr_min.gradient.x()) * decay;
  _force[1] = -gain * static_cast<double>(curr_min.gradient.y()) * decay;
  _force[2] = 0.0;

  rcpputils::assert_true(_force.allFinite());
  return _force;
}

Eigen::Vector3d nominalForce(const Node &curr, const double gain) {
  const Eigen::Vector2d dir =
      curr.nominal.translation() - curr.pose.translation();
  const Eigen::Rotation2Dd rot =
      Eigen::Rotation2Dd(curr.nominal.linear().inverse() * curr.pose.linear());

  const double distance = dir.norm();
  rcpputils::assert_true(std::isfinite(distance));

  Eigen::Vector3d _force;
  _force[0] = gain * std::exp(distance) * dir.x();
  _force[1] = gain * std::exp(distance) * dir.y();
  _force[2] = -0.1 * gain * rot.smallestAngle();

  rcpputils::assert_true(_force.allFinite());
  return _force;
}

} // namespace sim_band_planner
