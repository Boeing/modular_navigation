#ifndef SIM_BAND_PLANNER_SIMULATE_H
#define SIM_BAND_PLANNER_SIMULATE_H

#include <navigation_interface/types/path.h>
#include <sim_band_planner/band.h>
#include <sim_band_planner/distance_field.h>

namespace sim_band_planner {

double simulate(Band &path, const DistanceField &distance_field,
                const int num_iterations, const double collision_distance,
                const double nominal_force_gain,
                const double internal_force_gain,
                const double external_force_gain, const double rotation_gain,
                const double velocity_decay, const double alpha_start,
                const double alpha_decay);

void updateDistances(Node &node, const DistanceField &distance_field);
void updateDistances(Band &path, const DistanceField &distance_field);

Eigen::Vector3d internalForce(const Node &prev, const Node &curr,
                              const Node &next, const double gain);

Eigen::Vector3d rotationForce(const Node &prev, const Node &curr,
                              const Node &next, const double rotation_gain,
                              const double collision_distance);

Eigen::Vector3d externalForce(const Node &curr, const double gain,
                              const double collision_distance);

Eigen::Vector3d nominalForce(const Node &curr, const double gain);

} // namespace sim_band_planner

#endif
