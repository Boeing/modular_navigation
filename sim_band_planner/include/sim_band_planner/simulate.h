#ifndef SIM_BAND_PLANNER_SIMULATE_H
#define SIM_BAND_PLANNER_SIMULATE_H

#include <sim_band_planner/band.h>
#include <sim_band_planner/distance_field.h>

namespace sim_band_planner
{

double simulate(Band& path, const DistanceField& distance_field, const int num_iterations, const double min_overlap,
                const double min_distance, const double internal_force_gain, const double external_force_gain,
                const double rotation_factor, const bool reverse_direction, const double velocity_decay,
                const double alpha_start, const double alpha_decay, const double max_distance, const int max_nodes);

void updateDistances(Node& node, const DistanceField& distance_field, const double max_distance);
void updateDistances(Band& path, const DistanceField& distance_field, const double max_distance);

void refine(Band& path, const DistanceField& distance_field,
            const double min_distance,
            const double max_distance,
            const double min_overlap,
            const int max_nodes);

Eigen::Vector3d force(const Node& prev, const Node& curr, const Node& next, const double internal_force_gain,
                      const double external_force_gain, const double rotation_factor, const bool reverse_direction,
                      const double max_distance);
Eigen::Vector3d internalForce(const Node& prev, const Node& curr, const Node& next, const double internal_force_gain,
                              const double rotation_factor, const bool reverse_direction, const double max_distance);
Eigen::Vector3d externalForce(const Node& curr, const double external_force_gain);
}

#endif
