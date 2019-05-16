#ifndef SIM_BAND_PLANNER_SIMULATE_H
#define SIM_BAND_PLANNER_SIMULATE_H

#include <sim_band_planner/distance_field.h>
#include <sim_band_planner/band.h>

namespace sim_band_planner
{

void simulate(Band& path, const DistanceField& distance_field,
              const int num_iterations, const double min_overlap,
              const double min_distance, const double internal_force_gain,
              const double external_force_gain, const double rotation_factor,
              const double velocity_decay,
              const double alpha_start, const double alpha_decay,
              const double max_distance);

void updateDistances(Band& path, const DistanceField& distance_field, const double max_distance);

void refine(Band& path, const DistanceField& distance_field, const double tiny_bubble_distance,
            const double min_bubble_overlap);

Eigen::Vector3d force(const Node& prev, const Node& curr, const Node& next, const double internal_force_gain, const double external_force_gain, const double rotation_factor);
Eigen::Vector3d internalForce(const Node& prev, const Node& curr, const Node& next, const double internal_force_gain, const double rotation_factor);
Eigen::Vector3d externalForce(const Node& curr, const double external_force_gain);

}

#endif
