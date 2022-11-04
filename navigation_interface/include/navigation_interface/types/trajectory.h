#ifndef NAVIGATION_INTERFACE_TRAJECTORY_H
#define NAVIGATION_INTERFACE_TRAJECTORY_H

#include <Eigen/Geometry>

#include <std_msgs/msg/header.h>

#include <vector>

namespace navigation_interface
{

struct KinodynamicState
{
    Eigen::Isometry2d pose;
    Eigen::Vector3d velocity;
    double min_distance_to_collision;
};

struct Trajectory
{
    double cost;
    std::string id;
    std_msgs::msg::Header header;
    std::vector<KinodynamicState> states;
};
};  // namespace navigation_interface

#endif
