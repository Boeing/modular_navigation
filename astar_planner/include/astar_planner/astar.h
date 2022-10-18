#ifndef ASTAR_PLANNER_ASTAR_H
#define ASTAR_PLANNER_ASTAR_H

#include <astar_planner/costmap.h>
#include <astar_planner/node.h>
#include <navigation_interface/path_planner.h>
//#include <ros/console.h>
#include "rcpputils/asserts.hpp"

#include <algorithm>
#include <functional>
#include <map>
#include <queue>
#include <set>
#include <unordered_map>
#include <vector>

namespace astar_planner
{

struct ShortestPath2D
{
    bool success;
    Node2D* node;
    size_t iterations;
};

struct Explore2DCache
{
    std::unordered_map<std::size_t, Node2D> explore_2d;
    PriorityQueue2D* open_set;

    Explore2DCache() : open_set(nullptr)
    {
    }

    ~Explore2DCache()
    {
        if (open_set)
            delete open_set;
    }
};

double collisionCost(const int map_x, const int map_y, const CollisionChecker& collision_checker);

double rotationCollisionCost(const double distance_to_collision_m);

double traversalCost(const int map_x, const int map_y, const Costmap& costmap);

ShortestPath2D shortestPath2D(const State2D& start, const State2D& goal, Explore2DCache& explore_cache,
                              const Costmap& costmap, const double conservative_radius);

struct PathResult
{
    PathResult() : success(false), iterations(0), start_in_collision(false), goal_in_collision(false)
    {
    }

    ~PathResult()
    {
        for (auto it = explore_3d.begin(); it != explore_3d.end(); ++it)
        {
            delete (it->second);
        }
    }

    Eigen::Isometry2d start;
    Eigen::Isometry2d goal;

    bool success;
    size_t iterations;

    bool start_in_collision;
    bool goal_in_collision;

    std::vector<Node3D*> path;

    Explore2DCache explore_cache;

    std::unordered_map<uint64_t, Node3D*> explore_3d;
};

double updateH(const State2D& state, const State2D& goal, Explore2DCache& explore_cache, const Costmap& costmap,
               const double conservative_radius);

PathResult hybridAStar(const Eigen::Isometry2d& start, const Eigen::Isometry2d& goal, const size_t max_iterations,
                       const CollisionChecker& collision_checker, const double linear_resolution,
                       const double angular_resolution,
                       const navigation_interface::PathPlanner::GoalSampleSettings& goal_sample_settings,
                       const double backwards_mult, const double strafe_mult, const double rotation_mult);
}  // namespace astar_planner

#endif
