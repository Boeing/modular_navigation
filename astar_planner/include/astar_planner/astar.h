#ifndef ASTAR_PLANNER_ASTAR_H
#define ASTAR_PLANNER_ASTAR_H

#include <algorithm>
#include <functional>
#include <map>
#include <queue>
#include <set>
#include <unordered_map>
#include <vector>

#include <ros/console.h>

#include <astar_planner/costmap.h>
#include <astar_planner/node.h>

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
    std::vector<Node2D*> explore_2d;
    PriorityQueue2D* open_set;

    Explore2DCache(const std::size_t size_x, const std::size_t size_y)
        : explore_2d(std::vector<Node2D*>(static_cast<std::size_t>(size_x * size_y), nullptr)), open_set(nullptr)
    {
    }

    ~Explore2DCache()
    {
        for (auto it = explore_2d.begin(); it != explore_2d.end(); ++it)
        {
            if (*it != nullptr)
                delete (*it);
        }
        if (open_set)
            delete open_set;
    }
};

ShortestPath2D shortestPath2D(const State2D& start, const State2D& goal, Explore2DCache& explore_cache,
                              const Costmap& costmap, const float closest_distance = 0);

struct PathResult
{
    PathResult(const std::size_t size_x, const std::size_t size_y)
        : success(false), iterations(0), explore_cache(size_x, size_y)
    {
    }

    ~PathResult()
    {
        for (auto it = explore_3d.begin(); it != explore_3d.end(); ++it)
        {
            delete (it->second);
        }
    }

    bool success;
    size_t iterations;

    std::vector<Node3D*> path;

    Explore2DCache explore_cache;

    std::unordered_map<uint64_t, Node3D*> explore_3d;
};

double updateH(const Node3D& state, const State3D& goal, Explore2DCache& explore_cache, const Costmap& costmap,
               const float conservative_radius);

PathResult hybridAStar(const Eigen::Isometry2d& start, const Eigen::Isometry2d& goal, const size_t max_iterations,
                       const Costmap& costmap, const CollisionChecker& collision_checker,
                       const float conservative_radius, const double linear_resolution = 0.1,
                       const double angular_resolution = 0.2, const bool backwards = false, const bool strafe = false);
}

#endif
