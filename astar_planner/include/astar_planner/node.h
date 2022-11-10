#ifndef ASTAR_PLANNER_NODE_H
#define ASTAR_PLANNER_NODE_H

#include <boost/heap/binomial_heap.hpp>
//#include <ros/assert.h>
#include <array>
#include <cmath>

#include "rcpputils/asserts.hpp"

namespace astar_planner
{

inline double wrapAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle <= -M_PI)
        angle += 2 * M_PI;
    return angle;
}

struct State2D
{
    int x;
    int y;
};

inline bool operator==(const State2D& lhs, const State2D& rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

inline State2D operator+(const State2D& lhs, const State2D& rhs)
{
    return State2D{lhs.x + rhs.x, lhs.y + rhs.y};
}

inline State2D operator*(const double& scalar, const State2D& state)
{
    return State2D{static_cast<int>(std::round(state.x * scalar)), static_cast<int>(std::round(state.y * scalar))};
}

static const std::array<State2D, 16> directions_2d = {{{-1, -1},
                                                       {0, -1},
                                                       {1, -1},
                                                       {-1, 0},
                                                       {1, 0},
                                                       {-1, 1},
                                                       {0, 1},
                                                       {1, 1},

                                                       {1, 2},
                                                       {2, 1},
                                                       {2, -1},
                                                       {1, -2},
                                                       {-1, -2},
                                                       {-2, -1},
                                                       {-2, 1},
                                                       {-1, 2}}};
static const std::array<double, 16> directions_2d_cost = {std::sqrt(2.0),
                                                          1.0,
                                                          std::sqrt(2.0),
                                                          1.0,
                                                          1.0,
                                                          std::sqrt(2.0),
                                                          1.0,
                                                          std::sqrt(2.0),

                                                          std::sqrt(5.0),
                                                          std::sqrt(5.0),
                                                          std::sqrt(5.0),
                                                          std::sqrt(5.0),
                                                          std::sqrt(5.0),
                                                          std::sqrt(5.0),
                                                          std::sqrt(5.0),
                                                          std::sqrt(5.0)};

struct State3D
{
    double x;
    double y;
    double theta;
};

inline double linearDistance(const State3D& s1, const State3D& s2)
{
    const double dx = s2.x - s1.x;
    const double dy = s2.y - s1.y;
    return std::sqrt(dx * dx + dy * dy);
}

inline double distance(const State3D& s1, const State3D& s2)
{
    const double dx = s2.x - s1.x;
    const double dy = s2.y - s1.y;
    const double dt = s2.theta - s1.theta;
    return std::sqrt(dx * dx + dy * dy + dt * dt);
}

struct Node2D
{
    State2D state;
    Node2D* parent;

    bool visited;

    double cost_so_far;
    double cost_to_go;

    double cost() const
    {
        return cost_so_far + cost_to_go;
    }
};

struct Node3D
{
    State3D state;
    Node3D* parent;

    bool visited;

    double cost_so_far;
    double cost_to_go;

    double cost() const
    {
        return cost_so_far + cost_to_go;
    }
};

struct Node3dIndex
{
    int x;
    int y;
    int theta;
};

struct CompareNodes
{
    bool operator()(const Node3D* lhs, const Node3D* rhs) const
    {
        //        return lhs->cost_to_go > rhs->cost_to_go;
        return lhs->cost() > rhs->cost();
    }
    bool operator()(const Node2D* lhs, const Node2D* rhs) const
    {
        return lhs->cost() > rhs->cost();
    }
};

inline Node3dIndex StateToIndex(const State3D& state, const double linear_resolution, const double angular_resolution)
{
    // TODO perhaps bounds check
    //    ROS_ASSERT_MSG(state.x < linear_resolution * std::numeric_limits<int>::max(), "state: %f %f", state.x,
    //    state.y); ROS_ASSERT_MSG(state.y < linear_resolution * std::numeric_limits<int>::max(), "state: %f %f",
    //    state.x, state.y); ROS_ASSERT_MSG(state.x > linear_resolution * std::numeric_limits<int>::min(), "state: %f
    //    %f", state.x, state.y); ROS_ASSERT_MSG(state.y > linear_resolution * std::numeric_limits<int>::min(), "state:
    //    %f %f", state.x, state.y);

    const int x = static_cast<int>(std::round(state.x / linear_resolution));
    const int y = static_cast<int>(std::round(state.y / linear_resolution));
    const int theta = static_cast<int>(std::round(state.theta / angular_resolution));
    return {x, y, theta};
}

inline State3D IndexToState(const Node3dIndex& index, const double linear_resolution, const double angular_resolution)
{
    const double x = index.x * linear_resolution;
    const double y = index.y * linear_resolution;
    const double theta = index.theta * angular_resolution;
    return State3D{x, y, theta};
}

inline uint64_t IndexToKey(const Node3dIndex& index)
{
    uint64_t k_0(static_cast<uint16_t>(index.x));
    uint64_t k_1(static_cast<uint16_t>(index.y));
    uint64_t k_2(static_cast<uint32_t>(index.theta));
    return (k_0 << 48) | (k_1 << 32) | k_2;
}

inline Node3dIndex KeyToIndex(const uint64_t& key)
{
    return Node3dIndex{static_cast<int32_t>((key >> 48) & 0xFFFF), static_cast<int32_t>((key >> 32) & 0xFFFF),
                       static_cast<int32_t>(key & 0xFFFFFFFF)};
}

typedef boost::heap::binomial_heap<Node3D*, boost::heap::compare<CompareNodes>> PriorityQueue3D;
typedef boost::heap::binomial_heap<Node2D*, boost::heap::compare<CompareNodes>> PriorityQueue2D;
}  // namespace astar_planner

#endif
