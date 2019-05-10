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

namespace astar_planner
{

struct Coord2D
{
    Coord2D() : x(-1), y(-1)
    {
    }

    Coord2D(int x_, int y_) : x(x_), y(y_)
    {
    }

    bool operator==(const Coord2D& other) const;
    bool operator!=(const Coord2D& other) const
    {
        return !(*this == other);
    }

    int x;
    int y;
};

using HeuristicFunction = std::function<uint32_t(const Coord2D&, const Coord2D&)>;
using CoordinateList = std::vector<Coord2D>;

typedef std::pair<uint32_t, Coord2D> ScoreCoordPair;

struct CompareScore
{
    // Note: we want the priority_queue to be ordered from smaller to larger
    bool operator()(const ScoreCoordPair& a, const ScoreCoordPair& b)
    {
        return a.first > b.first;
    }
};

struct Cell
{
//    uint8_t world;
    bool already_visited;
    Coord2D path_parent;
    double cost;
};

struct PathResult
{
    bool success;
    double cost;
    CoordinateList path;
};

class PathFinder
{
  public:
    PathFinder(const int width, const int height, const uint8_t* data, const uint8_t obstacle_threshold = 253,
               const double neutral_cost = 10.0);
    ~PathFinder();

    PathResult findPath(Coord2D source_, Coord2D target_);

    const Cell& cell(const Coord2D& coordinates) const
    {
        return grid_map_[toIndex(coordinates)];
    }

    const std::vector<Cell>& gridMap() const
    {
        return grid_map_;
    };

  private:
    inline std::size_t toIndex(const Coord2D& coordinates) const
    {
        return static_cast<std::size_t>(world_width_ * coordinates.y + coordinates.x);
    }

    Cell& cell(const Coord2D& coordinates)
    {
        return grid_map_[world_width_ * coordinates.y + coordinates.x];
    }

    const uint8_t* data_;

    const int world_width_;
    const int world_height_;

    const std::array<Coord2D, 8> directions_;
    const std::array<double, 8> direction_cost_;

    const uint8_t obstacle_threshold_;
    const double neutral_cost_;

    HeuristicFunction heuristic_;
    std::priority_queue<ScoreCoordPair, std::vector<ScoreCoordPair>, CompareScore> open_set_;
    std::vector<Cell> grid_map_;
};

inline uint32_t manhattan(const Coord2D& source, const Coord2D& target)
{
    auto delta = Coord2D((source.x - target.x), (source.y - target.y));
    return static_cast<uint32_t>(10 * (abs(delta.x) + abs(delta.y)));
}

inline uint32_t euclidean(const Coord2D& source, const Coord2D& target)
{
    auto delta = Coord2D((source.x - target.x), (source.y - target.y));
    return static_cast<uint32_t>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

inline uint32_t linf_norm(const Coord2D& source, const Coord2D& target)
{
    return 10 * std::max(std::abs(source.x - target.x), std::abs(source.y - target.y));
}

inline uint32_t octagonal(const Coord2D& source, const Coord2D& target)
{
    auto delta = Coord2D(abs(source.x - target.x), abs(source.y - target.y));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}
}

#endif
