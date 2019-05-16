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

using HeuristicFunction = std::function<double(const Coord2D&, const Coord2D&)>;
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
    bool visited;
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
    PathFinder(const int width, const int height, const float* data,
               const double neutral_cost = 0.1);
    ~PathFinder();

    PathResult findPath(Coord2D source_, Coord2D target_);

    const Cell* cell(const Coord2D& coordinates) const
    {
        return grid_map_[toIndex(coordinates)];
    }

    const std::vector<Cell*>& gridMap() const
    {
        return grid_map_;
    };

  private:
    inline std::size_t toIndex(const Coord2D& coordinates) const
    {
        return static_cast<std::size_t>(world_width_ * coordinates.y + coordinates.x);
    }

    Cell* cell(const Coord2D& coordinates)
    {
        return grid_map_[world_width_ * coordinates.y + coordinates.x];
    }

    const float* data_;

    const int world_width_;
    const int world_height_;

    const std::array<Coord2D, 8> directions_;
    const std::array<double, 8> direction_cost_;

    const double neutral_cost_;

    HeuristicFunction heuristic_;
    std::priority_queue<ScoreCoordPair, std::vector<ScoreCoordPair>, CompareScore> open_set_;
    std::vector<Cell*> grid_map_;
};

inline double manhattan(const Coord2D& source, const Coord2D& target)
{
    return std::abs(source.x - target.x) + std::abs(source.y - target.y);
}

inline double euclidean(const Coord2D& source, const Coord2D& target)
{
    return 0.5 * std::sqrt(std::pow((source.x - target.x), 2) + std::pow((source.y - target.y), 2));
}

inline double linf_norm(const Coord2D& source, const Coord2D& target)
{
    return std::max(std::abs(source.x - target.x), std::abs(source.y - target.y));
}

}

#endif
