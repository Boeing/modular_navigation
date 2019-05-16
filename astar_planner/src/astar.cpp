#include <astar_planner/astar.h>

#include <algorithm>
#include <cinttypes>
#include <fstream>
#include <iostream>
#include <sstream>

#include <chrono>

namespace astar_planner
{

bool Coord2D::operator==(const Coord2D& other) const
{
    return (x == other.x && y == other.y);
}

Coord2D operator+(const Coord2D& left_, const Coord2D& right_)
{
    return {left_.x + right_.x, left_.y + right_.y};
}

PathFinder::PathFinder(const int width, const int height, const float* data,
                       const double neutral_cost)
    : world_width_(width), world_height_(height),
      directions_({{{-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1}}}),
      direction_cost_({std::sqrt(2.0), 1.0, std::sqrt(2.0), 1.0, 1.0, std::sqrt(2.0), 1.0, std::sqrt(2.0)}),
      neutral_cost_(neutral_cost),
      heuristic_(std::bind(&euclidean, std::placeholders::_1, std::placeholders::_2)),
      open_set_(CompareScore())
{
    const std::size_t size = static_cast<std::size_t>(width * height);
    data_ = data;
    grid_map_.resize(size, nullptr);
}

PathFinder::~PathFinder()
{
    for (auto it = grid_map_.begin(); it != grid_map_.end(); ++it)
    {
        if (*it != nullptr)
            delete (*it);
    }
    grid_map_.clear();
}

PathResult PathFinder::findPath(Coord2D start_pos, Coord2D goal_pos)
{
    const std::size_t start_index = toIndex(start_pos);
    const std::size_t goal_index = toIndex(goal_pos);

    PathResult result;
    result.success = false;
    if (data_[start_index] >= 1.0f)
    {
        ROS_WARN("Start in collision");
        return result;
    }
    if (data_[goal_index] >= 1.0f)
    {
        ROS_WARN("Goal in collision");
        return result;
    }

    open_set_.push({0, start_pos});
    grid_map_[start_index] = new Cell{false, Coord2D(), static_cast<double>(data_[start_index])};

    bool solution_found = false;

    while (!open_set_.empty())
    {
        Coord2D current_coord = open_set_.top().second;
        open_set_.pop();

        if (current_coord == goal_pos)
        {
            solution_found = true;
            break;
        }

        const std::size_t current_index = toIndex(current_coord);
        Cell* current_cell = grid_map_[current_index];

        assert(current_cell != nullptr);

        current_cell->visited = true;

        for (std::size_t i = 0; i < 8; ++i)
        {
            const Coord2D new_coord = current_coord + directions_[i];

            // Check out of bounds
            if (new_coord.x < 0 || new_coord.x >= world_width_ || new_coord.y < 0 || new_coord.y >= world_height_)
            {
                continue;
            }

            const std::size_t new_index = toIndex(new_coord);

            Cell* new_cell = grid_map_[new_index];

            // Allocate if necessary
            if (!new_cell)
            {
                grid_map_[new_index] = new Cell{false, Coord2D(), std::numeric_limits<double>::max()};
                new_cell = grid_map_[new_index];
            }

            // Check visited
            if (new_cell->visited)
            {
                continue;
            }

            // Check collisions
            const double world = static_cast<double>(data_[new_index]);
            if (world >= 1.0)
            {
                continue;
            }

            const double new_cost = current_cell->cost + direction_cost_[i] * (world + neutral_cost_);
            if (new_cost < new_cell->cost)
            {
                const double H = heuristic_(new_coord, goal_pos);
                open_set_.push({new_cost + H, new_coord});
                new_cell->cost = new_cost;
                new_cell->path_parent = current_coord;
            }
        }
    }

    if (solution_found)
    {
        result.success = true;
        result.cost = grid_map_[goal_index]->cost;
        Coord2D coord = goal_pos;
        while (coord != start_pos)
        {
            result.path.push_back(coord);
            coord = cell(coord)->path_parent;
        }
    }

    return result;
}
}
