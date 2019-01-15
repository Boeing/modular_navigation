#include <astar_planner/astar.h>

#include <algorithm>
#include <cinttypes>
#include <fstream>
#include <iostream>
#include <sstream>

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

PathFinder::PathFinder(const int width, const int height, const uint8_t* data, const uint8_t obstacle_threshold,
                       const double neutral_cost)
    : world_width_(width), world_height_(height),
      directions_({{{-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1}}}),
      direction_cost_({std::sqrt(2.0), 1.0, std::sqrt(2.0), 1.0, 1.0, std::sqrt(2.0), 1.0, std::sqrt(2.0)}),
      obstacle_threshold_(obstacle_threshold), neutral_cost_(neutral_cost),
      heuristic_(std::bind(&octagonal, std::placeholders::_1, std::placeholders::_2)), open_set_(CompareScore())
{
    const std::size_t size = static_cast<std::size_t>(width * height);
    grid_map_.resize(size);
    for (std::size_t i = 0; i < size; i++)
    {
        grid_map_[i].world = data[i];
        grid_map_[i].already_visited = false;
        grid_map_[i].cost = std::numeric_limits<double>::max();
    }
}

PathFinder::~PathFinder()
{
}

PathResult PathFinder::findPath(Coord2D startPos, Coord2D goalPos)
{
    const std::size_t startIndex = toIndex(startPos);

    open_set_.push({0, startPos});
    grid_map_[startIndex].cost = 0.0;

    bool solution_found = false;

    while (!open_set_.empty())
    {
        Coord2D current_coord = open_set_.top().second;
        open_set_.pop();

        if (current_coord == goalPos)
        {
            solution_found = true;
            break;
        }

        const std::size_t current_index = toIndex(current_coord);
        Cell& current_cell = grid_map_[current_index];
        current_cell.already_visited = true;

        for (std::size_t i = 0; i < 8; ++i)
        {
            const Coord2D new_coord = current_coord + directions_[i];

            // Check out of bounds
            if (new_coord.x < 0 || new_coord.x >= world_width_ || new_coord.y < 0 || new_coord.y >= world_height_)
            {
                continue;
            }

            Cell& new_cell = cell(new_coord);

            // Check visited
            if (new_cell.already_visited)
            {
                continue;
            }

            // Check collisions
            const uint8_t world = new_cell.world;
            if (world >= obstacle_threshold_)
            {
                continue;
            }

            const double new_cost =
                current_cell.cost + direction_cost_[i] * (static_cast<double>(world) + neutral_cost_);
            if (new_cost < new_cell.cost)
            {
                const uint32_t H = heuristic_(new_coord, goalPos);
                open_set_.push({new_cost + H, new_coord});
                new_cell.cost = new_cost;
                new_cell.path_parent = current_coord;
            }
        }
    }

    PathResult result;
    if (solution_found)
    {
        result.success = true;
        result.cost = grid_map_[toIndex(goalPos)].cost;
        Coord2D coord = goalPos;
        while (coord != startPos)
        {
            result.path.push_back(coord);
            coord = cell(coord).path_parent;
        }
    }
    else
    {
        result.success = false;
    }

    return result;
}
}
