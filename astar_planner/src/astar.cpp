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
    // TODO store a reference to the data rather than copying
    data_ = data;

    grid_map_.resize(size);
    for (std::size_t i = 0; i < size; i++)
    {
        // grid_map_[i].world = data[i];
        grid_map_[i].already_visited = false;
        grid_map_[i].cost = std::numeric_limits<double>::max();
    }
}

PathFinder::~PathFinder()
{
}

PathResult PathFinder::findPath(Coord2D start_pos, Coord2D goal_pos)
{
    const std::size_t start_index = toIndex(start_pos);

    PathResult result;
    result.success = false;
    if (data_[start_index] >= obstacle_threshold_)
    {
        ROS_INFO("Start in collision");
        return result;
    }

    open_set_.push({0, start_pos});
    grid_map_[start_index].cost = static_cast<double>(data_[start_index]);

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
            const uint8_t world = data_[toIndex(new_coord)]; // new_cell.world;
            if (world >= obstacle_threshold_)
            {
                continue;
            }

            const double new_cost =
                current_cell.cost + direction_cost_[i] * (static_cast<double>(world) + neutral_cost_);
            if (new_cost < new_cell.cost)
            {
                const uint32_t H = heuristic_(new_coord, goal_pos);
                open_set_.push({new_cost + H, new_coord});
                new_cell.cost = new_cost;
                new_cell.path_parent = current_coord;
            }
        }
    }

    if (solution_found)
    {
        result.success = true;
        result.cost = grid_map_[toIndex(goal_pos)].cost;
        Coord2D coord = goal_pos;
        while (coord != start_pos)
        {
            result.path.push_back(coord);
            coord = cell(coord).path_parent;
        }
    }

    return result;
}
}
