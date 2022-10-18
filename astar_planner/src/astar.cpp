#include <astar_planner/astar.h>
#include <astar_planner/visualisation.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>

namespace astar_planner
{

namespace
{

double heuristic2d(const State2D& s1, const State2D& s2)
{
    const double dx = s2.x - s1.x;
    const double dy = s2.y - s1.y;
    return std::sqrt(dx * dx + dy * dy);
}

struct ExploreDirection
{
    double x;
    double y;
    double rotation;
};

}  // namespace

// This is using the costmap (which is inflated the robot offset radius)
// Since we are considering the robot as a point for this check we need to subtract the conservative radius
double collisionCost(const int map_x, const int map_y, const CollisionChecker& collision_checker)
{
    const int distance_to_collision_px =
        static_cast<int>(collision_checker.costmap().distance_to_collision.at<float>(map_y, map_x));
    return collision_checker.collisionCost(distance_to_collision_px);
}

double rotationCollisionCost(const double distance_to_collision_m)
{
    return std::min(60.0, std::max(1.0, 1.0 / (distance_to_collision_m * distance_to_collision_m)));
}

double traversalCost(const int map_x, const int map_y, const Costmap& costmap)
{
    return static_cast<double>(costmap.traversal_cost->at<float>(map_y, map_x));
}

ShortestPath2D shortestPath2D(const State2D& start, const State2D& goal, Explore2DCache& explore_cache,
                              const CollisionChecker& collision_checker)
{
    const Costmap& costmap = collision_checker.costmap();

    rcpputils::assert_true(start.x >= 0);
    rcpputils::assert_true(start.x < costmap.width);

    rcpputils::assert_true(start.y >= 0);
    rcpputils::assert_true(start.y < costmap.height);

    rcpputils::assert_true(goal.x >= 0);
    rcpputils::assert_true(goal.x < costmap.width);

    rcpputils::assert_true(goal.y >= 0);
    rcpputils::assert_true(goal.y < costmap.height);

    rcpputils::assert_true(costmap.traversal_cost);

    const float closest_distance_px =
        static_cast<float>((collision_checker.conservativeRadius() - costmap.inflation_radius) / costmap.resolution);

    rcpputils::assert_true(closest_distance_px >= 0);

    if (costmap.distance_to_collision.at<float>(start.y, start.x) <= closest_distance_px)
    {
        return {false, nullptr, 0};
    }
    if (costmap.distance_to_collision.at<float>(goal.y, goal.x) <= closest_distance_px)
    {
        return {false, nullptr, 0};
    }

    // attempt to find path from goal to start
    // reverse order so that we can reuse the explore data structure as the start state changes

    const std::size_t start_index = costmap.to2DGridIndex(start);
    const std::size_t goal_index = costmap.to2DGridIndex(goal);

    auto goal_it = explore_cache.explore_2d.find(goal_index);
    if (goal_it == explore_cache.explore_2d.end())
    {
        bool success;
        std::tie(goal_it, success) =
            explore_cache.explore_2d.insert({goal_index, Node2D{goal, nullptr, false, 0, heuristic2d(goal, start)}});
    }

    auto start_it = explore_cache.explore_2d.find(start_index);
    if (start_it != explore_cache.explore_2d.end())
    {
        return {true, &(start_it->second), 0};
    }

    std::unordered_map<std::size_t, PriorityQueue2D::handle_type>
        handles;  // explore_cache.explore_2d.size(), PriorityQueue2D::handle_type{nullptr});

    PriorityQueue2D* open_set = new PriorityQueue2D();

    // re-calculate the heuristic
    // the start-state is moving, so the fastest way to get their changes
    // consequently the priority queue needs to be resorted
    // so we copy all nodes from the last priority queue into a new one but recalculate heuristics
    // all nodes which have already been searched remain searched and contain valid shortest paths
    if (explore_cache.open_set)
    {
        handles.reserve(explore_cache.open_set->size());
        for (const auto& item : *explore_cache.open_set)
        {
            item->cost_to_go = heuristic2d(item->state, start);
            handles[costmap.to2DGridIndex(item->state)] = open_set->push(item);
        }
    }
    else
    {
        // start exploring from goal state
        open_set = new PriorityQueue2D();
        goal_it->second.cost_to_go = heuristic2d(goal_it->second.state, start);
        handles[goal_index] = open_set->push(&goal_it->second);
    }

    size_t itr = 0;
    const size_t max_iterations = costmap.width * costmap.height;

    bool solution_found = false;
    while (!open_set->empty() && itr++ < max_iterations)
    {
        auto current_node = open_set->top();
        open_set->pop();

        const std::size_t current_index = costmap.to2DGridIndex(current_node->state);

        rcpputils::assert_true(!current_node->visited);
        current_node->visited = true;

        if (current_index == start_index)
        {
            solution_found = true;
            break;
        }

        for (std::size_t i = 0; i < directions_2d.size(); ++i)
        {
            const State2D new_state = current_node->state + directions_2d[i];

            if (new_state.x < 0 || new_state.x >= costmap.width || new_state.y < 0 || new_state.y >= costmap.height)
            {
                continue;
            }

            const std::size_t new_index = costmap.to2DGridIndex(new_state);
            auto new_node_it = explore_cache.explore_2d.find(new_index);

            // Allocate if necessary
            if (new_node_it == explore_cache.explore_2d.end())
            {
                bool success;
                std::tie(new_node_it, success) = explore_cache.explore_2d.insert(
                    {new_index, Node2D{new_state, current_node, false, std::numeric_limits<double>::max(),
                                       std::numeric_limits<double>::max()}});
            }
            else if (new_node_it->second.visited)
            {
                continue;
            }

            if (costmap.distance_to_collision.at<float>(new_state.y, new_state.x) <= closest_distance_px)
            {
                continue;
            }

            // add cost to distance from collisions
            // this keeps a nice boundary away from objects
            const double collision_cost = collisionCost(new_state.x, new_state.y, collision_checker);
            const double traversal_cost = traversalCost(new_state.x, new_state.y, costmap);

            const double cost_so_far =
                current_node->cost_so_far + directions_2d_cost[i] * traversal_cost * collision_cost;
            if (cost_so_far < new_node_it->second.cost_so_far)
            {
                new_node_it->second.cost_so_far = cost_so_far;
                new_node_it->second.cost_to_go = heuristic2d(new_state, start);

                new_node_it->second.parent = current_node;

                if (handles[new_index].node_)
                    open_set->decrease(handles[new_index]);
                else
                    handles[new_index] = open_set->push(&new_node_it->second);
            }
        }
    }

    if (explore_cache.open_set)
        delete explore_cache.open_set;
    explore_cache.open_set = open_set;

    start_it = explore_cache.explore_2d.find(start_index);
    Node2D* node_ptr = (start_it != explore_cache.explore_2d.end()) ? &start_it->second : nullptr;

    return {solution_found, node_ptr, itr};
}

double updateH(const State2D& state, const State2D& goal, Explore2DCache& explore_cache,
               const CollisionChecker& collision_checker)
{
    const Costmap& costmap = collision_checker.costmap();

    if (state == goal)
        return 0.0;

    // shortest path 2d with obstacles
    auto ret = shortestPath2D(state, goal, explore_cache, collision_checker);

    if (ret.success)
    {
        const double shortest_2d = ret.node->cost_so_far;
        rcpputils::assert_true(std::isfinite(shortest_2d));
        // Why do we multiply by 1.1?
        // Might be because of taking into account rotation in Djikstra
        return shortest_2d * costmap.resolution * 1.1;
    }
    else
    {
        return std::numeric_limits<double>::max();
    }
}

PathResult hybridAStar(const Eigen::Isometry2d& start, const Eigen::Isometry2d& goal, const size_t max_iterations,
                       const CollisionChecker& collision_checker, double linear_resolution,
                       const double angular_resolution,
                       const navigation_interface::PathPlanner::GoalSampleSettings& goal_sample_settings,
                       const double backwards_mult, const double strafe_mult, const double rotation_mult)
{
    const Costmap& costmap = collision_checker.costmap();
    PathResult result;
    result.start = start;
    result.goal = goal;
    result.success = false;
    result.start_in_collision = false;
    result.goal_in_collision = false;

    rcpputils::assert_true(costmap.traversal_cost);

    const State3D start_state{start.translation().x(), start.translation().y(),
                              Eigen::Rotation2Dd(start.linear()).smallestAngle()};
    const State3D nominal_goal_state{goal.translation().x(), goal.translation().y(),
                                     Eigen::Rotation2Dd(goal.linear()).smallestAngle()};

    if (!collision_checker.isValid(start_state))
    {
        result.start_in_collision = true;
        return result;
    }

    // sample to find valid goal
    rcpputils::assert_true(goal_sample_settings.std_x >= 0.0, "Goal Sample Standard Deviation X is negative");
    rcpputils::assert_true(goal_sample_settings.std_y >= 0.0, "Goal Sample Standard Deviation Y is negative");
    rcpputils::assert_true(goal_sample_settings.std_w >= 0.0, "Goal Sample Standard Deviation W is negative");

    State3D goal_state = nominal_goal_state;
    result.goal_in_collision = !collision_checker.isValid(goal_state);

    if (goal_sample_settings.std_x > std::numeric_limits<double>::epsilon() ||
        goal_sample_settings.std_y > std::numeric_limits<double>::epsilon() ||
        goal_sample_settings.std_w > std::numeric_limits<double>::epsilon())
    {
        std::random_device rd{};
        std::mt19937 gen{rd()};
        std::normal_distribution<double> dist_x{
            0, std::max(std::numeric_limits<double>::epsilon(), goal_sample_settings.std_x)};
        std::normal_distribution<double> dist_y{
            0, std::max(std::numeric_limits<double>::epsilon(), goal_sample_settings.std_y)};
        std::normal_distribution<double> dist_w{
            0, std::max(std::numeric_limits<double>::epsilon(), goal_sample_settings.std_w)};
        std::size_t samples = 0;

        while (result.goal_in_collision)
        {
            if (++samples > goal_sample_settings.max_samples)
                return result;

            const Eigen::Isometry2d sample_wrt_robot =
                Eigen::Translation2d(dist_x(gen), dist_y(gen)) * Eigen::Rotation2Dd(dist_w(gen));
            const Eigen::Isometry2d sample_wrt_goal = goal * sample_wrt_robot;

            goal_state.x = sample_wrt_goal.translation().x();
            goal_state.y = sample_wrt_goal.translation().y();
            goal_state.theta = Eigen::Rotation2Dd(sample_wrt_goal.linear()).smallestAngle();
            result.goal_in_collision = !collision_checker.isValid(goal_state);
        }
    }
    else if (result.goal_in_collision)
    {
        return result;
    }

    PriorityQueue3D open_set;
    std::unordered_map<uint64_t, PriorityQueue3D::handle_type> handles;

    const auto start_index = StateToIndex(start_state, linear_resolution, angular_resolution);
    const auto start_key = IndexToKey(start_index);

    const auto goal_index = StateToIndex(goal_state, linear_resolution, angular_resolution);
    const auto goal_key = IndexToKey(goal_index);

    const Eigen::Array2i goal_cell = costmap.getCellIndex({goal_state.x, goal_state.y});
    const State2D goal_state_2d{goal_cell.x(), goal_cell.y()};

    const Eigen::Array2i start_cell = costmap.getCellIndex({start_state.x, start_state.y});
    const State2D start_state_2d{start_cell.x(), start_cell.y()};

    auto start_node = new Node3D{start_state, nullptr, false, 0, 0};
    start_node->cost_to_go = updateH(start_state_2d, goal_state_2d, result.explore_cache, collision_checker);

    result.explore_3d.insert(std::make_pair(start_key, start_node));

    // start exploring from start state
    handles[start_key] = open_set.push(start_node);

    // x, y, rotation
    const std::vector<ExploreDirection> directions = {
        {linear_resolution, 0, 0},                    // forwards
        {linear_resolution, 0, angular_resolution},   // forwards + rotate left
        {linear_resolution, 0, -angular_resolution},  // forwards + rotate right
    };

    const std::vector<ExploreDirection> extra_directions = {
        {linear_resolution, 0, 0},                    // forwards
        {linear_resolution, 0, angular_resolution},   // forwards + rotate left
        {linear_resolution, 0, -angular_resolution},  // forwards + rotate right
        {-linear_resolution, 0, 0},                   // backwards
        {0, linear_resolution, 0},                    // left
        {0, -linear_resolution, 0},                   // right
        {0, 0, angular_resolution},                   // rotate left
        {0, 0, -angular_resolution}                   // rotate right
    };

    result.iterations = 0;
    while (!open_set.empty() && result.iterations++ < max_iterations)
    {
        auto current_node = open_set.top();
        open_set.pop();

        rcpputils::assert_true(!current_node->visited);
        current_node->visited = true;

        const auto current_index = StateToIndex(current_node->state, linear_resolution, angular_resolution);
        const auto current_key = IndexToKey(current_index);

        const double distance_to_goal_x = std::abs(current_node->state.x - goal_state.x);
        const double distance_to_goal_y = std::abs(current_node->state.y - goal_state.y);
        const double distance_to_goal_m =
            std::sqrt(distance_to_goal_x * distance_to_goal_x + distance_to_goal_y * distance_to_goal_y);
        const double rotation_to_goal = std::abs(wrapAngle(current_node->state.theta - goal_state.theta));

        if (current_key == goal_key ||
            (distance_to_goal_x <= linear_resolution * 2 && distance_to_goal_y <= linear_resolution * 2 &&
             rotation_to_goal < angular_resolution * 2))
        {
            auto alloc = new Node3D{goal_state, current_node, false, current_node->cost_so_far, 0};
            result.explore_3d.insert(std::make_pair(goal_key, alloc));
            break;
        }

        const Eigen::Array2i map_cell = costmap.getCellIndex({current_node->state.x, current_node->state.y});
        const double traversal_cost_first = traversalCost(map_cell.x(), map_cell.y(), costmap);
        const double collision_cost_first = collisionCost(map_cell.x(), map_cell.y(), collision_checker);

        const double distance_to_collision_px = collision_checker.clearance(current_node->state);
        const double distance_to_collision_m = distance_to_collision_px * costmap.resolution;

        const double d_to_collision_or_goal = std::min(distance_to_collision_m, distance_to_goal_m);

        // discretize
        double res_mult = 1.0;
        if (d_to_collision_or_goal < 0.2)
        {
            res_mult = 1;
        }
        else if (d_to_collision_or_goal < 0.4)
        {
            res_mult = 2;
        }
        else
        {
            res_mult = 4;
        }

        const double s1 = std::abs(costmap.resolution / std::cos(current_node->state.theta));
        const double s2 = std::abs(costmap.resolution / std::sin(current_node->state.theta));
        // I think this needs to be sqrt(2) = 1.4, use 1.5 to help rounding up
        const double step_mult = 1.5 * res_mult * std::min(s1, s2) / costmap.resolution;

        rcpputils::assert_true(step_mult >= 1.0);

        const double distance_to_start_x = std::abs(current_node->state.x - start_state.x);
        const double distance_to_start_y = std::abs(current_node->state.y - start_state.y);
        const double distance_to_start =
            std::sqrt(distance_to_start_x * distance_to_start_x + distance_to_start_y * distance_to_start_y);

        const double d_to_stuff = std::min(d_to_collision_or_goal, distance_to_start);
        const std::vector<ExploreDirection>& current_directions = d_to_stuff > 0.5 ? directions : extra_directions;

        for (std::size_t i = 0; i < current_directions.size(); ++i)
        {
            State3D new_state = current_node->state;

            if (std::abs(current_directions[i].rotation) > 0)
                new_state.theta = wrapAngle(current_node->state.theta + current_directions[i].rotation);

            if (std::abs(current_directions[i].x) > 0 || std::abs(current_directions[i].y) > 0)
            {
                const double st = std::sin(new_state.theta);
                const double ct = std::cos(new_state.theta);
                const double dx = step_mult * (current_directions[i].x * ct - current_directions[i].y * st);
                const double dy = step_mult * (current_directions[i].x * st + current_directions[i].y * ct);
                new_state.x += dx;
                new_state.y += dy;
            }

            const auto down_index = StateToIndex(new_state, linear_resolution * res_mult, angular_resolution);
            new_state = IndexToState(down_index, linear_resolution * res_mult, angular_resolution);

            auto new_index = StateToIndex(new_state, linear_resolution, angular_resolution);
            const auto new_key = IndexToKey(new_index);

            //            ROS_ASSERT(new_key != current_key);
            if (new_key == current_key)
                continue;

            auto new_node = result.explore_3d.find(new_key);

            // Allocate if necessary
            if (new_node == result.explore_3d.end())
            {
                auto alloc = new Node3D{new_state, current_node, false, std::numeric_limits<double>::max(),
                                        std::numeric_limits<double>::max()};
                new_node = result.explore_3d.insert(std::make_pair(new_key, alloc)).first;
            }
            else if (new_node->second->visited)
            {
                continue;
            }

            if (!collision_checker.isWithinBounds(new_state))
            {
                continue;
            }

            if (!collision_checker.isValid(new_state))
            {
                continue;
            }

            const double trans_x = (new_state.x - current_node->state.x);
            const double trans_y = (new_state.y - current_node->state.y);

            const double st = std::sin(new_state.theta);
            const double ct = std::cos(new_state.theta);
            // inverse rotation matrix
            const double dx = (trans_x * ct + trans_y * st);
            const double dy = (-trans_x * st + trans_y * ct);

            const double trans_w = wrapAngle(new_state.theta - current_node->state.theta);

            const double x_cost = std::abs((dx > 0) ? dx : backwards_mult * dx);
            const double y_cost = std::abs(strafe_mult * dy);

            const Eigen::Array2i new_map_cell = costmap.getCellIndex({new_state.x, new_state.y});
            const double collision_cost_second = collisionCost(new_map_cell.x(), new_map_cell.y(), collision_checker);
            const double traversal_cost_second = traversalCost(new_map_cell.x(), new_map_cell.y(), costmap);

            const double collision_cost = (collision_cost_first + collision_cost_second) / 2.0;
            const double traversal_cost = (traversal_cost_first + traversal_cost_second) / 2.0;

            const double new_state_d_to_collision_m = collision_checker.clearance(new_state) * costmap.resolution;
            const double rotation_collision_cost = rotationCollisionCost(new_state_d_to_collision_m);

            const double rotation_cost = std::abs(trans_w) * rotation_mult * rotation_collision_cost;
            const double translation_cost = (x_cost + y_cost) * collision_cost * traversal_cost;

            const double cost = translation_cost + rotation_cost;
            const double cost_so_far = current_node->cost_so_far + cost;

            if (cost_so_far < new_node->second->cost_so_far)
            {
                // Calculate start and end map coordinates
                const Eigen::Array2i cell =
                    costmap.getCellIndex({new_node->second->state.x, new_node->second->state.y});
                const State2D state_2d{cell.x(), cell.y()};

                const double old_cost = new_node->second->cost();
                const double cost_to_go = updateH(state_2d, goal_state_2d, result.explore_cache, collision_checker);

                if (cost_to_go + cost_so_far < old_cost)
                {
                    new_node->second->state = new_state;
                    new_node->second->cost_so_far = cost_so_far;
                    new_node->second->cost_to_go = cost_to_go;
                    new_node->second->parent = current_node;

                    rcpputils::assert_true(std::isfinite(new_node->second->cost_to_go));

                    auto it = handles.find(new_key);
                    if (it != handles.end())
                    {
                        rcpputils::assert_true(it->second.node_->value == new_node->second);
                        open_set.decrease(it->second);
                    }
                    else
                    {
                        handles[new_key] = open_set.push(new_node->second);
                    }
                }
            }

            rcpputils::assert_true(new_node->second->parent != new_node->second);
        }
    }

    auto goal_node = result.explore_3d.find(goal_key);
    if (goal_node != result.explore_3d.end())
    {
        result.success = true;
        auto node = goal_node->second;
        uint64_t node_key = 0;
        do
        {
            const auto node_index = StateToIndex(node->state, linear_resolution, angular_resolution);
            node_key = IndexToKey(node_index);
            result.path.push_back(node);
            node = node->parent;
        } while (node && node_key != start_key);
    }

    return result;
}
}  // namespace astar_planner
