#include <astar_planner/astar.h>

#include <algorithm>
#include <cinttypes>
#include <fstream>
#include <iostream>
#include <sstream>

#include <chrono>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace astar_planner
{

namespace
{

double angle2vecs(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2)
{
    const double v1_norm = v1.norm();
    const double v2_norm = v2.norm();
    if (v1_norm > 1e-3 && v2_norm > 1e-3)
    {
        const double f = v1.dot(v2) / (v1_norm * v2_norm);
        return std::acos(std::min(1.0, std::max(-1.0, f)));
    }
    return 0;
}

double heuristic2d(const State2D& s1, const State2D& s2)
{
    const double dx = s2.x - s1.x;
    const double dy = s2.y - s1.y;
    return std::sqrt(dx * dx + dy * dy);
}
}  // namespace

ShortestPath2D shortestPath2D(const State2D& start, const State2D& goal, Explore2DCache& explore_cache,
                              const Costmap& costmap, const float closest_distance)
{
    ROS_ASSERT(start.x >= 0);
    ROS_ASSERT(start.x < costmap.width);

    ROS_ASSERT(start.y >= 0);
    ROS_ASSERT(start.y < costmap.height);

    ROS_ASSERT(goal.x >= 0);
    ROS_ASSERT(goal.x < costmap.width);

    ROS_ASSERT(goal.y >= 0);
    ROS_ASSERT(goal.y < costmap.height);

    ROS_ASSERT(costmap.traversal_cost);

    if (costmap.distance_to_collision.at<float>(start.y, start.x) <= 0)
    {
        return {false, nullptr, 0};
    }
    if (costmap.distance_to_collision.at<float>(goal.y, goal.x) <= 0)
    {
        return {false, nullptr, 0};
    }

    // attempt to find path from goal to start
    // reverse order so that we can reuse the explore data structure as the start state changes

    const std::size_t start_index = costmap.to2DGridIndex(start);
    const std::size_t goal_index = costmap.to2DGridIndex(goal);

    if (!explore_cache.explore_2d[goal_index])
        explore_cache.explore_2d[goal_index] = new Node2D{goal, nullptr, false, 0, heuristic2d(goal, start)};

    if (explore_cache.explore_2d[start_index])
    {
        return {true, explore_cache.explore_2d[start_index], 0};
    }

    std::vector<PriorityQueue2D::handle_type> handles(explore_cache.explore_2d.size(),
                                                      PriorityQueue2D::handle_type{nullptr});
    auto open_set = new PriorityQueue2D();

    // re-calculate the heuristic
    if (explore_cache.open_set)
    {
        for (const auto& item : *explore_cache.open_set)
        {
            item->cost_to_go = heuristic2d(item->state, start);
            handles[costmap.to2DGridIndex(item->state)] = open_set->push(item);
        }
    }
    else
    {
        // start exploring from goal state
        explore_cache.explore_2d[goal_index]->cost_to_go =
            heuristic2d(explore_cache.explore_2d[goal_index]->state, start);
        handles[goal_index] = open_set->push(explore_cache.explore_2d[goal_index]);
    }

    size_t itr = 0;

    const size_t max_iterations = explore_cache.explore_2d.size();

    bool solution_found = false;
    while (!open_set->empty() && itr++ < max_iterations)
    {
        auto current_node = open_set->top();
        open_set->pop();

        const std::size_t current_index = costmap.to2DGridIndex(current_node->state);

        const double traversal_cost_scale =
            static_cast<double>(costmap.traversal_cost->at<float>(static_cast<int>(current_index)));

        ROS_ASSERT(!current_node->visited);
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
            Node2D* new_node = explore_cache.explore_2d[new_index];

            // Allocate if necessary
            if (!new_node)
            {
                explore_cache.explore_2d[new_index] =
                    new Node2D{new_state, current_node, false, std::numeric_limits<double>::max(),
                               std::numeric_limits<double>::max()};
                new_node = explore_cache.explore_2d[new_index];
            }
            else if (new_node->visited)
            {
                continue;
            }

            if (costmap.distance_to_collision.at<float>(new_state.y, new_state.x) <= closest_distance)
            {
                continue;
            }

            const double cost_so_far = current_node->cost_so_far + directions_2d_cost[i] * traversal_cost_scale;
            if (cost_so_far < new_node->cost_so_far)
            {
                new_node->cost_so_far = cost_so_far;
                new_node->cost_to_go = heuristic2d(new_state, start);

                new_node->parent = current_node;

                if (handles[new_index].node_)
                    open_set->decrease(handles[new_index]);
                else
                    handles[new_index] = open_set->push(new_node);
            }
        }
    }

    if (explore_cache.open_set)
        delete explore_cache.open_set;
    explore_cache.open_set = open_set;

    return {solution_found, explore_cache.explore_2d[start_index], itr};
}

double updateH(const State3D& state, const State3D& goal, Explore2DCache& explore_cache, const Costmap& costmap,
               const float conservative_radius)
{
    // Calculate start and end map coordinates
    const int start_x = static_cast<int>(std::round((state.x - costmap.origin_x) / costmap.resolution));
    const int start_y = static_cast<int>(std::round((state.y - costmap.origin_y) / costmap.resolution));
    const int goal_x = static_cast<int>(std::round((goal.x - costmap.origin_x) / costmap.resolution));
    const int goal_y = static_cast<int>(std::round((goal.y - costmap.origin_y) / costmap.resolution));

    const State2D start_state{start_x, start_y};
    const State2D goal_state{goal_x, goal_y};

    // shortest path 2d with obstacles
    auto ret =
        shortestPath2D(start_state, goal_state, explore_cache, costmap, conservative_radius - costmap.inflation_radius);
    const double shortest_2d =
        ret.success ? ret.node->cost_so_far * costmap.resolution : std::numeric_limits<double>::max();

    ROS_ASSERT(std::isfinite(shortest_2d));

    double path_angle = 0;

    if (ret.success)
    {
        std::vector<State2D> path2d;
        auto node = ret.node;
        auto ref = ret.node;
        const double min_px_distance = 0.2 / costmap.resolution;
        do
        {
            if (!node->parent || heuristic2d(ref->state, node->state) > min_px_distance)
            {
                ref = node;
                path2d.push_back(node->state);
            }
            node = node->parent;
        } while (node);

        if (path2d.size() > 2)
        {
            const Eigen::Vector2d start_pose_dir = Eigen::Rotation2Dd(state.theta) * Eigen::Vector2d::UnitX();
            const Eigen::Vector2d start_dir{static_cast<double>(path2d[1].x - path2d[0].x),
                                            static_cast<double>(path2d[1].y - path2d[0].y)};
            const double to_start_angle = angle2vecs(start_pose_dir, start_dir);

            for (size_t i = 0; i < path2d.size() - 2; ++i)
            {
                const Eigen::Vector2d dir{static_cast<double>(path2d[i + 1].x - path2d[i].x),
                                          static_cast<double>(path2d[i + 1].y - path2d[i].y)};
                const Eigen::Vector2d dir_next{static_cast<double>(path2d[i + 2].x - path2d[i].x),
                                               static_cast<double>(path2d[i + 2].y - path2d[i].y)};
                path_angle += angle2vecs(dir, dir_next);
            }

            const Eigen::Vector2d end_pose_dir = Eigen::Rotation2Dd(goal.theta) * Eigen::Vector2d::UnitX();
            const Eigen::Vector2d end_dir{
                static_cast<double>(path2d[path2d.size() - 1].x - path2d[path2d.size() - 2].x),
                static_cast<double>(path2d[path2d.size() - 1].y - path2d[path2d.size() - 2].y)};
            const double to_end_angle = angle2vecs(end_pose_dir, end_dir);

            path_angle += to_start_angle;
            path_angle += to_end_angle;
            path_angle /= M_PI;

            ROS_ASSERT(std::isfinite(path_angle));
        }
    }

    return 1.25 * shortest_2d + 1.25 * path_angle;
}

PathResult hybridAStar(const Eigen::Isometry2d& start, const Eigen::Isometry2d& goal, const size_t max_iterations,
                       const Costmap& costmap, const CollisionChecker& collision_checker,
                       const float conservative_radius, const double linear_resolution, const double angular_resolution,
                       const bool backwards, const bool strafe)
{
    PathResult result(static_cast<std::size_t>(costmap.width), static_cast<std::size_t>(costmap.height));
    result.success = false;

    ROS_ASSERT(costmap.traversal_cost);

    const State3D start_state{start.translation().x(), start.translation().y(),
                              Eigen::Rotation2Dd(start.linear()).smallestAngle()};
    const State3D goal_state{goal.translation().x(), goal.translation().y(),
                             Eigen::Rotation2Dd(goal.linear()).smallestAngle()};

    if (!collision_checker.isValid(start_state))
    {
        ROS_WARN("Start in collision");
        return result;
    }
    if (!collision_checker.isValid(goal_state))
    {
        ROS_WARN("Goal in collision");
        return result;
    }

    PriorityQueue3D open_set;
    std::unordered_map<uint64_t, PriorityQueue3D::handle_type> handles;

    const auto start_index = StateToIndex(start_state, linear_resolution, angular_resolution);
    const auto start_key = IndexToKey(start_index);

    const auto goal_index = StateToIndex(goal_state, linear_resolution, angular_resolution);
    const auto goal_key = IndexToKey(goal_index);

    auto start_node = new Node3D{start_state, nullptr, false, 0, 0};
    start_node->cost_to_go = updateH(start_node->state, goal_state, result.explore_cache, costmap, conservative_radius);

    result.explore_3d.insert(std::make_pair(start_key, start_node));

    // start exploring from start state
    handles[start_key] = open_set.push(start_node);

    std::vector<std::array<double, 5>> normal_directions = {
        {linear_resolution, 0, 0, linear_resolution, 0},           // forwards
        {0, 0, angular_resolution, 0, angular_resolution / M_PI},  // rotate left
        {0, 0, -angular_resolution, 0, angular_resolution / M_PI}  // rotate right
    };

    std::vector<std::array<double, 5>> start_directions = normal_directions;
    start_directions.push_back({-linear_resolution, 0, 0, 1.5 * linear_resolution, 0});  // backwards

    if (backwards)
    {
        normal_directions.push_back({-linear_resolution, 0, 0, 1.5 * linear_resolution, 0});  // backwards
    }

    if (strafe)
    {
        normal_directions.push_back({0, linear_resolution, 0, 4.0 * linear_resolution, 0});   // strafe left
        normal_directions.push_back({0, -linear_resolution, 0, 4.0 * linear_resolution, 0});  // strafe right

        start_directions.push_back({0, linear_resolution, 0, 4.0 * linear_resolution, 0});   // strafe left
        start_directions.push_back({0, -linear_resolution, 0, 4.0 * linear_resolution, 0});  // strafe right
    }

    result.iterations = 0;

    while (!open_set.empty() && result.iterations++ < max_iterations)
    {
        auto current_node = open_set.top();
        open_set.pop();

        ROS_ASSERT(!current_node->visited);
        current_node->visited = true;

        const auto current_index = StateToIndex(current_node->state, linear_resolution, angular_resolution);
        const auto current_key = IndexToKey(current_index);

        if (current_key == goal_key)
        {
            break;
        }

        const int state_x =
            static_cast<int>(std::round((current_node->state.x - costmap.origin_x) / costmap.resolution));
        const int state_y =
            static_cast<int>(std::round((current_node->state.y - costmap.origin_y) / costmap.resolution));
        const State2D state2d{state_x, state_y};
        const std::size_t state_index = costmap.to2DGridIndex(state2d);
        const double traversal_cost_scale =
            static_cast<double>(costmap.traversal_cost->at<float>(static_cast<int>(state_index)));

        const double distance_to_start_x = std::abs(current_node->state.x - start_state.x);
        const double distance_to_start_y = std::abs(current_node->state.y - start_state.y);

        const double distance_to_goal_x = std::abs(current_node->state.x - goal_state.x);
        const double distance_to_goal_y = std::abs(current_node->state.y - goal_state.y);
        const double distance_to_goal =
            std::sqrt(distance_to_goal_x * distance_to_goal_x + distance_to_goal_y * distance_to_goal_y);

        if (distance_to_goal_x <= linear_resolution && distance_to_goal_y <= linear_resolution)
        {
            auto alloc = new Node3D{goal_state, current_node, false, current_node->cost_so_far, 0};
            result.explore_3d.insert(std::make_pair(goal_key, alloc));
            break;
        }

        const double distance_to_collision_px = collision_checker.clearance(current_node->state);
        const double distance_to_collision_m = distance_to_collision_px * costmap.resolution;
        const double step_mult = std::max(
            1.0, std::min(0.80 * std::min(distance_to_goal, distance_to_collision_m) / linear_resolution, 20.0));
        const double rot_mult = std::min(3.0, step_mult);

        // Allow more motion when near the start
        const std::vector<std::array<double, 5>>* directions;
        if ((distance_to_start_x <= 1.0 && distance_to_start_y <= 1.0) ||
            (distance_to_goal_x <= 1.0 && distance_to_goal_y <= 1.0))
        {
            directions = &start_directions;
        }
        else
        {
            directions = &normal_directions;
        }

        for (std::size_t i = 0; i < directions->size(); ++i)
        {
            State3D new_state = current_node->state;
            const Eigen::Vector2d new_position =
                step_mult * (Eigen::Rotation2Dd(current_node->state.theta) *
                             Eigen::Vector2d((*directions)[i][0], (*directions)[i][1]));
            new_state.x += new_position.x();
            new_state.y += new_position.y();

            new_state.theta = wrapAngle(current_node->state.theta + rot_mult * (*directions)[i][2]);

            const auto new_index = StateToIndex(new_state, linear_resolution, angular_resolution);
            const auto new_key = IndexToKey(new_index);

            // discretize
            new_state = IndexToState(new_index, linear_resolution, angular_resolution);

            if (new_key == current_key)
            {
                continue;
            }

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

            const double cost_so_far = current_node->cost_so_far +
                                       (step_mult * (*directions)[i][3]) * traversal_cost_scale +
                                       (rot_mult * (*directions)[i][4]);
            if (cost_so_far < new_node->second->cost_so_far)
            {
                const double old_cost = new_node->second->cost();
                const double cost_to_go =
                    updateH(new_node->second->state, goal_state, result.explore_cache, costmap, conservative_radius);

                if (cost_to_go + cost_so_far < old_cost)
                {
                    new_node->second->state = new_state;
                    new_node->second->cost_so_far = cost_so_far;
                    new_node->second->cost_to_go = cost_to_go;
                    new_node->second->parent = current_node;

                    ROS_ASSERT(std::isfinite(new_node->second->cost_to_go));

                    auto it = handles.find(new_key);
                    if (it != handles.end())
                    {
                        ROS_ASSERT(it->second.node_->value == new_node->second);
                        open_set.decrease(it->second);
                    }
                    else
                    {
                        handles[new_key] = open_set.push(new_node->second);
                    }
                }
            }

            ROS_ASSERT(new_node->second->parent != new_node->second);
        }
    }

    auto goal_node = result.explore_3d.find(goal_key);
    if (goal_node != result.explore_3d.end())
    {
        result.success = true;
        auto node = goal_node->second;
        auto node_key = goal_key;
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
