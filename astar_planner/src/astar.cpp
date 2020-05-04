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

std::vector<Eigen::Vector2d> simplifyDouglasPeucker(const std::vector<Eigen::Vector2d>& poses,
                                                    const double trans_tolerance, const double max_segment_length)
{
    std::vector<std::size_t> included;
    included.push_back(0);

    std::vector<std::pair<std::size_t, std::size_t>> stack;
    stack.push_back({0, poses.size() - 1});

    const auto segmentLength = [](const std::vector<Eigen::Vector2d>& poses,
                                  const std::pair<std::size_t, std::size_t>& range) {
        double d = 0;
        for (std::size_t i = range.first; i + 1 < range.second; ++i)
            d += (poses[i + 1] - poses[i]).norm();
        return d;
    };

    while (!stack.empty())
    {
        const auto range = stack.back();
        stack.pop_back();

        if (range.second == range.first)
            continue;

        std::size_t index = range.first + 1;

        // find index of point with maximum distance from first and last point
        double max_dist = -1;
        const auto line = Eigen::ParametrizedLine<double, 2>::Through(poses[range.first], poses[range.second]);
        std::size_t dist_idx = index;
        for (std::size_t i = range.first + 1; i < range.second; ++i)
        {

            const double perpendicular_distance = line.distance(poses[i]);
            if (perpendicular_distance > max_dist)
            {
                dist_idx = i;
                max_dist = perpendicular_distance;
            }
        }

        if (max_dist > trans_tolerance)
        {
            included.push_back(dist_idx);
            stack.push_back({range.first, dist_idx});
            stack.push_back({dist_idx, range.second});
        }
        else if (segmentLength(poses, range) > max_segment_length)
        {
            const std::size_t mid_index = (range.second + range.first) / 2;
            included.push_back(mid_index);
            stack.push_back({range.first, mid_index});
            stack.push_back({mid_index, range.second});
        }
    }

    included.push_back(poses.size() - 1);

    std::sort(included.begin(), included.end());

    std::vector<Eigen::Vector2d> out;
    for (std::size_t i = 0; i < included.size(); ++i)
    {
        out.push_back(poses[included[i]]);
    }

    return out;
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
double collisionCost(const int map_x, const int map_y, const Costmap& costmap, const double conservative_radius)
{
    ROS_ASSERT(conservative_radius >= costmap.inflation_radius);
    const double distance_to_collision_px = static_cast<double>(costmap.distance_to_collision.at<float>(map_y, map_x));
    const double distance_to_collision_m =
        (distance_to_collision_px * costmap.resolution) - (conservative_radius - costmap.inflation_radius);
    return distance_to_collision_m < 1.0 ? std::min(4.0, std::max(1.0, 1.0 / (std::pow(distance_to_collision_m, 0.2))))
                                         : 1.0;
}

double traversalCost(const int map_x, const int map_y, const Costmap& costmap)
{
    return static_cast<double>(costmap.traversal_cost->at<float>(map_y, map_x));
}

ShortestPath2D shortestPath2D(const State2D& start, const State2D& goal, Explore2DCache& explore_cache,
                              const Costmap& costmap, const double conservative_radius)
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

    const float closest_distance_px =
        static_cast<float>((conservative_radius - costmap.inflation_radius) / costmap.resolution);

    ROS_ASSERT(closest_distance_px >= 0);

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

        ROS_ASSERT(!current_node->visited);
        current_node->visited = true;

        if (current_index == start_index)
        {
            solution_found = true;
            break;
        }

        const double collision_cost_first =
            collisionCost(current_node->state.x, current_node->state.y, costmap, conservative_radius);
        const double traversal_cost_first = traversalCost(current_node->state.x, current_node->state.y, costmap);

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

            if (costmap.distance_to_collision.at<float>(new_state.y, new_state.x) <= closest_distance_px)
            {
                continue;
            }

            // add cost to distance from collisions
            // this keeps a nice boundary away from objects
            const double collision_cost_second = collisionCost(new_state.x, new_state.y, costmap, conservative_radius);
            const double collision_cost = std::max(collision_cost_first, collision_cost_second);

            const double traversal_cost_second = traversalCost(new_state.x, new_state.y, costmap);
            const double traversal_cost = std::max(traversal_cost_first, traversal_cost_second);

            const double cost_so_far =
                current_node->cost_so_far + directions_2d_cost[i] * traversal_cost * collision_cost;
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
               const double conservative_radius)
{
    // Calculate start and end map coordinates
    const int start_x = static_cast<int>(std::round((state.x - costmap.origin_x) / costmap.resolution));
    const int start_y = static_cast<int>(std::round((state.y - costmap.origin_y) / costmap.resolution));
    const int goal_x = static_cast<int>(std::round((goal.x - costmap.origin_x) / costmap.resolution));
    const int goal_y = static_cast<int>(std::round((goal.y - costmap.origin_y) / costmap.resolution));

    const State2D start_state{start_x, start_y};
    const State2D goal_state{goal_x, goal_y};

    if (start_state == goal_state)
        return 0.0;

    // shortest path 2d with obstacles
    auto ret = shortestPath2D(start_state, goal_state, explore_cache, costmap, conservative_radius);

    if (ret.success)
    {
        double strafe_cost = 0;
        double straight_cost = 0;

        const double shortest_2d = ret.success ? ret.node->cost_so_far : std::numeric_limits<double>::max();
        ROS_ASSERT(std::isfinite(shortest_2d));

        std::vector<Eigen::Vector2d> path;
        auto node = ret.node;  // start node
        do
        {
            path.push_back({node->state.x, node->state.y});
            node = node->parent;
        } while (node);
        const std::vector<Eigen::Vector2d> simplified_path = simplifyDouglasPeucker(path, 3, 6);

        ROS_ASSERT_MSG(simplified_path.size() >= 2, "Size: %i Start: %i %i Goal: %i %i", simplified_path.size(),
                       start_x, start_y, goal_x, goal_y);

        // estimate cost to drive in straight line and rotate
        {
            double path_angle = 0;

            // Add the estimated rotations throughout the path
            if (simplified_path.size() > 2)
            {
                for (size_t i = 0; i < simplified_path.size() - 2; ++i)
                {
                    const Eigen::Vector2d dir{static_cast<double>(simplified_path[i + 1].x() - simplified_path[i].x()),
                                              static_cast<double>(simplified_path[i + 1].y() - simplified_path[i].y())};
                    const Eigen::Vector2d dir_next{
                        static_cast<double>(simplified_path[i + 2].x() - simplified_path[i].x()),
                        static_cast<double>(simplified_path[i + 2].y() - simplified_path[i].y())};
                    const double collision_cost_first =
                        collisionCost(static_cast<int>(simplified_path[i].x()),
                                      static_cast<int>(simplified_path[i].y()), costmap, conservative_radius);
                    const double collision_cost_second =
                        collisionCost(static_cast<int>(simplified_path[i + 1].x()),
                                      static_cast<int>(simplified_path[i + 1].y()), costmap, conservative_radius);
                    const double collision_cost = std::max(collision_cost_first, collision_cost_second);
                    path_angle += angle2vecs(dir, dir_next) * collision_cost;
                }
            }

            // Add the rotation at the start
            {
                const Eigen::Vector2d start_pose_dir = Eigen::Rotation2Dd(state.theta) * Eigen::Vector2d::UnitX();
                const Eigen::Vector2d start_dir{static_cast<double>(simplified_path[1].x() - simplified_path[0].x()),
                                                static_cast<double>(simplified_path[1].y() - simplified_path[0].y())};
                const double collision_cost =
                    collisionCost(static_cast<int>(simplified_path[0].x()), static_cast<int>(simplified_path[0].y()),
                                  costmap, conservative_radius);
                path_angle += angle2vecs(start_pose_dir, start_dir) * collision_cost;
            }

            // Add the rotation at the end
            {
                const Eigen::Vector2d end_pose_dir = Eigen::Rotation2Dd(goal.theta) * Eigen::Vector2d::UnitX();
                const Eigen::Vector2d end_dir{static_cast<double>(simplified_path[simplified_path.size() - 1].x() -
                                                                  simplified_path[simplified_path.size() - 2].x()),
                                              static_cast<double>(simplified_path[simplified_path.size() - 1].y() -
                                                                  simplified_path[simplified_path.size() - 2].y())};
                const double collision_cost =
                    collisionCost(static_cast<int>(simplified_path.back().x()),
                                  static_cast<int>(simplified_path.back().y()), costmap, conservative_radius);
                path_angle += angle2vecs(end_pose_dir, end_dir) * collision_cost;
            }

            straight_cost = shortest_2d * costmap.resolution + ANGULAR_MULT * path_angle;
        }

        // estimate cost to strafe
        {
            for (size_t i = 0; i < simplified_path.size() - 1; ++i)
            {
                const Eigen::Vector2d dir{static_cast<double>(simplified_path[i + 1].x() - simplified_path[i].x()),
                                          static_cast<double>(simplified_path[i + 1].y() - simplified_path[i].y())};
                const Eigen::Vector2d trans_dir = Eigen::Rotation2Dd(state.theta) * dir;

                const double x_cost = std::abs((trans_dir[0] > 0) ? trans_dir[0] : BACKWARDS_MULT * trans_dir[0]);
                const double y_cost = std::abs(STRAFE_MULT * trans_dir[1]);

                const double traversal_cost_first = traversalCost(static_cast<int>(simplified_path[i].x()),
                                                                  static_cast<int>(simplified_path[i].y()), costmap);
                const double traversal_cost_second =
                    traversalCost(static_cast<int>(simplified_path[i + 1].x()),
                                  static_cast<int>(simplified_path[i + 1].y()), costmap);
                const double traversal_cost = std::max(traversal_cost_first, traversal_cost_second);

                const double collision_cost_first =
                    collisionCost(static_cast<int>(simplified_path[i].x()), static_cast<int>(simplified_path[i].y()),
                                  costmap, conservative_radius);
                const double collision_cost_second =
                    collisionCost(static_cast<int>(simplified_path[i + 1].x()),
                                  static_cast<int>(simplified_path[i + 1].y()), costmap, conservative_radius);
                const double collision_cost = std::max(collision_cost_first, collision_cost_second);

                strafe_cost += (x_cost + y_cost) * costmap.resolution * traversal_cost * collision_cost;
            }

            // add rotation cost at end
            const double collision_cost =
                collisionCost(static_cast<int>(simplified_path.back().x()),
                              static_cast<int>(simplified_path.back().y()), costmap, conservative_radius);
            strafe_cost += ANGULAR_MULT * std::abs(wrapAngle(goal.theta - state.theta)) * collision_cost;
        }

        // Debug visualisation
        /*
                {
                    cv::Mat disp;
                    cv::cvtColor(costmap.obstacle_map, disp, cv::COLOR_GRAY2BGR);

                    double max_cost_so_far = 0;
                    for (const auto& node : explore_cache.explore_2d)
                    {
                        if (node && node->cost_so_far < std::numeric_limits<double>::max())
                            max_cost_so_far = std::max(node->cost_so_far, max_cost_so_far);
                    }

                    for (const auto& node : explore_cache.explore_2d)
                    {
                        if (node)
                        {
                            const unsigned char c = static_cast<unsigned char>(255.0 - 255.0 * node->cost_so_far /
           max_cost_so_far); disp.at<cv::Vec3b>(node->state.y, node->state.x) = cv::Vec3b(c, c, 0);
                        }
                    }

                    node = ret.node;  // start node
                    do
                    {
                        disp.at<cv::Vec3b>(node->state.y, node->state.x) = cv::Vec3b(0, 0, 200);
                        node = node->parent;
                    } while (node);

                    cv::namedWindow("win", cv::WINDOW_NORMAL);
                    cv::imshow("win", disp);
                    cv::waitKey(0);
                }
        */

        // Overestimate the cost slightly to help speed up planning
        // If we overestimate we rule out possible paths more aggressively, which reduces the overall search space
        // The downside is the path is less optimal
        return std::min(1.1 * strafe_cost, 1.1 * straight_cost);
    }
    else
    {
        return std::numeric_limits<double>::max();
    }
}

PathResult hybridAStar(const Eigen::Isometry2d& start, const Eigen::Isometry2d& goal, const size_t max_iterations,
                       const Costmap& costmap, const CollisionChecker& collision_checker,
                       const double conservative_radius, double linear_resolution, const double angular_resolution,
                       const navigation_interface::PathPlanner::GoalSampleSettings& goal_sample_settings)
{
    PathResult result(static_cast<std::size_t>(costmap.width), static_cast<std::size_t>(costmap.height));
    result.success = false;
    result.start_in_collision = false;
    result.goal_in_collision = false;

    ROS_ASSERT(costmap.traversal_cost);

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
    ROS_ASSERT_MSG(goal_sample_settings.std_x >= 0.0, "Goal Sample Standard Deviation X is negative");
    ROS_ASSERT_MSG(goal_sample_settings.std_y >= 0.0, "Goal Sample Standard Deviation Y is negative");
    ROS_ASSERT_MSG(goal_sample_settings.std_w >= 0.0, "Goal Sample Standard Deviation W is negative");

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

    auto start_node = new Node3D{start_state, nullptr, false, 0, 0};
    start_node->cost_to_go = updateH(start_node->state, goal_state, result.explore_cache, costmap, conservative_radius);

    result.explore_3d.insert(std::make_pair(start_key, start_node));

    // start exploring from start state
    handles[start_key] = open_set.push(start_node);

    // x, y, rotation
    const std::vector<ExploreDirection> directions = {
        {linear_resolution, 0, 0},   // forwards
        {-linear_resolution, 0, 0},  // backwards
        {0, linear_resolution, 0},   // left
        {0, -linear_resolution, 0},  // right
        {0, 0, angular_resolution},  // rotate left
        {0, 0, -angular_resolution}  // rotate right
    };

    result.iterations = 0;
    while (!open_set.empty() && result.iterations++ < max_iterations)
    {
        auto current_node = open_set.top();
        open_set.pop();

        ROS_ASSERT(!current_node->visited);
        current_node->visited = true;

        const auto current_index = StateToIndex(current_node->state, linear_resolution, angular_resolution);
        const auto current_key = IndexToKey(current_index);

        const double distance_to_goal_x = std::abs(current_node->state.x - goal_state.x);
        const double distance_to_goal_y = std::abs(current_node->state.y - goal_state.y);
        const double distance_to_goal =
            std::sqrt(distance_to_goal_x * distance_to_goal_x + distance_to_goal_y * distance_to_goal_y);
        const double rotation_to_goal = std::abs(wrapAngle(current_node->state.theta - goal_state.theta));

        if (current_key == goal_key ||
            (distance_to_goal_x <= linear_resolution && distance_to_goal_y <= linear_resolution &&
             rotation_to_goal < angular_resolution))
        {
            auto alloc = new Node3D{goal_state, current_node, false, current_node->cost_so_far, 0};
            result.explore_3d.insert(std::make_pair(goal_key, alloc));
            break;
        }

        const Eigen::Array2i map_cell = costmap.getCellIndex({current_node->state.x, current_node->state.y});
        const double traversal_cost_first = traversalCost(map_cell.x(), map_cell.y(), costmap);
        const double collision_cost_first = collisionCost(map_cell.x(), map_cell.y(), costmap, conservative_radius);

        const double distance_to_collision_px = collision_checker.clearance(current_node->state);
        const double distance_to_collision_m = distance_to_collision_px * costmap.resolution;

        const double max_multiple = 20.0;
        const double step_mult =
            std::max(1.0, std::min(0.80 * std::min(distance_to_goal, distance_to_collision_m) / linear_resolution,
                                   max_multiple));
        ROS_ASSERT(step_mult >= 1.0);
        ROS_ASSERT(step_mult <= max_multiple);

        for (std::size_t i = 0; i < directions.size(); ++i)
        {
            State3D new_state = current_node->state;
            const Eigen::Vector2d trans = step_mult * (Eigen::Rotation2Dd(current_node->state.theta) *
                                                       Eigen::Vector2d(directions[i].x, directions[i].y));
            new_state.x += trans.x();
            new_state.y += trans.y();
            new_state.theta = wrapAngle(current_node->state.theta + directions[i].rotation);

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

            const Eigen::Array2i new_map_cell = costmap.getCellIndex({new_state.x, new_state.y});
            const double collision_cost_second =
                collisionCost(new_map_cell.x(), new_map_cell.y(), costmap, conservative_radius);
            const double traversal_cost_second = traversalCost(new_map_cell.x(), new_map_cell.y(), costmap);

            const double x_cost =
                step_mult * std::abs((directions[i].x > 0) ? directions[i].x : BACKWARDS_MULT * directions[i].x);
            const double y_cost = step_mult * std::abs(STRAFE_MULT * directions[i].y);

            const double collision_cost = (collision_cost_first + collision_cost_second) / 2.0;
            const double traversal_cost = (traversal_cost_first + traversal_cost_second) / 2.0;

            const double rotation_cost = std::abs(directions[i].rotation) * ANGULAR_MULT * collision_cost;
            const double translation_cost = (x_cost + y_cost) * collision_cost * traversal_cost;

            const double cost_so_far = current_node->cost_so_far + (translation_cost + rotation_cost);
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
