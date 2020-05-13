#include <astar_planner/simple_svg.h>
#include <astar_planner/visualisation.h>
#include <boost/function.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/property_map/function_property_map.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <fstream>

namespace astar_planner
{

cv::Mat visualise(const Costmap& costmap, const PathResult& astar_result)
{
    cv::Mat disp;
    cv::cvtColor(costmap.obstacle_map, disp, cv::COLOR_GRAY2BGR);
    return visualise(disp, costmap, astar_result);
}

cv::Mat visualise(cv::Mat& disp, const Costmap& costmap, const PathResult& astar_result)
{
    double max_cost_so_far = 0;
    for (const auto& node : astar_result.explore_cache.explore_2d)
    {
        if (node.second.cost_so_far < std::numeric_limits<double>::max())
            max_cost_so_far = std::max(node.second.cost_so_far, max_cost_so_far);
    }

    for (const auto& node : astar_result.explore_cache.explore_2d)
    {
        const unsigned char c = static_cast<unsigned char>(255.0 - 255.0 * node.second.cost_so_far / max_cost_so_far);
        disp.at<cv::Vec3b>(node.second.state.y, node.second.state.x) = cv::Vec3b(c, c, 0);
    }

    double max_cost_3d = 0;
    double min_cost_3d = std::numeric_limits<double>::max();
    for (const auto& node : astar_result.explore_3d)
    {
        if (node.second->cost_so_far < std::numeric_limits<double>::max())
            max_cost_3d = std::max(node.second->cost_so_far, max_cost_3d);
        min_cost_3d = std::min(node.second->cost_so_far, min_cost_3d);
    }

    std::size_t max_count = 0;
    std::unordered_map<size_t, size_t> counts;
    for (auto node : astar_result.explore_3d)
    {
        const Eigen::Array2i cell = costmap.getCellIndex({node.second->state.x, node.second->state.y});
        const std::size_t idx = costmap.to2DGridIndex({cell.x(), cell.y()});
        counts[idx]++;
        if (counts[idx] > max_count)
            max_count = counts[idx];
    }

    for (auto node : astar_result.explore_3d)
    {
        if (!node.second->parent)
            continue;

        const Eigen::Array2i cell = costmap.getCellIndex({node.second->state.x, node.second->state.y});
        const std::size_t idx = costmap.to2DGridIndex({cell.x(), cell.y()});

        const unsigned char c = static_cast<unsigned char>(
            std::max(0.0, 255.0 * static_cast<double>(counts[idx]) / static_cast<double>(max_count)));

        disp.at<cv::Vec3b>(cell.y(), cell.x()) = cv::Vec3b(0, c, 0);
    }

    if (astar_result.success)
    {
        for (size_t i = 0; i < astar_result.path.size(); i++)
        {
            auto& node = astar_result.path[i];

            const Eigen::Vector2d position(node->state.x, node->state.y);
            const Eigen::Rotation2Dd rotation(node->state.theta);

            const int start_x = static_cast<int>(std::round((node->state.x - costmap.origin_x) / costmap.resolution));
            const int start_y = static_cast<int>(std::round((node->state.y - costmap.origin_y) / costmap.resolution));

            const Eigen::Vector2d x_end =
                Eigen::Vector2d(start_x, start_y) +
                Eigen::Vector2d(Eigen::Rotation2Dd(node->state.theta) * Eigen::Vector2d(10, 0));
            const Eigen::Vector2d y_end =
                Eigen::Vector2d(start_x, start_y) +
                Eigen::Vector2d(Eigen::Rotation2Dd(node->state.theta) * Eigen::Vector2d(0, 10));

            if (i % 20 == 0)
                cv::circle(disp, cv::Point(start_x, start_y), 0.416 / costmap.resolution, cv::Scalar(0, 255, 0), 1);

            cv::line(disp, cv::Point(start_x, start_y), cv::Point(x_end.x(), x_end.y()), cv::Scalar(0, 0, 255), 1);
            cv::line(disp, cv::Point(start_x, start_y), cv::Point(y_end.x(), y_end.y()), cv::Scalar(0, 255, 0), 1);
        }
    }

    if (!astar_result.path.empty())
    {
        auto first_node = astar_result.path.back();
        const Eigen::Array2i start_cell = costmap.getCellIndex({first_node->state.x, first_node->state.y});
        const std::size_t start_index = costmap.to2DGridIndex({start_cell.x(), start_cell.y()});
        auto node_it = astar_result.explore_cache.explore_2d.find(start_index);
        if (node_it != astar_result.explore_cache.explore_2d.end())
        {
            auto node = &node_it->second;
            do
            {
                disp.at<cv::Vec3b>(node->state.y, node->state.x) = cv::Vec3b(255, 0, 0);
                node = node->parent;
            } while (node);
        }
    }

    return disp;
}

void drawDot(const Costmap& costmap, const PathResult& astar_result, const Eigen::Isometry2d& goal,
             const std::string& graph_path, const double linear_resolution, const double angular_resolution)
{
    typedef boost::property<boost::edge_weight_t, double> EdgeProperties;

    struct VertexProperties
    {
        std::string tag;
        std::string location;
        // cppcheck-suppress unusedStructMember
        Node3D* node;
    };

    struct GraphProperties
    {
    };

    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, VertexProperties, EdgeProperties,
                                  GraphProperties>
        Explore3dGraph;
    typedef boost::graph_traits<Explore3dGraph>::vertex_descriptor vertex_descriptor;

    Explore3dGraph graph;

    std::unordered_map<uint64_t, vertex_descriptor> node_to_vertex_map;

    const State3D goal_state{goal.translation().x(), goal.translation().y(),
                             Eigen::Rotation2Dd(goal.linear()).smallestAngle()};

    const auto goal_index = StateToIndex(goal_state, linear_resolution, angular_resolution);
    const auto goal_key = IndexToKey(goal_index);

    for (auto node : astar_result.explore_3d)
    {
        std::string tag = "START";
        if (node.second->parent)
        {
            const double angle = node.second->parent->state.theta - node.second->state.theta;

            const auto node_index = StateToIndex(node.second->state, linear_resolution, angular_resolution);
            const auto node_key = IndexToKey(node_index);

            if (node_key == goal_key)
                tag = "GOAL";
            else if (std::abs(angle) < 1e-3)
                tag = "S";
            else if (angle < 0)
                tag = "L";
            else
                tag = "R";
        }
        const std::string location = std::to_string(node.second->state.x / costmap.resolution) + "," +
                                     std::to_string(-node.second->state.y / costmap.resolution) + "!";
        node_to_vertex_map[node.first] = boost::add_vertex({tag, location, node.second}, graph);
    }

    for (auto node : astar_result.explore_3d)
    {
        if (!node.second->parent)
            continue;

        const double cost = node.second->cost_so_far - node.second->parent->cost_so_far;

        const auto node_index = StateToIndex(node.second->state, linear_resolution, angular_resolution);
        const auto node_key = IndexToKey(node_index);

        const auto parent_index = StateToIndex(node.second->parent->state, linear_resolution, angular_resolution);
        const auto parent_key = IndexToKey(parent_index);

        boost::add_edge(node_to_vertex_map[parent_key], node_to_vertex_map[node_key], cost, graph);
    }

    std::ofstream dotfile(graph_path.c_str());

    boost::dynamic_properties dp;
    dp.property("node_id", boost::get(boost::vertex_index, graph));
    dp.property("label", boost::get(&VertexProperties::tag, graph));
    dp.property("pos", boost::get(&VertexProperties::location, graph));

    boost::write_graphviz_dp(dotfile, graph, dp);
}

bool drawPathSVG(const PathResult& astar_result, const std::string& svg_path)
{
    ROS_ASSERT(astar_result.success);

    const auto layout = std::make_shared<svg::Layout>(svg::Dimensions(0, 0), svg::Layout::Origin::TopLeft);
    const auto viewbox = std::make_shared<svg::ViewBox>(-10000, -10000, 20000, 20000);
    svg::Document doc(layout, viewbox);

    for (size_t i = 0; i < astar_result.path.size(); i++)
    {
        auto& node = astar_result.path[i];

        const Eigen::Vector2d position(node->state.x, node->state.y);
        const Eigen::Rotation2Dd rotation(node->state.theta);

        const Eigen::Vector2d x_dir = Eigen::Rotation2Dd(node->state.theta) * Eigen::Vector2d::UnitX();
        const Eigen::Vector2d y_dir = Eigen::Rotation2Dd(node->state.theta) * Eigen::Vector2d::UnitY();

        svg::Polyline x_path(svg::Stroke(3, svg::Color::Red()));
        x_path << svg::Point(node->state.x * 1000, node->state.y * 1000);
        x_path << svg::Point(node->state.x * 1000 + x_dir[0] * 0.04 * 1000,
                             node->state.y * 1000 + x_dir[1] * 0.04 * 1000);
        doc << x_path;

        svg::Polyline y_path(svg::Stroke(3, svg::Color::Green()));
        y_path << svg::Point(node->state.x * 1000, node->state.y * 1000);
        y_path << svg::Point(node->state.x * 1000 + y_dir[0] * 0.04 * 1000,
                             node->state.y * 1000 + y_dir[1] * 0.04 * 1000);
        doc << y_path;
    }

    return doc.save(svg_path);
}

}  // namespace astar_planner
