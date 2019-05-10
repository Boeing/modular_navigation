#include <astar_planner/astar.h>
#include <astar_planner/plugin.h>

#include <navigation_interface/params.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <pluginlib/class_list_macros.h>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/simplify.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

PLUGINLIB_EXPORT_CLASS(astar_planner::AStarPlanner, navigation_interface::PathPlanner)

namespace astar_planner
{

namespace
{

std::vector<Eigen::Vector2i> drawLine(const Eigen::Vector2i& start, const Eigen::Vector2i& end)
{
    if (start == end)
        return {end};

    double x1 = start.x();
    double x2 = end.x();

    double y1 = start.y();
    double y2 = end.y();

    const bool steep = (std::abs(y2 - y1) > std::abs(x2 - x1));
    if (steep)
    {
        std::swap(x1, y1);
        std::swap(x2, y2);
    }

    bool reverse = false;
    if (x1 > x2)
    {
        std::swap(x1, x2);
        std::swap(y1, y2);
        reverse = true;
    }

    const double dx = x2 - x1;
    const double dy = std::abs(y2 - y1);

    double error = dx / 2.0;
    const int ystep = (y1 < y2) ? 1 : -1;
    int y = static_cast<int>(y1);

    const int max_x = static_cast<int>(x2);

    std::vector<Eigen::Vector2i> line;
    for (int x = static_cast<int>(x1); x < max_x; ++x)
    {
        if (steep)
        {
            line.push_back({y, x});
        }
        else
        {
            line.push_back({x, y});
        }

        error -= dy;
        if (error < 0)
        {
            y += ystep;
            error += dx;
        }
    }

    if (reverse)
        std::reverse(line.begin(), line.end());

    return line;
}

}

AStarPlanner::AStarPlanner()
{
}

AStarPlanner::~AStarPlanner()
{
}

navigation_interface::PathPlanner::Result AStarPlanner::plan(const Eigen::Isometry2d& start, const Eigen::Isometry2d& goal)
{
    navigation_interface::PathPlanner::Result result;

    const cv::Mat cv_im_raw = cv::Mat(costmap_->getSizeInCellsY(), costmap_->getSizeInCellsX(), CV_8UC1, reinterpret_cast<void*>(costmap_->getCharMap()));

    cv::Mat cv_im;
    cv_im_raw.copyTo(cv_im);

    // inflate
    const double robot_radius = 0.7;
    {
        cv_im.setTo(0, cv_im == 255);
        cv_im.setTo(255, cv_im > 253);

        cv::Mat inv_cv_im;
        cv::bitwise_not(cv_im, inv_cv_im);

        cv::Mat dist;
        cv::distanceTransform(inv_cv_im, dist, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32S);

        dist.setTo(0, dist < robot_radius / costmap_->getResolution());

        dist = - (dist - robot_radius) * costmap_->getResolution();

        cv::exp(dist, dist);

        dist.convertTo(cv_im, CV_8U, 255.0, 0);
    }

    // Calculate start and end map coordinates
    const double start_x = (start.translation().x() - costmap_->getOriginX()) / costmap_->getResolution() - 0.5;
    const double start_y = (start.translation().y() - costmap_->getOriginY()) / costmap_->getResolution() - 0.5;
    const double goal_x = (goal.translation().x() - costmap_->getOriginX()) / costmap_->getResolution() - 0.5;
    const double goal_y = (goal.translation().y() - costmap_->getOriginY()) / costmap_->getResolution() - 0.5;

    ROS_DEBUG_STREAM("START: " << start.translation().transpose());
    ROS_DEBUG_STREAM("GOAL: " << goal.translation().transpose());

    if (start_x < 0 || start_x > costmap_->getSizeInCellsX() || start_y < 0 || start_y > costmap_->getSizeInCellsY())
    {
        ROS_WARN("The robot's start position is outside the global costmap");
        result.outcome = navigation_interface::PathPlanner::Outcome::FAILED;;
        return result;
    }

    if (goal_x < 0 || goal_x > costmap_->getSizeInCellsX() || goal_y < 0 || goal_y > costmap_->getSizeInCellsY())
    {
        ROS_WARN("The goal position is outside the global costmap");
        result.outcome = navigation_interface::PathPlanner::Outcome::FAILED;;
        return result;
    }

    const uint8_t obstacle_threshold = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    PathFinder astar(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(),
                     reinterpret_cast<uint8_t*>(cv_im.data), obstacle_threshold, neutral_cost_);

    const Coord2D start_coord(static_cast<int>(start_x), static_cast<int>(start_y));
    const Coord2D goal_coord(static_cast<int>(goal_x), static_cast<int>(goal_y));

    const auto t0 = std::chrono::steady_clock::now();

    PathResult astar_result = astar.findPath(start_coord, goal_coord);

    ROS_INFO_STREAM(
        "A-STAR took "
        << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count());

    if (astar_result.success)
    {
        ROS_INFO_STREAM("Found a plan with cost: " << astar_result.cost);
        std::vector<std::pair<double, double>> simplified_path;
        {
            typedef boost::geometry::model::d2::point_xy<double> xy;

            boost::geometry::model::linestring<xy> line;
            boost::geometry::append(line, xy(goal_x, goal_y));
            for (const Coord2D& coord : astar_result.path)
                boost::geometry::append(line, xy(coord.x, coord.y));
            boost::geometry::append(line, xy(start_x, start_y));

            boost::geometry::model::linestring<xy> simplified;
            const double step_size = 0.25 / costmap_->getResolution();
            boost::geometry::simplify(line, simplified, step_size);

            ROS_DEBUG_STREAM("Path of length: " << line.size() << " simplified to " << simplified.size());

            simplified_path.push_back({simplified.front().x(), simplified.front().y()});
            for (std::size_t i = 1; i < simplified.size(); ++i)
            {
                const double distance = boost::geometry::distance(simplified[i], simplified[i - 1]);
                if (distance > step_size)
                {
                    const unsigned int steps = static_cast<unsigned int>(distance / step_size);
                    for (std::size_t s = 1; s <= steps; ++s)
                    {
                        const double f = static_cast<double>(s) / (steps + 1);
                        const double x = simplified[i - 1].x() + (simplified[i].x() - simplified[i - 1].x()) * f;
                        const double y = simplified[i - 1].y() + (simplified[i].y() - simplified[i - 1].y()) * f;
                        simplified_path.push_back({x, y});
                    }
                }
                simplified_path.push_back({simplified[i].x(), simplified[i].y()});
            }
        }

        for (auto r_it = simplified_path.crbegin(); r_it != simplified_path.crend(); ++r_it)
        {
            const double x = costmap_->getOriginY() + (r_it->first + 0.5) * costmap_->getResolution();
            const double y = costmap_->getOriginY() + (r_it->second + 0.5) * costmap_->getResolution();
            const Eigen::Isometry2d p = Eigen::Translation2d(x, y) * Eigen::Rotation2Dd(0);
            result.path.nodes.push_back(p);
        }

        result.path.nodes.push_back(goal);

        orientation_filter_.processPath(result.path.nodes);

        result.cost = cost(result.path);
        result.outcome = navigation_interface::PathPlanner::Outcome::SUCCESSFUL;
    }
    else
    {
        ROS_ERROR("Failed to get a plan");
        result.outcome = navigation_interface::PathPlanner::Outcome::FAILED;
    }

    return result;
}

bool AStarPlanner::valid(const navigation_interface::Path& path) const
{
    return true;
}

double AStarPlanner::cost(const navigation_interface::Path& path) const
{
    std::vector<Eigen::Vector2i> line;
    for (std::size_t i=1; i < path.nodes.size(); ++i)
    {
        const auto start = path.nodes[i-1].translation();
        const auto end = path.nodes[i].translation();

        unsigned int start_cell_x, start_cell_y;
        if (!costmap_->worldToMap(start.x(), start.y(), start_cell_x, start_cell_y))
            return std::numeric_limits<double>::max();

        unsigned int end_cell_x, end_cell_y;
        if (!costmap_->worldToMap(end.x(), end.y(), end_cell_x, end_cell_y))
            return std::numeric_limits<double>::max();

        const std::vector<Eigen::Vector2i> segment =
            drawLine(Eigen::Vector2i(start_cell_x, start_cell_y), Eigen::Vector2i(end_cell_x, end_cell_y));

        line.insert(line.end(), segment.begin(), segment.end());
    }

    double cost = 0.0;
    for (std::size_t j = 0; j < line.size(); ++j)
    {
        const Eigen::Vector2i& p = line.at(j);
        const double world = static_cast<double>(costmap_->getCost(p.x(), p.y()));

        if (j == 0)
        {
            cost += world;
        }
        else
        {
            const double direction = (line.at(j) - line.at(j-1)).cast<double>().norm();
            cost += direction * (world + neutral_cost_);
        }
    }

    return cost;
}

void AStarPlanner::initialize(const XmlRpc::XmlRpcValue& parameters,
                              const std::shared_ptr<const costmap_2d::Costmap2D>& costmap)
{
    costmap_ = costmap;

    neutral_cost_ = navigation_interface::get_config_with_default_warn<double>(parameters, "neutral_cost", neutral_cost_, XmlRpc::XmlRpcValue::TypeDouble);

    orientation_filter_.setMode(1);
    orientation_filter_.setWindowSize(1);
}

}
