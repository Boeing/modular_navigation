#include <astar_planner/astar.h>
#include <astar_planner/plugin.h>

#include <gridmap/operations/raytrace.h>

#include <navigation_interface/params.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>

#include <pluginlib/class_list_macros.h>

#include <nav_msgs/OccupancyGrid.h>

PLUGINLIB_EXPORT_CLASS(astar_planner::AStarPlanner, navigation_interface::PathPlanner)

namespace astar_planner
{

namespace
{

struct Costmap
{
    cv::Mat costmap;
    cv::Mat obstacle_map;

    double resolution;
    double origin_x;
    double origin_y;
};

Costmap buildCostmap(const gridmap::MapData& map_data, const double robot_radius, const double exponential_weight,
                     const int down_sample)
{
    Costmap grid;

    // downsample
    {
        auto lock = map_data.grid.getLock();

        grid.resolution = map_data.grid.dimensions().resolution() * down_sample;

        const int size_x = map_data.grid.dimensions().size().x() / down_sample;
        const int size_y = map_data.grid.dimensions().size().y() / down_sample;

        const Eigen::Vector2d world_origin = map_data.grid.dimensions().getCellCenter({0, 0});
        grid.origin_x = world_origin.x();
        grid.origin_y = world_origin.y();

        grid.obstacle_map = cv::Mat(size_y, size_x, CV_8U, cv::Scalar(0));

        int index = 0;
        for (int i = 0; i < map_data.grid.dimensions().size().x(); ++i)
        {
            for (int j = 0; j < map_data.grid.dimensions().size().y(); ++j)
            {
                const int sub_i = i / down_sample;
                const int sub_j = j / down_sample;
                const int sub_index = sub_i * size_x + sub_j;
                if (map_data.grid.cells()[index] == gridmap::OccupancyGrid::OCCUPIED)
                    grid.obstacle_map.at<unsigned char>(sub_index) = 255;
                ++index;
            }
        }
    }

    // dilate robot radius
    cv::Mat dilated;
    const int cell_inflation_radius = static_cast<int>(robot_radius / grid.resolution);
    auto ellipse = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(cell_inflation_radius, cell_inflation_radius));
    cv::dilate(grid.obstacle_map, dilated, ellipse);

    // flip
    cv::bitwise_not(dilated, grid.obstacle_map);

    // allocate
    grid.costmap = cv::Mat(grid.obstacle_map.size(), CV_32F);

    // find obstacle distances
    cv::distanceTransform(grid.obstacle_map, grid.costmap, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);

    // inflate
    grid.costmap = -exponential_weight * grid.costmap * map_data.grid.dimensions().resolution();

    // negative exponent maps values to [1,0)
    cv::exp(grid.costmap, grid.costmap);

    return grid;
}

double pathCost(const navigation_interface::Path& path, const Costmap& costmap, const double neutral_cost)
{
    std::vector<Eigen::Vector2i> line;
    for (std::size_t i = 1; i < path.nodes.size(); ++i)
    {
        const auto start = path.nodes[i - 1].translation();
        const auto end = path.nodes[i].translation();

        const int start_cell_x = static_cast<int>((start.x() - costmap.origin_x) / costmap.resolution);
        const int start_cell_y = static_cast<int>((start.y() - costmap.origin_y) / costmap.resolution);

        const int end_cell_x = static_cast<int>((end.x() - costmap.origin_x) / costmap.resolution);
        const int end_cell_y = static_cast<int>((end.y() - costmap.origin_y) / costmap.resolution);

        const std::vector<Eigen::Array2i> segment =
            gridmap::drawLine(start_cell_x, start_cell_y, end_cell_x, end_cell_y);

        line.insert(line.end(), segment.begin(), segment.end());
    }

    double cost = 0.0;
    for (std::size_t j = 0; j < line.size(); ++j)
    {
        const Eigen::Vector2i& p = line.at(j);

        if (p.x() > costmap.costmap.size().width || p.x() < 0 || p.y() > costmap.costmap.size().height || p.y() < 0)
            return std::numeric_limits<double>::max();

        double world = static_cast<double>(costmap.costmap.at<float>(p.y(), p.x()));

        if (world >= 1.0)
            return std::numeric_limits<double>::max();

        if (j == 0)
        {
            cost += world;
        }
        else
        {
            const double direction = (line.at(j) - line.at(j - 1)).cast<double>().norm();
            cost += direction * (world + neutral_cost);
        }
    }

    return cost;
}
}

AStarPlanner::AStarPlanner()
{
}

AStarPlanner::~AStarPlanner()
{
}

navigation_interface::PathPlanner::Result AStarPlanner::plan(const Eigen::Isometry2d& start,
                                                             const Eigen::Isometry2d& goal)
{
    navigation_interface::PathPlanner::Result result;

    const Costmap costmap = buildCostmap(*map_data_, robot_radius_, exponential_weight_, down_sample_);

    if (debug_viz_)
    {
        nav_msgs::OccupancyGrid grid;
        grid.header.frame_id = "map";
        grid.header.stamp = ros::Time::now();
        grid.info.resolution = costmap.resolution;
        grid.info.width = costmap.costmap.size().width;
        grid.info.height = costmap.costmap.size().height;
        grid.info.origin.position.x = costmap.origin_x;
        grid.info.origin.position.y = costmap.origin_y;
        grid.info.origin.orientation.w = 1.0;
        grid.data.resize(grid.info.width * grid.info.height);
        for (unsigned int i = 0; i < grid.data.size(); i++)
        {
            grid.data[i] = costmap.costmap.at<float>(i) * 100.f;
        }
        pub_.publish(grid);
    }

    // Calculate start and end map coordinates
    const double start_x = (start.translation().x() - costmap.origin_x) / costmap.resolution - 0.5;
    const double start_y = (start.translation().y() - costmap.origin_y) / costmap.resolution - 0.5;
    const double goal_x = (goal.translation().x() - costmap.origin_x) / costmap.resolution - 0.5;
    const double goal_y = (goal.translation().y() - costmap.origin_y) / costmap.resolution - 0.5;

    if (start_x < 0 || start_x > costmap.costmap.size().width || start_y < 0 || start_y > costmap.costmap.size().height)
    {
        ROS_WARN("The robot's start position is outside the costmap");
        result.outcome = navigation_interface::PathPlanner::Outcome::FAILED;
        ;
        return result;
    }

    if (goal_x < 0 || goal_x > costmap.costmap.size().width || goal_y < 0 || goal_y > costmap.costmap.size().height)
    {
        ROS_WARN("The goal position is outside the costmap");
        result.outcome = navigation_interface::PathPlanner::Outcome::FAILED;
        ;
        return result;
    }

    PathFinder astar(costmap.costmap.size().width, costmap.costmap.size().height,
                     reinterpret_cast<float*>(costmap.costmap.data), neutral_cost_);

    const Coord2D start_coord(static_cast<int>(start_x), static_cast<int>(start_y));
    const Coord2D goal_coord(static_cast<int>(goal_x), static_cast<int>(goal_y));

    PathResult astar_result = astar.findPath(start_coord, goal_coord);

    if (debug_viz_)
    {
        nav_msgs::OccupancyGrid grid;
        grid.header.frame_id = "map";
        grid.header.stamp = ros::Time::now();
        grid.info.resolution = costmap.resolution;
        grid.info.width = costmap.costmap.size().width;
        grid.info.height = costmap.costmap.size().height;
        grid.info.origin.position.x = costmap.origin_x;
        grid.info.origin.position.y = costmap.origin_y;
        grid.info.origin.orientation.w = 1.0;
        grid.data.resize(grid.info.width * grid.info.height);
        for (std::size_t i = 0; i < grid.data.size(); ++i)
        {
            unsigned char cost = 100;
            if (astar.gridMap()[i])
                cost = 255;
            grid.data[i] = cost;
        }
        explore_pub_.publish(grid);
    }

    if (astar_result.success)
    {
        ROS_INFO_STREAM("astar found path of length: " << astar_result.path.size());

        for (auto r_it = astar_result.path.crbegin(); r_it != astar_result.path.crend(); ++r_it)
        {
            const double x = costmap.origin_x + (r_it->x + 0.5) * costmap.resolution;
            const double y = costmap.origin_y + (r_it->y + 0.5) * costmap.resolution;
            const Eigen::Isometry2d p = Eigen::Translation2d(x, y) * Eigen::Rotation2Dd(0);
            result.path.nodes.push_back(p);
        }

        result.path.nodes.push_back(goal);

        orientation_filter_.processPath(result.path.nodes);

        result.cost = astar_result.cost;
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
    const Costmap costmap = buildCostmap(*map_data_, robot_radius_, exponential_weight_, down_sample_);
    return pathCost(path, costmap, neutral_cost_) < std::numeric_limits<double>::max();
}

double AStarPlanner::cost(const navigation_interface::Path& path) const
{
    const Costmap costmap = buildCostmap(*map_data_, robot_radius_, exponential_weight_, down_sample_);
    return pathCost(path, costmap, neutral_cost_);
}

void AStarPlanner::onInitialize(const XmlRpc::XmlRpcValue& parameters)
{
    debug_viz_ = navigation_interface::get_config_with_default_warn<bool>(parameters, "debug_viz", debug_viz_,
                                                                          XmlRpc::XmlRpcValue::TypeBoolean);
    neutral_cost_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "neutral_cost", neutral_cost_, XmlRpc::XmlRpcValue::TypeDouble);
    robot_radius_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "robot_radius", robot_radius_, XmlRpc::XmlRpcValue::TypeDouble);
    exponential_weight_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "exponential_weight", exponential_weight_, XmlRpc::XmlRpcValue::TypeDouble);
    down_sample_ = navigation_interface::get_config_with_default_warn<int>(parameters, "down_sample", down_sample_,
                                                                           XmlRpc::XmlRpcValue::TypeInt);

    orientation_filter_.setMode(1);
    orientation_filter_.setWindowSize(1);

    if (debug_viz_)
    {
        ros::NodeHandle nh;
        pub_ = nh.advertise<nav_msgs::OccupancyGrid>("astar", 100);
        explore_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("explore", 100);
    }
}

void AStarPlanner::onMapDataChanged()
{
}
}
