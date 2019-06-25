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

    double resolution;
    double origin_x;
    double origin_y;
};

Costmap buildCostmap(const gridmap::MapData& map_data, const double robot_radius, const double exponential_weight,
                     const int down_sample)
{
    Costmap grid;

    std::chrono::steady_clock::time_point t0;

    // downsample
    cv::Mat dilated;
    {
        auto lock = map_data.grid.getLock();

        grid.resolution = map_data.grid.dimensions().resolution() * down_sample;

        const int size_x = map_data.grid.dimensions().size().x() / down_sample;
        const int size_y = map_data.grid.dimensions().size().y() / down_sample;

        grid.origin_x = map_data.grid.dimensions().origin().x();
        grid.origin_y = map_data.grid.dimensions().origin().y();

        cv::Mat obstacle_map;
        t0 = std::chrono::steady_clock::now();
        if (down_sample > 1)
        {
            obstacle_map = cv::Mat(size_y, size_x, CV_8U, cv::Scalar(0));
            int index = 0;
            for (int j = 0; j < map_data.grid.dimensions().size().y(); ++j)
            {
                for (int i = 0; i < map_data.grid.dimensions().size().x(); ++i)
                {
                    const int sub_i = i / down_sample;
                    const int sub_j = j / down_sample;
                    const int sub_index = sub_j * size_x + sub_i;
                    if (map_data.grid.cells()[index] == gridmap::OccupancyGrid::OCCUPIED)
                        obstacle_map.at<unsigned char>(sub_index) = 255;
                    ++index;
                }
            }
        }
        else
        {
            obstacle_map = cv::Mat(size_y, size_x, CV_8U,
                                   reinterpret_cast<void*>(const_cast<uint8_t*>(map_data.grid.cells().data())));
        }
        ROS_INFO_STREAM("downsampling took " << std::chrono::duration_cast<std::chrono::duration<double>>(
                                                    std::chrono::steady_clock::now() - t0)
                                                    .count());

        // dilate robot radius
        t0 = std::chrono::steady_clock::now();
        const int cell_inflation_radius = static_cast<int>(2.0 * robot_radius / grid.resolution);
        auto ellipse =
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(cell_inflation_radius, cell_inflation_radius));
        cv::dilate(obstacle_map, dilated, ellipse);
        ROS_INFO_STREAM("dilation took " << std::chrono::duration_cast<std::chrono::duration<double>>(
                                                std::chrono::steady_clock::now() - t0)
                                                .count());
    }

    t0 = std::chrono::steady_clock::now();

    // flip
    cv::bitwise_not(dilated, dilated);

    // allocate
    grid.costmap = cv::Mat(dilated.size(), CV_32F);

    // find obstacle distances
    cv::distanceTransform(dilated, grid.costmap, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);

    // inflate
    grid.costmap = -exponential_weight * grid.costmap * map_data.grid.dimensions().resolution();

    // negative exponent maps values to [1,0)
    cv::exp(grid.costmap, grid.costmap);

    ROS_INFO_STREAM(
        "inflation took "
        << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count());

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
        return result;
    }

    if (goal_x < 0 || goal_x > costmap.costmap.size().width || goal_y < 0 || goal_y > costmap.costmap.size().height)
    {
        ROS_WARN("The goal position is outside the costmap");
        result.outcome = navigation_interface::PathPlanner::Outcome::FAILED;
        return result;
    }

    PathFinder astar(costmap.costmap.size().width, costmap.costmap.size().height,
                     reinterpret_cast<float*>(costmap.costmap.data), neutral_cost_);

    const Coord2D start_coord(static_cast<int>(start_x), static_cast<int>(start_y));
    const Coord2D goal_coord(static_cast<int>(goal_x), static_cast<int>(goal_y));

    const auto t0 = std::chrono::steady_clock::now();

    PathResult astar_result = astar.findPath(start_coord, goal_coord);

    ROS_INFO_STREAM(
        "astar took "
        << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count());

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

        result.path.nodes.push_back(start);

        for (auto r_it = astar_result.path.crbegin(); r_it != astar_result.path.crend(); ++r_it)
        {
            Eigen::Isometry2d p;
            p.translation().x() = costmap.origin_x + (r_it->x + 0.5) * costmap.resolution;
            p.translation().y() = costmap.origin_y + (r_it->y + 0.5) * costmap.resolution;
            p.linear() = start.linear();
            result.path.nodes.push_back(p);
        }

        result.path.nodes.push_back(goal);

        // Rotation guestimation
        // This is expensive and most of the time the trajectory planner has a better idea anyway
        //        const int window_size = 4;
        //        for (std::size_t i=1; i < result.path.nodes.size() - 1; ++i)
        //        {
        //            const std::size_t index1 = std::min(result.path.nodes.size() - 1, i + window_size);

        //            const double x0 = result.path.nodes[i].translation().x();
        //            const double y0 = result.path.nodes[i].translation().y();
        //            const double x1 = result.path.nodes[index1].translation().x();
        //            const double y1 = result.path.nodes[index1].translation().y();

        //            const double angle = atan2(y1 - y0, x1 - x0);
        //            result.path.nodes[i].linear() = Eigen::Rotation2Dd(angle).matrix();
        //        }

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
