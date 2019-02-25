#include <astar_planner/astar.h>
#include <astar_planner/plugin.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <pluginlib/class_list_macros.h>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/simplify.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

PLUGINLIB_EXPORT_CLASS(astar_planner::AStarPlanner, nav_core::BaseGlobalPlanner)

namespace astar_planner
{

AStarPlanner::AStarPlanner()
    : tf_buffer_(nullptr), global_costmap_(nullptr), local_costmap_(nullptr), publish_potential_(true),
      neutral_cost_(10.0)
{
}

AStarPlanner::~AStarPlanner()
{
}

void AStarPlanner::initialize(const std::string& name, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                              const std::shared_ptr<costmap_2d::Costmap2DROS>& global_costmap,
                              const std::shared_ptr<costmap_2d::Costmap2DROS>& local_costmap)
{
    tf_buffer_ = tf_buffer;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    ros::NodeHandle private_nh("~/" + name);

    neutral_cost_ = private_nh.param("neutral_cost", 30.0);

    orientation_filter_.setMode(1);
    orientation_filter_.setWindowSize(1);

    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);
}

nav_core::PlanResult AStarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                            const geometry_msgs::PoseStamped& goal)
{
    std::lock_guard<std::mutex> lock(mutex_);

    nav_core::PlanResult result;

    // const std::string robot_frame = local_costmap_->getBaseFrameID();
    // const std::string local_frame = local_costmap_->getGlobalFrameID();
    const std::string global_frame = global_costmap_->getGlobalFrameID();
    const ros::Time now = ros::Time::now();

    // TODO transform local frame to global frame when super imposing map

    if (goal.header.frame_id != global_frame)
    {
        ROS_ERROR_STREAM("Goal must be in global_frame: " << global_frame);
        result.success = false;
        return result;
    }

    if (start.header.frame_id != global_frame)
    {
        ROS_ERROR_STREAM("Start must be in global_frame: " << global_frame);
        result.success = false;
        return result;
    }

    //
    // Copy the local costmap data
    // We can't spend too much time doing this because the local planner needs access to it
    //
    cv::Mat local_costmap;
    std::vector<unsigned char> local_costmap_data;
    unsigned int l_x_size;
    unsigned int l_y_size;
    double l_resolution;
    double l_origin_x;
    double l_origin_y;
    {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> g(*local_costmap_->getCostmap()->getMutex());
        l_x_size = local_costmap_->getCostmap()->getSizeInCellsX();
        l_y_size = local_costmap_->getCostmap()->getSizeInCellsY();
        l_resolution = local_costmap_->getCostmap()->getResolution();
        l_origin_x = local_costmap_->getCostmap()->getOriginX();
        l_origin_y = local_costmap_->getCostmap()->getOriginY();
        const unsigned int _size = l_x_size * l_y_size;
        local_costmap_data.resize(_size);
        std::copy_n(local_costmap_->getCostmap()->getCharMap(), _size, local_costmap_data.begin());
        local_costmap = cv::Mat(l_y_size, l_x_size, CV_8UC1, local_costmap_data.data());
    }

    //
    // Global costmap properties
    //
    const unsigned int g_x_size = global_costmap_->getCostmap()->getSizeInCellsX();
    const unsigned int g_y_size = global_costmap_->getCostmap()->getSizeInCellsY();
    const double g_resolution = global_costmap_->getCostmap()->getResolution();
    const double g_origin_x = global_costmap_->getCostmap()->getOriginX();
    const double g_origin_y = global_costmap_->getCostmap()->getOriginY();

    //
    // Merged map
    // resolution changes
    // origin stays the same
    //
    const double scale = g_resolution / l_resolution;
    const unsigned int mm_size_x = g_x_size * scale;
    const unsigned int mm_size_y = g_y_size * scale;
    const double mm_origin_x = g_origin_x;
    const double mm_origin_y = g_origin_y;
    const double mm_resolution = l_resolution;

    ROS_DEBUG_STREAM("l_x_size: " << l_x_size);
    ROS_DEBUG_STREAM("l_y_size: " << l_y_size);
    ROS_DEBUG_STREAM("l_resolution: " << l_resolution);
    ROS_DEBUG_STREAM("l_origin_x: " << l_origin_x);
    ROS_DEBUG_STREAM("l_origin_y: " << l_origin_y);

    ROS_DEBUG_STREAM("g_x_size: " << g_x_size);
    ROS_DEBUG_STREAM("g_y_size: " << g_y_size);
    ROS_DEBUG_STREAM("g_resolution: " << g_resolution);
    ROS_DEBUG_STREAM("g_origin_x: " << g_origin_x);
    ROS_DEBUG_STREAM("g_origin_y: " << g_origin_y);

    ROS_DEBUG_STREAM("mm_size_x: " << mm_size_x);
    ROS_DEBUG_STREAM("mm_size_y: " << mm_size_y);
    ROS_DEBUG_STREAM("mm_resolution: " << mm_resolution);
    ROS_DEBUG_STREAM("mm_origin_x: " << mm_origin_x);
    ROS_DEBUG_STREAM("mm_origin_y: " << mm_origin_y);

    //
    // Copy the global costmap data but resize to match the local costmap resolution
    //
    cv::Mat merged_costmap;
    cv::Mat global_costmap(g_y_size, g_x_size, CV_8UC1, global_costmap_->getCostmap()->getCharMap());
    cv::resize(global_costmap, merged_costmap, cv::Size(mm_size_x, mm_size_y), 0, 0, cv::INTER_LINEAR);

    //
    // Super impose the local costmap data
    //
    int merged_local_x = (l_origin_x - mm_origin_x) / mm_resolution;
    int merged_local_y = (l_origin_y - mm_origin_y) / mm_resolution;
    local_costmap.copyTo(
        merged_costmap(cv::Rect(merged_local_x, merged_local_y, local_costmap.cols, local_costmap.rows)));
    outlineMap(merged_costmap.data, mm_size_x, mm_size_y, costmap_2d::LETHAL_OBSTACLE);

    //    cv::imwrite("/home/boeing/local.png", local_costmap);
    //    cv::imwrite("/home/boeing/global.png", global_costmap);
    //    cv::imwrite("/home/boeing/merged.png", merged_costmap);

    //
    // Calculate start and end map coordinates
    //
    const double start_x = (start.pose.position.x - mm_origin_x) / mm_resolution - 0.5;
    const double start_y = (start.pose.position.y - mm_origin_y) / mm_resolution - 0.5;
    const double goal_x = (goal.pose.position.x - mm_origin_x) / mm_resolution - 0.5;
    const double goal_y = (goal.pose.position.y - mm_origin_y) / mm_resolution - 0.5;

    ROS_DEBUG_STREAM("START: " << start.pose.position.x << " " << start.pose.position.y << " -> " << start_x << " "
                               << start_y);
    ROS_DEBUG_STREAM("GOAL: " << goal.pose.position.x << " " << goal.pose.position.y << " -> " << goal_x << " "
                              << goal_y);

    if (start_x < 0 || start_x > mm_size_x || start_y < 0 || start_y > mm_size_y)
    {
        ROS_WARN("The robot's start position is outside the global costmap");
        result.success = false;
        return result;
    }

    if (goal_x < 0 || goal_x > mm_size_x || goal_y < 0 || goal_y > mm_size_y)
    {
        ROS_WARN("The goal position is outside the global costmap");
        result.success = false;
        return result;
    }

    const uint8_t obstacle_threshold = 253;
    PathFinder astar(mm_size_x, mm_size_y, merged_costmap.data, obstacle_threshold, neutral_cost_);

    const Coord2D start_coord(static_cast<int>(start_x), static_cast<int>(start_y));
    const Coord2D goal_coord(static_cast<int>(goal_x), static_cast<int>(goal_y));

    const auto t0 = std::chrono::steady_clock::now();

    PathResult astar_result = astar.findPath(start_coord, goal_coord);

    ROS_INFO_STREAM(
        "A-STAR took "
        << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count());

    if (publish_potential_)
    {
        nav_msgs::OccupancyGrid grid;

        grid.header.frame_id = global_frame;
        grid.header.stamp = now;
        grid.info.resolution = mm_resolution;
        grid.info.width = mm_size_x;
        grid.info.height = mm_size_y;
        grid.info.origin.position.x = mm_origin_x;
        grid.info.origin.position.y = mm_origin_y;
        grid.info.origin.orientation.w = 1.0;

        grid.data.resize(mm_size_x * mm_size_y);

        double max = 0.0;
        for (unsigned int i = 0; i < grid.data.size(); i++)
        {
            const double p = astar.gridMap()[i].cost;
            if (p > max)
            {
                max = p;
            }
        }

        for (unsigned int i = 0; i < grid.data.size(); i++)
        {
            if (astar.gridMap()[i].cost >= std::numeric_limits<double>::max())
            {
                grid.data[i] = -1;
            }
            else
            {
                grid.data[i] = static_cast<char>(astar.gridMap()[i].cost * 100.0 / max);
            }
        }
        potential_pub_.publish(grid);
    }

    if (astar_result.success)
    {
        ROS_INFO("Found a plan");
        std::vector<std::pair<double, double>> simplified_path;
        {
            typedef boost::geometry::model::d2::point_xy<double> xy;

            boost::geometry::model::linestring<xy> line;
            boost::geometry::append(line, xy(goal_x, goal_y));
            for (const Coord2D& coord : astar_result.path)
                boost::geometry::append(line, xy(coord.x, coord.y));
            boost::geometry::append(line, xy(start_x, start_y));

            boost::geometry::model::linestring<xy> simplified;
            const double step_size = 0.04 / mm_resolution;
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
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = now;
            pose.header.frame_id = global_frame;
            pose.pose.position.x = mm_origin_x + (r_it->first + 0.5) * mm_resolution;
            pose.pose.position.y = mm_origin_y + (r_it->second + 0.5) * mm_resolution;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            result.plan.push_back(pose);
        }
        geometry_msgs::PoseStamped goal_copy = goal;
        goal_copy.header.stamp = now;
        result.plan.push_back(goal_copy);

        // add orientations if needed
        orientation_filter_.processPath(result.plan);

        // publish the plan for visualization purposes
        if (!result.plan.empty())
        {
            nav_msgs::Path gui_path;
            gui_path.header = result.plan.front().header;
            gui_path.poses = result.plan;
            plan_pub_.publish(gui_path);
        }

        result.cost = astar_result.cost;
        result.success = true;
    }
    else
    {
        ROS_ERROR("Failed to get a plan");
        result.success = false;
    }

    return result;
}

double AStarPlanner::cost(const std::vector<geometry_msgs::PoseStamped>&)
{
    return 0.0;
}

void AStarPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value)
{
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}
}
