#include <Eigen/Geometry>

#include <astar_planner/astar.h>
#include <astar_planner/plugin.h>
#include <astar_planner/visualisation.h>
#include <graph_map/Region.h>
#include <graph_map/Zone.h>
#include <gridmap/operations/rasterize.h>
#include <gridmap/operations/raytrace.h>
#include <nav_msgs/OccupancyGrid.h>
#include <navigation_interface/params.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>

#include <chrono>

PLUGINLIB_EXPORT_CLASS(astar_planner::AStarPlanner, navigation_interface::PathPlanner)

namespace astar_planner
{

namespace
{

double pathCost(const navigation_interface::Path& path, const astar_planner::CollisionChecker& collision_checker,
                const double backwards_mult, const double strafe_mult, const double rotation_mult)
{
    double cost = 0.0;
    for (std::size_t i = 0; i < path.nodes.size() - 1; ++i)
    {
        const State3D state{path.nodes[i].translation().x(), path.nodes[i].translation().y(),
                            Eigen::Rotation2Dd(path.nodes[i].linear()).smallestAngle()};
        if (!collision_checker.isValid(state))
            return std::numeric_limits<double>::max();

        const Eigen::Vector2d dir = path.nodes[i + 1].translation() - path.nodes[i].translation();
        const Eigen::Vector2d dir_wrt_robot = path.nodes[i].linear().inverse() * dir;
        const Eigen::Rotation2Dd rotation(path.nodes[i].linear().inverse() * path.nodes[i + 1].linear());

        const double x_cost = std::abs((dir_wrt_robot[0] > 0) ? dir_wrt_robot[0] : backwards_mult * dir_wrt_robot[0]);
        const double y_cost = std::abs(strafe_mult * dir_wrt_robot[1]);

        const Eigen::Array2i map_point = collision_checker.costmap().getCellIndex(path.nodes[i].translation());

        const double collision_cost = collisionCost(map_point.x(), map_point.y(), collision_checker);
        const double traversal_cost = traversalCost(map_point.x(), map_point.y(), collision_checker.costmap());

        const double d_to_collision_m = collision_checker.clearance(state) * collision_checker.costmap().resolution;
        const double rotation_collision_cost = rotationCollisionCost(d_to_collision_m);

        cost += (x_cost + y_cost) * traversal_cost * collision_cost;
        cost += std::abs(rotation.smallestAngle()) * rotation_mult * rotation_collision_cost;
    }
    return cost;
}
}  // namespace

AStarPlanner::AStarPlanner()
{
}

AStarPlanner::~AStarPlanner()
{
}

navigation_interface::PathPlanner::Result
    // cppcheck-suppress unusedFunction
    AStarPlanner::plan(const gridmap::AABB& local_region, const Eigen::Isometry2d& start, const Eigen::Isometry2d& goal,
                       const GoalSampleSettings& sample, const double avoid_distance, const double backwards_mult,
                       const double strafe_mult, const double rotation_mult)
{
    navigation_interface::PathPlanner::Result result;

    // Add the avoid_distance to the robot_radius_ (little circles) and the conservative_robot_radius_ (big circle)
    const double _avoid_distance = std::max(0.0, avoid_distance);
    const double inflated_robot_radius = robot_radius_ + _avoid_distance;
    const double inflated_conservative_robot_radius = conservative_robot_radius_ + _avoid_distance;

    {
        // cppcheck-suppress unreadVariable
        auto lock = map_data_->grid.getReadLock();
        costmap_ = std::make_shared<Costmap>(*map_data_, inflated_robot_radius);
    }

    // clear the robot footprint
    const int radius_px = static_cast<int>(inflated_robot_radius / costmap_->resolution);
    for (const auto& offset : offsets_)
    {
        const Eigen::Vector2d p = start * offset;
        const Eigen::Array2i map_cell = costmap_->getCellIndex({p.x(), p.y()});
        cv::circle(costmap_->obstacle_map, cv::Point(map_cell.x(), map_cell.y()), radius_px, cv::Scalar(0), -1);
    }

    costmap_->processObstacleMap(local_region);

    rcpputils::assert_true(traversal_cost_);
    costmap_->traversal_cost = traversal_cost_;
    const astar_planner::CollisionChecker collision_checker(*costmap_, offsets_, inflated_conservative_robot_radius);

    const size_t max_iterations = 3e5;
    const double linear_resolution = 0.04;
    const double angular_resolution = M_PI / 16;

    const double backwards_mult_applied = backwards_mult > 0.0 ? backwards_mult : backwards_mult_;
    const double strafe_mult_applied = strafe_mult > 0.0 ? strafe_mult : strafe_mult_;
    const double rotation_mult_applied = rotation_mult > 0.0 ? rotation_mult : rotation_mult_;

    const auto t0 = std::chrono::steady_clock::now();

    const astar_planner::PathResult astar_result = astar_planner::hybridAStar(
        start, goal, max_iterations, collision_checker, linear_resolution, angular_resolution, sample,
        backwards_mult_applied, strafe_mult_applied, rotation_mult_applied);

    RCLCPP_INFO_STREAM(rclcpp::get_logger(""),
        "Hybrid A Star took "
        << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count()
        << " iterations: " << astar_result.iterations << " nodes: " << astar_result.explore_3d.size());

    if (debug_viz_)
    {
        //if (explore_pub_.getNumSubscribers() > 0)
        if (explore_pub_.get_subscription_count() > 0) // 
        {
            cv::Mat disp = astar_planner::visualise(*costmap_, astar_result);
            cv::cvtColor(disp, disp, cv::COLOR_BGR2GRAY);

            nav_msgs::OccupancyGrid og;
            //og.header.stamp = ros::Time::now();
            og.header.stamp = node->get_clock()->now();//.nanoseconds()
            og.info.resolution = costmap_->resolution;
            og.info.width = costmap_->width;
            og.info.height = costmap_->height;
            og.data.resize(static_cast<size_t>(og.info.width * og.info.height));
            og.info.origin.position.x = costmap_->origin_x;
            og.info.origin.position.y = costmap_->origin_y;
            og.info.origin.orientation.w = 1.0;

            unsigned char* input = (unsigned char*)(disp.data);
            std::copy_n(input, og.data.size(), &og.data[0]);

            explore_pub_.publish(og);
        }
    }

    if (astar_result.success)
    {
        result.path.nodes.push_back(start);

        for (auto r_it = astar_result.path.crbegin(); r_it != astar_result.path.crend(); ++r_it)
        {
            const Eigen::Isometry2d p =
                Eigen::Translation2d((*r_it)->state.x, (*r_it)->state.y) * Eigen::Rotation2Dd((*r_it)->state.theta);
            result.path.nodes.push_back(p);
        }

        result.cost = astar_result.path.front()->cost_so_far;
        result.outcome = navigation_interface::PathPlanner::Outcome::SUCCESSFUL;
    }
    else
    {
        if (astar_result.start_in_collision)
            RCLCPP_WARN(rclcpp::get_logger(""),"Start in collision!");
        if (astar_result.goal_in_collision)
            RCLCPP_WARN(rclcpp::get_logger(""),"Goal in collision!");
        result.outcome = navigation_interface::PathPlanner::Outcome::FAILED;
    }
    return result;
}

bool AStarPlanner::valid(const navigation_interface::Path& path, const double avoid_distance) const
{
    const double inflated_conservative_robot_radius = conservative_robot_radius_ + std::max(0.0, avoid_distance);

    // assume this is called immediately after plan to re-use the data structures
    rcpputils::assert_true(costmap_);

    const astar_planner::CollisionChecker collision_checker(*costmap_, offsets_, inflated_conservative_robot_radius);
    return pathCost(path, collision_checker, backwards_mult_, strafe_mult_, rotation_mult_) <
           std::numeric_limits<double>::max();
}

double AStarPlanner::cost(const navigation_interface::Path& path, const double avoid_distance) const
{
    const double inflated_conservative_robot_radius = conservative_robot_radius_ + std::max(0.0, avoid_distance);

    // assume this is called immediately after plan to re-use the data structures
    rcpputils::assert_true(costmap_);

    const astar_planner::CollisionChecker collision_checker(*costmap_, offsets_, inflated_conservative_robot_radius);
    return pathCost(path, collision_checker, backwards_mult_, strafe_mult_, rotation_mult_);
}

// cppcheck-suppress unusedFunction
void AStarPlanner::onInitialize(const YAML::Node& parameters)
{
    debug_viz_ = parameters["debug_viz"].as<bool>(debug_viz_);
    robot_radius_ = parameters["robot_radius"].as<double>(robot_radius_);
    conservative_robot_radius_ = parameters["conservative_robot_radius"].as<double>(conservative_robot_radius_);

    backwards_mult_ = parameters["backwards_mult"].as<double>(backwards_mult_);
    strafe_mult_ = parameters["strafe_mult"].as<double>(strafe_mult_);
    rotation_mult_ = parameters["rotation_mult"].as<double>(rotation_mult_);

    offsets_ = navigation_interface::get_point_list(parameters, "robot_radius_offsets",
                                                    {{-0.268, 0.000},
                                                     {0.268, 0.000},
                                                     {0.265, -0.185},
                                                     {0.077, -0.185},
                                                     {-0.077, -0.185},
                                                     {-0.265, -0.185},
                                                     {0.265, 0.185},
                                                     {-0.265, 0.185},
                                                     {-0.077, 0.185},
                                                     {0.077, 0.185}});

    if (debug_viz_)
    {
        //ros::NodeHandle nh("~");
        //explore_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("expansion", 100);
        // TODO What is the scope on this? consider moving it to plugin.h
        auto explore_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("expansion", 100);
    }
}

// cppcheck-suppress unusedFunction
void AStarPlanner::onMapDataChanged()
{
    RCLCPP_INFO(rclcpp::get_logger(""), "Adding cost-modifying zones to traversal costmap");

    // need to generate a data structure for zones
    traversal_cost_ = std::make_shared<cv::Mat>(map_data_->grid.dimensions().size().y(),
                                                map_data_->grid.dimensions().size().x(), CV_32F, cv::Scalar(1.0));

    for (const graph_map::Zone& zone : map_data_->zones)
    {
        // By default, cost is zero
        if (zone.drivable && zone.cost > 0.0)
        {
            for (const graph_map::Region& region : zone.regions)
            {
                const geometry_msgs::Polygon& polygon = region.polygon;
                int min_x = std::numeric_limits<int>::max();
                int max_x = 0;

                int min_y = std::numeric_limits<int>::max();
                int max_y = 0;

                std::vector<Eigen::Array2i> map_polygon;
                for (const geometry_msgs::Point32& p : polygon.points)
                {
                    const Eigen::Array2i map_point = map_data_->grid.dimensions().getCellIndex({p.x, p.y});
                    min_x = std::min(map_point.x(), min_x);
                    max_x = std::max(map_point.x(), max_x);
                    min_y = std::min(map_point.y(), min_y);
                    max_y = std::max(map_point.y(), max_y);
                    map_polygon.push_back(map_point);
                }
                if (!map_polygon.empty())
                    map_polygon.push_back(map_polygon.front());

                const std::vector<Eigen::Array2i> connected = gridmap::connectPolygon(map_polygon);

                cv::Mat& traversal_cost = *(traversal_cost_.get());
                const float cost = static_cast<float>(zone.cost);
                auto append_raster = [&traversal_cost, cost](const int x, const int y) {
                    if (x >= 0 && x < traversal_cost.cols && y >= 0 && y < traversal_cost.rows)
                    {
                        traversal_cost.at<float>(y, x) = cost;
                    }
                };

                gridmap::rasterPolygonFill(append_raster, connected, min_x, max_x, min_y, max_y);
            }
        }
    }

    // blue the traversal cost map to help provide a smooth manifold for planning
    cv::GaussianBlur(*traversal_cost_, *traversal_cost_, cv::Size(11, 11), 0);
}

}  // namespace astar_planner
