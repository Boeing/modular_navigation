#include <astar_planner/astar.h>
#include <astar_planner/plugin.h>
#include <gridmap/operations/rasterize.h>
#include <gridmap/operations/raytrace.h>
#include <nav_msgs/OccupancyGrid.h>
#include <navigation_interface/params.h>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Geometry>
#include <chrono>

PLUGINLIB_EXPORT_CLASS(astar_planner::AStarPlanner, navigation_interface::PathPlanner)

namespace astar_planner
{

namespace
{

double pathCost(const navigation_interface::Path& path, const astar_planner::CollisionChecker& collision_checker)
{
    for (std::size_t i = 0; i < path.nodes.size(); ++i)
    {
        const State3D state{path.nodes[i].translation().x(), path.nodes[i].translation().y(),
                            Eigen::Rotation2Dd(path.nodes[i].linear()).smallestAngle()};
        if (!collision_checker.isValid(state))
        {
            return std::numeric_limits<double>::max();
        }
    }

    double cost = 0.0;
    for (std::size_t i = 0; i < path.nodes.size() - 1; ++i)
    {
        const Eigen::Vector2d dir = path.nodes[i + 1].translation() - path.nodes[i].translation();
        const Eigen::Vector2d dir_wrt_robot = path.nodes[i].linear().inverse() * dir;

        double x_cost = dir_wrt_robot.x();

        // strafing is very expensive
        double y_cost = 4.0 * dir_wrt_robot.y();

        // backwards motion is expensive
        if (dir_wrt_robot.x() > 0)
            x_cost *= 1.5;

        cost += x_cost + y_cost;
        cost += std::abs(wrapAngle(Eigen::Rotation2Dd(path.nodes[i + 1].linear()).smallestAngle() -
                                   Eigen::Rotation2Dd(path.nodes[i].linear()).smallestAngle())) /
                M_PI;
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

// cppcheck-suppress unusedFunction
navigation_interface::PathPlanner::Result AStarPlanner::plan(const Eigen::Isometry2d& start,
                                                             const Eigen::Isometry2d& goal)
{
    navigation_interface::PathPlanner::Result result;

    costmap_ = buildCostmap(*map_data_, robot_radius_);
    ROS_ASSERT(traversal_cost_);
    costmap_->traversal_cost = traversal_cost_;
    const astar_planner::CollisionChecker collision_checker(*costmap_, offsets_);

    const size_t max_iterations = 1e5;
    const double linear_resolution = 0.02;
    const double angular_resolution = 2 * M_PI / 16;

    const auto t0 = std::chrono::steady_clock::now();

    const astar_planner::PathResult astar_result =
        astar_planner::hybridAStar(start, goal, max_iterations, *costmap_, collision_checker,
                                   conservative_robot_radius_, linear_resolution, angular_resolution);

    ROS_INFO_STREAM(
        "astar took "
        << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count()
        << " iterations: " << astar_result.iterations);

    if (debug_viz_)
    {
        if (explore_pub_.getNumSubscribers() > 0)
        {
            visualization_msgs::MarkerArray ma;

            visualization_msgs::Marker delete_all;
            delete_all.action = visualization_msgs::Marker::DELETEALL;
            ma.markers.push_back(delete_all);

            const auto now = ros::Time::now();

            auto make_cylider = [&ma, &now](const std_msgs::ColorRGBA& color, const double radius, const double height,
                                            const Eigen::Isometry3d& pose) {
                visualization_msgs::Marker marker;
                marker.ns = "cylinder";
                marker.id = static_cast<int>(ma.markers.size());
                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.action = visualization_msgs::Marker::ADD;
                marker.header.stamp = now;
                marker.header.frame_id = "map";
                marker.frame_locked = true;
                marker.scale.x = radius;
                marker.scale.y = radius;
                marker.scale.z = height;
                marker.color = color;
                const Eigen::Quaterniond qt(pose.linear());
                marker.pose.orientation.w = qt.w();
                marker.pose.orientation.x = qt.x();
                marker.pose.orientation.y = qt.y();
                marker.pose.orientation.z = qt.z();
                marker.pose.position.x = pose.translation().x();
                marker.pose.position.y = pose.translation().y();
                marker.pose.position.z = pose.translation().z();
                return marker;
            };

            std_msgs::ColorRGBA red;
            red.r = 1.0;
            red.a = 1.0;

            std_msgs::ColorRGBA green;
            green.g = 1.0;
            green.a = 1.0;

            std_msgs::ColorRGBA blue;
            blue.b = 1.0;
            blue.a = 1.0;

            const double length = 0.02;
            const double radius = 0.002;

            for (auto node : astar_result.explore_3d)
            {
                const Eigen::Vector3d _position(node.second->state.x, node.second->state.y, 0.0);
                const Eigen::Quaterniond _rotation(
                    Eigen::AngleAxisd(node.second->state.theta, Eigen::Vector3d::UnitZ()));
                const Eigen::Isometry3d pose = Eigen::Translation3d(_position) * _rotation;

                // X Axis
                const Eigen::Isometry3d x_pose = pose * (Eigen::Translation3d(length / 2.0, 0, 0) *
                                                         Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()));
                ma.markers.push_back(make_cylider(red, radius, length, x_pose));

                // Y Axis
                const Eigen::Isometry3d y_pose = pose * (Eigen::Translation3d(0, length / 2.0, 0) *
                                                         Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()));
                ma.markers.push_back(make_cylider(green, radius, length, y_pose));

                // Z Axis
                const Eigen::Isometry3d z_pose =
                    pose * (Eigen::Translation3d(0, 0, length / 2.0) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));
                ma.markers.push_back(make_cylider(blue, radius, length, z_pose));

                if (node.second->parent)
                {
                    visualization_msgs::Marker marker;
                    marker.ns = "line";
                    marker.id = static_cast<int>(ma.markers.size());
                    marker.type = visualization_msgs::Marker::LINE_STRIP;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.header.stamp = now;
                    marker.header.frame_id = "map";
                    marker.frame_locked = true;
                    marker.scale.x = 0.002;
                    marker.scale.y = 0.002;
                    marker.scale.z = 0.002;
                    marker.color.g = 1.0;
                    marker.color.a = 1.0;
                    marker.pose.orientation.w = 1;
                    geometry_msgs::Point p1;
                    p1.x = _position.x();
                    p1.y = _position.y();
                    geometry_msgs::Point p2;
                    p2.x = node.second->parent->state.x;
                    p2.y = node.second->parent->state.y;
                    marker.points.push_back(p1);
                    marker.points.push_back(p2);
                    ma.markers.push_back(marker);
                }
            }

            explore_pub_.publish(ma);
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

        result.path.nodes.push_back(goal);

        result.cost = astar_result.path.front()->cost_so_far;
        result.outcome = navigation_interface::PathPlanner::Outcome::SUCCESSFUL;
    }
    else
    {
        if (astar_result.start_in_collision)
            ROS_WARN("Start in collision!");
        if (astar_result.goal_in_collision)
            ROS_WARN("Goal in collision!");
        result.outcome = navigation_interface::PathPlanner::Outcome::FAILED;
    }
    return result;
}

// cppcheck-suppress unusedFunction
bool AStarPlanner::valid(const navigation_interface::Path& path) const
{
    // assume this is called immediately after plan to re-use the data structures
    ROS_ASSERT(costmap_);

    const astar_planner::CollisionChecker collision_checker(*costmap_, offsets_);
    return pathCost(path, collision_checker) < std::numeric_limits<double>::max();
}

double AStarPlanner::cost(const navigation_interface::Path& path) const
{
    // assume this is called immediately after plan to re-use the data structures
    ROS_ASSERT(costmap_);

    const astar_planner::CollisionChecker collision_checker(*costmap_, offsets_);
    return pathCost(path, collision_checker);
}

// cppcheck-suppress unusedFunction
void AStarPlanner::onInitialize(const XmlRpc::XmlRpcValue& parameters)
{
    debug_viz_ = navigation_interface::get_config_with_default_warn<bool>(parameters, "debug_viz", debug_viz_,
                                                                          XmlRpc::XmlRpcValue::TypeBoolean);
    robot_radius_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "robot_radius", robot_radius_, XmlRpc::XmlRpcValue::TypeDouble);
    conservative_robot_radius_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "conservative_robot_radius", conservative_robot_radius_, XmlRpc::XmlRpcValue::TypeDouble);
    max_holonomic_distance_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "max_holonomic_distance", max_holonomic_distance_, XmlRpc::XmlRpcValue::TypeDouble);
    max_reverse_distance_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "max_reverse_distance", max_reverse_distance_, XmlRpc::XmlRpcValue::TypeDouble);

    avoid_zone_cost_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "avoid_zone_cost", avoid_zone_cost_, XmlRpc::XmlRpcValue::TypeDouble);

    path_cost_ = navigation_interface::get_config_with_default_warn<double>(parameters, "path_cost", path_cost_,
                                                                            XmlRpc::XmlRpcValue::TypeDouble);

    if (parameters.hasMember("robot_radius_offsets"))
    {
        offsets_ = navigation_interface::get_point_list(parameters, "robot_radius_offsets");
    }
    else
    {
        offsets_ = {{-0.268, 0.000},  {0.268, 0.000}, {0.265, -0.185}, {0.077, -0.185}, {-0.077, -0.185},
                    {-0.265, -0.185}, {0.265, 0.185}, {-0.265, 0.185}, {-0.077, 0.185}, {0.077, 0.185}};
    }

    if (debug_viz_)
    {
        ros::NodeHandle nh("~");
        explore_pub_ = nh.advertise<visualization_msgs::MarkerArray>("expansion", 100);
    }
}

// cppcheck-suppress unusedFunction
void AStarPlanner::onMapDataChanged()
{
    ROS_INFO("Building avoid zone traversal costmap");

    // need to generate a data structure for zones
    traversal_cost_ = std::make_shared<cv::Mat>(map_data_->grid.dimensions().size().y(),
                                                map_data_->grid.dimensions().size().x(), CV_32F, cv::Scalar(1.0));

    for (const hd_map::Zone& zone : map_data_->hd_map.zones)
    {
        if (zone.zone_type == hd_map::Zone::AVOID_ZONE)
        {
            int min_x = std::numeric_limits<int>::max();
            int max_x = 0;

            int min_y = std::numeric_limits<int>::max();
            int max_y = 0;

            std::vector<Eigen::Array2i> map_polygon;
            for (const geometry_msgs::Point32& p : zone.polygon.points)
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
            const float avoid_zone_cost = static_cast<float>(avoid_zone_cost_);
            auto append_raster = [&traversal_cost, avoid_zone_cost](const int x, const int y) {
                if (x >= 0 && x < traversal_cost.cols && y >= 0 && y < traversal_cost.rows)
                {
                    traversal_cost.at<float>(y, x) = avoid_zone_cost;
                }
            };

            gridmap::rasterPolygonFill(append_raster, connected, min_x, max_x, min_y, max_y);
        }
    }

    // TODO turn this back on when it is more stable with the heuristic
    /*
        for (const hd_map::Path& path : map_data_->hd_map.paths)
        {
            for (size_t i = 0; i < path.nodes.size() - 1; ++i)
            {
                const std::string first_id = path.nodes[i];
                const std::string next_id = path.nodes[i + 1];

                auto first_it = std::find_if(map_data_->hd_map.nodes.begin(), map_data_->hd_map.nodes.end(),
                                             [&first_id](const hd_map::Node& n) { return n.id == first_id; });
                ROS_ASSERT(first_it != map_data_->hd_map.nodes.end());
                hd_map::Node start_node = *first_it;

                auto next_it = std::find_if(map_data_->hd_map.nodes.begin(), map_data_->hd_map.nodes.end(),
                                            [&next_id](const hd_map::Node& n) { return n.id == next_id; });
                ROS_ASSERT(next_it != map_data_->hd_map.nodes.end());
                hd_map::Node end_node = *next_it;

                const Eigen::Array2i start_mp = map_data_->grid.dimensions().getCellIndex({start_node.x, start_node.y});
                const Eigen::Array2i end_mp = map_data_->grid.dimensions().getCellIndex({end_node.x, end_node.y});

                cv::line(*traversal_cost_, cv::Point(start_mp[0], start_mp[1]), cv::Point(end_mp[0], end_mp[1]),
       path_cost_, 10);
            }
        }
        */
}

}  // namespace astar_planner
