#include <navigation_interface/params.h>
#include <pluginlib/class_list_macros.h>
#include <sim_band_planner/plugin.h>
#include <visualization_msgs/MarkerArray.h>

PLUGINLIB_EXPORT_CLASS(sim_band_planner::SimBandPlanner, navigation_interface::TrajectoryPlanner)

namespace sim_band_planner
{

namespace
{

std_msgs::ColorRGBA distanceToColor(const double distance)
{
    std_msgs::ColorRGBA color;
    color.a = 0.25f;

    if (distance <= 0)
    {
        color.r = 1.0f;
    }
    else if (distance < 0.1)
    {
        color.r = 1.0f;
        color.g = 0.5f;
    }
    else if (distance < 0.2)
    {
        color.r = 1.0f;
        color.g = 1.0f;
    }
    else
    {
        color.g = 1.0f;
    }
    return color;
}

}  // namespace

SimBandPlanner::SimBandPlanner()
{
}

SimBandPlanner::~SimBandPlanner()
{
}

// cppcheck-suppress unusedFunction
bool SimBandPlanner::setPath(const navigation_interface::Path& path)
{
    if (path.nodes.empty())
        return false;

    moving_window_.reset(new MovingWindow(path, offsets_));

    return true;
}

// cppcheck-suppress unusedFunction
void SimBandPlanner::clearPath()
{
    moving_window_.reset();
}

// cppcheck-suppress unusedFunction
boost::optional<std::string> SimBandPlanner::pathId() const
{
    if (moving_window_)
        return boost::optional<std::string>(moving_window_->nominal_path.id);
    else
        return {};
}

boost::optional<navigation_interface::Path> SimBandPlanner::path() const
{
    if (moving_window_)
        return boost::optional<navigation_interface::Path>(moving_window_->nominal_path);
    else
        return {};
}

navigation_interface::TrajectoryPlanner::Result
    // cppcheck-suppress unusedFunction
    SimBandPlanner::plan(const gridmap::AABB& local_region, const navigation_interface::KinodynamicState& robot_state,
                         const Eigen::Isometry2d& map_to_odom)
{
    navigation_interface::TrajectoryPlanner::Result result;

    if (!moving_window_)
    {
        result.outcome = navigation_interface::TrajectoryPlanner::Outcome::FAILED;
        return result;
    }

    try
    {
        // Take a window of the planning scene at the robot pose
        const Eigen::Isometry2d robot_pose = map_to_odom * robot_state.pose;

        moving_window_->updateWindow(robot_pose, robot_state.velocity, max_window_length_);

        auto lock = map_data_->grid.getLock();
        gridmap::Grid2D<uint8_t> local_grid(map_data_->grid, local_region);
        lock.unlock();

        // Construct and cv::Mat from the costmap data
        const cv::Mat cv_im = cv::Mat(local_grid.dimensions().size().y(), local_grid.dimensions().size().x(), CV_8U,
                                      reinterpret_cast<void*>(local_grid.cells().data()));

        DistanceField distance_field(cv_im, local_grid.dimensions().origin().x(), local_grid.dimensions().origin().y(),
                                     local_grid.dimensions().resolution(), robot_radius_);

        Band sim_band(offsets_);

        // Add the robot position to the front of the band
        sim_band.nodes.push_back(Node(robot_pose, sim_band.radius_offsets));

        // The first moving window node is the previous segment (exclude from optimization)
        ROS_ASSERT(moving_window_->window.nodes.size() >= 1);
        if (moving_window_->window.nodes.size() > 1)
            sim_band.nodes.insert(sim_band.nodes.end(), moving_window_->window.nodes.begin(),
                                  moving_window_->window.nodes.end());
        else
            sim_band.nodes.push_back(moving_window_->window.nodes.front());

        simulate(sim_band, distance_field, num_iterations_, collision_distance_, nominal_force_gain_,
                 internal_force_gain_, external_force_gain_, rotation_gain_, velocity_decay_, 1.0, alpha_decay_);

        // debug viz
        if (debug_viz_ && marker_pub_.getNumSubscribers() > 0)
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

            const double length = 0.1;
            const double radius = 0.02;

            for (const auto& node : sim_band.nodes)
            {
                const Eigen::Isometry3d pose =
                    Eigen::Translation3d(node.pose.translation().x(), node.pose.translation().y(), 0.0) *
                    Eigen::AngleAxisd(Eigen::Rotation2Dd(node.pose.rotation()).angle(), Eigen::Vector3d::UnitZ());

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

                {
                    const sim_band_planner::ControlPoint& cp = node.control_points[node.closest_point];
                    const Eigen::Vector2d cp_pose = node.pose.translation() + node.pose.linear() * cp.offset;
                    const Eigen::Vector2d obj_pose =
                        cp_pose + (cp.distance + robot_radius_) * cp.gradient.cast<double>();

                    geometry_msgs::Point start;
                    start.x = node.pose.translation().x();
                    start.y = node.pose.translation().y();

                    geometry_msgs::Point end;
                    end.x = obj_pose.x();
                    end.y = obj_pose.y();

                    visualization_msgs::Marker marker;
                    marker.ns = "arrow";
                    marker.id = static_cast<int>(ma.markers.size());
                    marker.type = visualization_msgs::Marker::ARROW;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.header.stamp = now;
                    marker.header.frame_id = "map";
                    marker.frame_locked = true;
                    marker.scale.x = 0.006;
                    marker.scale.y = 0.006;
                    marker.scale.z = 0.02;
                    marker.color.r = 1.0;
                    marker.color.a = 1.0;
                    marker.pose.orientation.w = 1;
                    marker.points.push_back(start);
                    marker.points.push_back(end);
                    ma.markers.push_back(marker);
                }
            }

            for (size_t c = 0; c < sim_band.nodes.front().control_points.size(); c++)
            {
                const ControlPoint& cp = sim_band.nodes.front().control_points[c];
                const Eigen::Vector2d position =
                    sim_band.nodes.front().pose.translation() + sim_band.nodes.front().pose.linear() * cp.offset;
                const Eigen::Isometry3d pose =
                    Eigen::Translation3d(position.x(), position.y(), 0.0) * Eigen::Quaterniond::Identity();

                const std_msgs::ColorRGBA color = distanceToColor(cp.distance);

                ma.markers.push_back(make_cylider(color, robot_radius_ * 2.0, 0.1, pose));

                {
                    const Eigen::Vector2d obj_pose =
                        position + (cp.distance + robot_radius_) * cp.gradient.cast<double>();

                    geometry_msgs::Point start;
                    start.x = position.x();
                    start.y = position.y();

                    geometry_msgs::Point end;
                    end.x = obj_pose.x();
                    end.y = obj_pose.y();

                    visualization_msgs::Marker marker;
                    marker.ns = "arrow";
                    marker.id = static_cast<int>(ma.markers.size());
                    marker.type = visualization_msgs::Marker::ARROW;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.header.stamp = now;
                    marker.header.frame_id = "map";
                    marker.frame_locked = true;
                    marker.scale.x = 0.006;
                    marker.scale.y = 0.006;
                    marker.scale.z = 0.02;
                    marker.color = color;
                    marker.pose.orientation.w = 1;
                    marker.points.push_back(start);
                    marker.points.push_back(end);
                    ma.markers.push_back(marker);
                }
            }

            marker_pub_.publish(ma);
        }

        // copy the nodes back to the moving window
        ROS_ASSERT(!sim_band.nodes.empty());
        moving_window_->window.nodes.clear();
        // don't copy the first node (its just the current robot pose)
        moving_window_->window.nodes.insert(moving_window_->window.nodes.end(), sim_band.nodes.begin() + 1,
                                            sim_band.nodes.end());

        // trim to valid
        result.outcome = navigation_interface::TrajectoryPlanner::Outcome::SUCCESSFUL;
        result.path_start_i = 0;
        result.path_end_i = moving_window_->end_i;
        std::size_t end_i;
        for (end_i = 0; end_i < sim_band.nodes.size(); ++end_i)
        {
            if (sim_band.nodes[end_i].control_points[sim_band.nodes[end_i].closest_point].distance < 0)
            {
                ROS_WARN_STREAM("Point: " << end_i << " of trajectory is in collision");
                //                end_i = end_i > 1 ? end_i - 1 : 0;
                result.outcome = navigation_interface::TrajectoryPlanner::Outcome::PARTIAL;
                result.path_end_i = moving_window_->end_i - (sim_band.nodes.size() - 1 - end_i);
                break;
            }
        }

        if (end_i == 0)
            return result;

        // TODO collision check interpolation

        // Transform to odom frame
        result.trajectory.header.frame_id = "odom";
        result.cost = 0;
        const Eigen::Isometry2d odom_to_map = map_to_odom.inverse();

        for (std::size_t i = 0; i < end_i; ++i)
        {
            result.trajectory.states.push_back(
                {odom_to_map * sim_band.nodes[i].pose,
                 {0, 0, 0},
                 sim_band.nodes[i].control_points[sim_band.nodes[i].closest_point].distance});
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Optimization failed: " << e.what());
        result.outcome = navigation_interface::TrajectoryPlanner::Outcome::FAILED;
        return result;
    }

    return result;
}

// cppcheck-suppress unusedFunction
bool SimBandPlanner::valid(const navigation_interface::Trajectory&) const
{
    return true;
}

double SimBandPlanner::cost(const navigation_interface::Trajectory&) const
{
    return 0.0;
}

// cppcheck-suppress unusedFunction
void SimBandPlanner::onInitialize(const XmlRpc::XmlRpcValue& parameters)
{
    debug_viz_ = navigation_interface::get_config_with_default_warn<bool>(parameters, "debug_viz", debug_viz_,
                                                                          XmlRpc::XmlRpcValue::TypeBoolean);
    num_iterations_ = navigation_interface::get_config_with_default_warn<int>(
        parameters, "num_iterations", num_iterations_, XmlRpc::XmlRpcValue::TypeInt);

    collision_distance_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "collision_distance", collision_distance_, XmlRpc::XmlRpcValue::TypeDouble);

    nominal_force_gain_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "nominal_force_gain", nominal_force_gain_, XmlRpc::XmlRpcValue::TypeDouble);
    internal_force_gain_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "internal_force_gain", internal_force_gain_, XmlRpc::XmlRpcValue::TypeDouble);
    external_force_gain_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "external_force_gain", external_force_gain_, XmlRpc::XmlRpcValue::TypeDouble);
    rotation_gain_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "rotation_gain", rotation_gain_, XmlRpc::XmlRpcValue::TypeDouble);

    max_window_length_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "max_window_length", max_window_length_, XmlRpc::XmlRpcValue::TypeDouble);

    velocity_decay_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "velocity_decay", velocity_decay_, XmlRpc::XmlRpcValue::TypeDouble);
    alpha_decay_ = navigation_interface::get_config_with_default_warn<double>(parameters, "alpha_decay", alpha_decay_,
                                                                              XmlRpc::XmlRpcValue::TypeDouble);

    desired_x_speed_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "desired_x_speed", desired_x_speed_, XmlRpc::XmlRpcValue::TypeDouble);
    desired_y_speed_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "desired_y_speed", desired_y_speed_, XmlRpc::XmlRpcValue::TypeDouble);
    desired_w_speed_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "desired_w_speed", desired_w_speed_, XmlRpc::XmlRpcValue::TypeDouble);

    max_acc_[0] = navigation_interface::get_config_with_default_warn<double>(parameters, "max_x_acc", max_acc_[0],
                                                                             XmlRpc::XmlRpcValue::TypeDouble);
    max_acc_[1] = navigation_interface::get_config_with_default_warn<double>(parameters, "max_y_acc", max_acc_[1],
                                                                             XmlRpc::XmlRpcValue::TypeDouble);
    max_acc_[2] = navigation_interface::get_config_with_default_warn<double>(parameters, "max_w_acc", max_acc_[2],
                                                                             XmlRpc::XmlRpcValue::TypeDouble);

    robot_radius_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "robot_radius", robot_radius_, XmlRpc::XmlRpcValue::TypeDouble);
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
        marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("sim_band", 100);
    }
}

// cppcheck-suppress unusedFunction
void SimBandPlanner::onMapDataChanged()
{
}
}  // namespace sim_band_planner
