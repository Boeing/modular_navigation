#include <navigation_interface/params.h>
#include <pure_pursuit_controller/plugin.h>

#include <gridmap/operations/rasterize.h>

#include <pluginlib/class_list_macros.h>

#include <visualization_msgs/MarkerArray.h>

#define EPS 0.001

PLUGINLIB_EXPORT_CLASS(pure_pursuit_controller::PurePursuitController, navigation_interface::Controller)

namespace pure_pursuit_controller
{

namespace
{

std::pair<bool, visualization_msgs::Marker> robotInCollision(const gridmap::MapData& map_data,
                                                             const Eigen::Isometry2d& robot_pose,
                                                             const std::vector<Eigen::Vector2d>& footprint,
                                                             const float alpha)
{
    int min_x = std::numeric_limits<int>::max();
    int max_x = 0;

    int min_y = std::numeric_limits<int>::max();
    int max_y = 0;

    std::vector<Eigen::Array2i> map_footprint;
    for (const Eigen::Vector2d& p : footprint)
    {
        const Eigen::Vector2d world_point = robot_pose.translation() + robot_pose.rotation() * p;
        const auto map_point = map_data.grid.dimensions().getCellIndex(world_point);
        map_footprint.push_back(map_point);
        min_x = std::min(map_point.x(), min_x);
        max_x = std::max(map_point.x(), max_x);
        min_y = std::min(map_point.y(), min_y);
        max_y = std::max(map_point.y(), max_y);
    }

    const auto connected_poly = gridmap::connectPolygon(map_footprint);

    visualization_msgs::Marker marker;
    marker.ns = "points";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker.frame_locked = true;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.00;
    marker.color.a = 1.f;
    marker.pose.orientation.w = 1.0;

    bool in_collision = false;
    auto append_raster = [&map_data, &marker, &in_collision, alpha](const int x, const int y) {
        const Eigen::Array2i p{x, y};

        const Eigen::Vector2d w = map_data.grid.dimensions().getCellCenter(p);
        geometry_msgs::Point mp;
        mp.x = w.x();
        mp.y = w.y();
        mp.z = 0.0;
        marker.points.push_back(mp);
        std_msgs::ColorRGBA c;
        c.a = alpha;
        if (map_data.grid.dimensions().contains(p) && map_data.grid.occupied(p))
        {
            in_collision = true;
            c.r = 1.0;
        }
        else
        {
            c.g = 1.0;
        }
        marker.colors.push_back(c);
    };

    gridmap::rasterPolygonFill(append_raster, connected_poly, min_x, max_x, min_y, max_y);

    return std::make_pair(in_collision, marker);
}

visualization_msgs::Marker buildMarker(const navigation_interface::KinodynamicState& robot_state,
                                       const navigation_interface::KinodynamicState& target_state)
{
    visualization_msgs::Marker marker;
    marker.ns = "target";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "odom";
    marker.frame_locked = true;
    marker.scale.x = 0.02;
    marker.scale.y = 0.04;
    marker.scale.z = 0.04;
    marker.color.a = 1.f;
    marker.pose.orientation.w = 1.0;

    {
        geometry_msgs::Point mp;
        mp.x = robot_state.pose.translation().x();
        mp.y = robot_state.pose.translation().y();
        mp.z = 0.0;
        marker.points.push_back(mp);
    }

    {
        geometry_msgs::Point mp;
        mp.x = target_state.pose.translation().x();
        mp.y = target_state.pose.translation().y();
        mp.z = 0.0;
        marker.points.push_back(mp);
    }

    return marker;
}

std::pair<std::size_t, double> targetState(const Eigen::Isometry2d& pose,
                                           const navigation_interface::Trajectory& trajectory)
{
    std::vector<double> distances;
    std::transform(trajectory.states.begin(), trajectory.states.end(), std::back_inserter(distances),
                   [&pose](const navigation_interface::KinodynamicState& state) {
                       return (state.pose.translation() - pose.translation()).norm();
                   });
    auto it = std::min_element(distances.begin(), distances.end());
    long dist = std::distance(distances.begin(), it);
    {
        if (dist < static_cast<int>(trajectory.states.size()) - 1)
        {
            const double fut_seg =
                (trajectory.states[dist + 1].pose.translation() - trajectory.states[dist].pose.translation()).norm();
            if (distances[dist + 1] < fut_seg)
            {
                it++;
                dist = std::distance(distances.begin(), it);
            }
        }

        return {dist, *it};
    }
}

navigation_interface::KinodynamicState targetState(const Eigen::Isometry2d& pose,
                                                   const navigation_interface::Trajectory& trajectory,
                                                   const double linear_lookahead, const double angular_lookahead)
{
    const auto target = targetState(pose, trajectory);

    auto it = trajectory.states.begin() + static_cast<int>(target.first);
    while (
        it != trajectory.states.end() &&
        ((it->pose.translation() - pose.translation()).norm() < linear_lookahead &&
         std::abs(Eigen::Rotation2Dd(it->pose.linear().inverse() * pose.linear()).smallestAngle()) < angular_lookahead))
    {
        ++it;
    }

    if (it == trajectory.states.end())
        it--;

    const Eigen::Vector2d dir = it->pose.translation() - pose.translation();
    const double dist = dir.norm();

    const Eigen::Rotation2Dd rot(pose.linear());
    const Eigen::Rotation2Dd future_rot(it->pose.linear());
    const Eigen::Rotation2Dd rotation(future_rot.inverse() * rot);
    const double angle = std::abs(rotation.smallestAngle());

    const double linear_fraction = dist > 0.0 ? (linear_lookahead / dist) : 0.0;
    const double angular_fraction = angle > 0.0 ? (angular_lookahead / angle) : 0.0;
    const double fraction = std::min(linear_fraction, angular_fraction);

    if (fraction >= 1)
    {
        return *it;
    }
    else
    {
        return {Eigen::Translation2d(pose.translation() + fraction * dir) * rot.slerp(fraction, future_rot),
                it->velocity};
    }
}
}  // namespace

PurePursuitController::PurePursuitController()
{
}

PurePursuitController::~PurePursuitController()
{
}

// cppcheck-suppress unusedFunction
bool PurePursuitController::setTrajectory(const navigation_interface::Trajectory& trajectory)
{
    if (trajectory.states.empty())
        return false;

    trajectory_.reset(new navigation_interface::Trajectory(trajectory));
    control_integral_ = Eigen::Vector3d::Zero();
    control_error_ = Eigen::Vector3d::Zero();
    last_update_ = ros::SteadyTime(0);

    return true;
}

// cppcheck-suppress unusedFunction
void PurePursuitController::clearTrajectory()
{
    trajectory_.reset();
}

// cppcheck-suppress unusedFunction
boost::optional<std::string> PurePursuitController::trajectoryId() const
{
    if (trajectory_)
        return boost::optional<std::string>(trajectory_->id);
    else
        return {};
}

boost::optional<navigation_interface::Trajectory> PurePursuitController::trajectory() const
{
    if (trajectory_)
        return boost::optional<navigation_interface::Trajectory>(*trajectory_);
    else
        return {};
}

navigation_interface::Controller::Result
    // cppcheck-suppress unusedFunction
    PurePursuitController::control(const ros::SteadyTime& time,
                                   const navigation_interface::KinodynamicState& robot_state,
                                   const Eigen::Isometry2d& map_to_odom)
{
    navigation_interface::Controller::Result result;
    result.command = Eigen::Vector3d::Zero();

    if (!trajectory_)
    {
        result.outcome = navigation_interface::Controller::Outcome::FAILED;
        return result;
    }

    double dt = std::max(0.01, time.toSec() - last_update_.toSec());
    if (last_update_ == ros::SteadyTime(0))
        dt = 0.05;
    last_update_ = time;

    //
    // Find target i (look 0.5s into the future)
    //
    const double linear_lookahead = std::max(0.04, robot_state.velocity.x() * 0.5);
    const double angular_lookahead = std::max(0.04, robot_state.velocity.z() * 0.5);
    const auto target_state = targetState(robot_state.pose, *trajectory_, linear_lookahead, angular_lookahead);

    if (debug_viz_)
        target_state_pub_.publish(buildMarker(robot_state, target_state));

    const double dist_to_goal = (trajectory_->states.back().pose.translation() - robot_state.pose.translation()).norm();
    const double angle_to_goal =
        std::abs(Eigen::Rotation2Dd(robot_state.pose.linear().inverse() * trajectory_->states.back().pose.linear())
                     .smallestAngle());

    ROS_ASSERT(robot_state.pose.linear().allFinite());
    ROS_ASSERT(robot_state.pose.translation().allFinite());

    const Eigen::Vector2d goal_vec = target_state.pose.translation() - robot_state.pose.translation();
    const Eigen::Vector2d goal_vec_wrt_robot = robot_state.pose.linear().inverse() * goal_vec;

    const auto rot_diff = robot_state.pose.linear().inverse() * target_state.pose.linear();

    Eigen::Vector3d control_error;
    control_error.topRows(2) = goal_vec_wrt_robot;
    control_error[2] = Eigen::Rotation2Dd(rot_diff).smallestAngle();

    ROS_ASSERT(control_error.allFinite());

    const Eigen::Vector2d goal_direction = goal_vec.normalized();
    const Eigen::Vector2d goal_direction_wrt_robot = robot_state.pose.linear().inverse() * goal_direction;

    Eigen::Vector3d target_velocity = Eigen::Vector3d::Zero();
    target_velocity.topRows(2) = (target_state.velocity.topRows(2).norm() * goal_direction_wrt_robot);
    const double finish_time = goal_vec.norm() / target_velocity.topRows(2).norm();
    target_velocity[2] = control_error[2] / finish_time;

    const Eigen::Vector3d control_dot_ = (control_error - control_error_) / dt;
    control_error_ = control_error;

    ROS_ASSERT(control_error_.allFinite());

    //
    // Check immediate collisions
    //
    {
        // TODO need a more intelligent way to lock read access without waiting for path planning
        // auto lock = map_data_->grid.getLock();

        const Eigen::Isometry2d map_robot_pose = map_to_odom * robot_state.pose;
        const Eigen::Isometry2d map_goal_pose = map_to_odom * target_state.pose;

        bool in_collision;
        visualization_msgs::Marker marker;

        std::tie(in_collision, marker) = robotInCollision(*map_data_, map_robot_pose, robot_footprint_, 1.f);
        if (debug_viz_)
            footprint_pub_.publish(marker);

        bool future_in_collision;
        visualization_msgs::Marker future_marker;

        std::tie(future_in_collision, future_marker) =
            robotInCollision(*map_data_, map_goal_pose, robot_footprint_, 0.2f);
        if (debug_viz_)
            future_footprint_pub_.publish(future_marker);

        if (in_collision)
        {
            ROS_WARN_STREAM("Current robot pose is in collision!");
            result.outcome = navigation_interface::Controller::Outcome::FAILED;
            last_update_ = ros::SteadyTime(0);
            return result;
        }

        if (future_in_collision)
        {
            ROS_WARN_STREAM("Target robot pose is in collision!");
            result.outcome = navigation_interface::Controller::Outcome::FAILED;
            last_update_ = ros::SteadyTime(0);
            return result;
        }
    }

    //
    // Goal condition
    //
    if (angle_to_goal < yaw_goal_tolerance_ && dist_to_goal < xy_goal_tolerance_)
    {
        result.outcome = navigation_interface::Controller::Outcome::COMPLETE;
        last_update_ = ros::SteadyTime(0);
        return result;
    }

    //
    // PID
    //
    if (dist_to_goal < goal_radius_)
    {
        control_integral_ += control_error;
        target_velocity = goal_p_gain_.cwiseProduct(control_error) + goal_i_gain_.cwiseProduct(control_integral_) +
                          goal_d_gain_.cwiseProduct(control_dot_);
    }
    else
    {
        control_integral_ = Eigen::Vector3d::Zero();
        control_error_ = Eigen::Vector3d::Zero();
    }
    ROS_ASSERT(target_velocity.allFinite());

    double ddx = (target_velocity[0] - robot_state.velocity[0]) / dt;
    double ddy = (target_velocity[1] - robot_state.velocity[1]) / dt;
    double ddw = (target_velocity[2] - robot_state.velocity[2]) / dt;

    //
    // Max acceleration check
    //
    {
        double acc_factor_x = 1.0;
        if (std::abs(ddx) > max_acceleration_x_ && std::signbit(robot_state.velocity.x()) == std::signbit(ddx))
        {
            acc_factor_x = std::abs(ddx / max_acceleration_x_);
        }

        double acc_factor_y = 1.0;
        if (std::abs(ddy) > max_acceleration_y_ && std::signbit(robot_state.velocity.y()) == std::signbit(ddy))
        {
            acc_factor_y = std::abs(ddy / max_acceleration_y_);
        }

        double acc_factor_w = 1.0;
        if (std::abs(ddw) > max_acceleration_w_ && std::signbit(robot_state.velocity.z()) == std::signbit(ddw))
        {
            acc_factor_w = std::abs(ddw / max_acceleration_w_);
        }

        const double acc_factor = std::max(acc_factor_w, std::max(acc_factor_x, acc_factor_y));
        if (acc_factor > 0)
        {
            ddx /= acc_factor;
            ddy /= acc_factor;
            ddw /= acc_factor;
        }
        else
        {
            ddx = 0;
            ddy = 0;
            ddw = 0;
        }

        ROS_ASSERT(std::isfinite(ddx));
        ROS_ASSERT(std::isfinite(ddy));
        ROS_ASSERT(std::isfinite(ddw));
    }

    //
    // Integrate new velocity command
    //
    Eigen::Vector3d x_dot_command;
    x_dot_command[0] = robot_state.velocity[0] + ddx * dt;
    x_dot_command[1] = robot_state.velocity[1] + ddy * dt;
    x_dot_command[2] = robot_state.velocity[2] + ddw * dt;
    ROS_ASSERT(x_dot_command.allFinite());

    //
    // Max velocity check
    //
    {
        double velocity_factor_x = 1.0;
        if (std::abs(x_dot_command.x()) > max_velocity_x_)
        {
            ROS_DEBUG_STREAM("Velocity in X exceeds limit: " << x_dot_command.x() << " > " << max_velocity_x_);
            velocity_factor_x = std::abs(x_dot_command.x() / max_velocity_x_);
        }

        double velocity_factor_y = 1.0;
        if (std::abs(x_dot_command.y()) > max_velocity_y_)
        {
            ROS_DEBUG_STREAM("Velocity in Y exceeds limit: " << x_dot_command.y() << " > " << max_velocity_y_);
            velocity_factor_y = std::abs(x_dot_command.y() / max_velocity_y_);
        }

        double velocity_factor_w = 1.0;
        if (std::abs(x_dot_command.z()) > max_velocity_w_)
        {
            ROS_DEBUG_STREAM("Velocity in W exceeds limit: " << x_dot_command.z() << " > " << max_velocity_w_);
            velocity_factor_w = std::abs(x_dot_command.z() / max_velocity_w_);
        }

        const double velocity_factor = std::max(velocity_factor_w, std::max(velocity_factor_x, velocity_factor_y));
        if (velocity_factor > 0)
        {
            x_dot_command.x() /= velocity_factor;
            x_dot_command.y() /= velocity_factor;
            x_dot_command.z() /= velocity_factor;
        }

        ROS_ASSERT(x_dot_command.allFinite());
        ROS_ASSERT_MSG(std::abs(x_dot_command.x()) <= max_velocity_x_ + EPS, "dx: %f > %f", x_dot_command.x(),
                       max_velocity_x_);
        ROS_ASSERT_MSG(std::abs(x_dot_command.y()) <= max_velocity_y_ + EPS, "dy: %f > %f", x_dot_command.y(),
                       max_velocity_y_);
        ROS_ASSERT_MSG(std::abs(x_dot_command.z()) <= max_velocity_w_ + EPS, "dw: %f > %f", x_dot_command.z(),
                       max_velocity_w_);
    }

    result.command = x_dot_command;
    result.outcome = navigation_interface::Controller::Outcome::SUCCESSFUL;

    return result;
}

// cppcheck-suppress unusedFunction
void PurePursuitController::onInitialize(const XmlRpc::XmlRpcValue& parameters)
{
    look_ahead_ = navigation_interface::get_config_with_default_warn<double>(parameters, "look_ahead", look_ahead_,
                                                                             XmlRpc::XmlRpcValue::TypeDouble);
    max_velocity_x_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "max_velocity_x", max_velocity_x_, XmlRpc::XmlRpcValue::TypeDouble);
    max_velocity_y_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "max_velocity_y", max_velocity_y_, XmlRpc::XmlRpcValue::TypeDouble);
    max_velocity_w_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "max_velocity_w", max_velocity_w_, XmlRpc::XmlRpcValue::TypeDouble);
    max_acceleration_x_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "max_acceleration_x", max_acceleration_x_, XmlRpc::XmlRpcValue::TypeDouble);
    max_acceleration_y_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "max_acceleration_y", max_acceleration_y_, XmlRpc::XmlRpcValue::TypeDouble);
    max_acceleration_w_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "max_acceleration_w", max_acceleration_w_, XmlRpc::XmlRpcValue::TypeDouble);
    goal_radius_ = navigation_interface::get_config_with_default_warn<double>(parameters, "goal_radius", goal_radius_,
                                                                              XmlRpc::XmlRpcValue::TypeDouble);
    xy_goal_tolerance_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "xy_goal_tolerance", xy_goal_tolerance_, XmlRpc::XmlRpcValue::TypeDouble);
    yaw_goal_tolerance_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "yaw_goal_tolerance", yaw_goal_tolerance_, XmlRpc::XmlRpcValue::TypeDouble);

    goal_p_gain_.x() = navigation_interface::get_config_with_default_warn<double>(
        parameters, "goal_p_gain_x", goal_p_gain_.x(), XmlRpc::XmlRpcValue::TypeDouble);
    goal_p_gain_.y() = navigation_interface::get_config_with_default_warn<double>(
        parameters, "goal_p_gain_y", goal_p_gain_.y(), XmlRpc::XmlRpcValue::TypeDouble);
    goal_p_gain_.z() = navigation_interface::get_config_with_default_warn<double>(
        parameters, "goal_p_gain_z", goal_p_gain_.z(), XmlRpc::XmlRpcValue::TypeDouble);

    goal_i_gain_.x() = navigation_interface::get_config_with_default_warn<double>(
        parameters, "goal_i_gain_x", goal_i_gain_.x(), XmlRpc::XmlRpcValue::TypeDouble);
    goal_i_gain_.y() = navigation_interface::get_config_with_default_warn<double>(
        parameters, "goal_i_gain_y", goal_i_gain_.y(), XmlRpc::XmlRpcValue::TypeDouble);
    goal_i_gain_.z() = navigation_interface::get_config_with_default_warn<double>(
        parameters, "goal_i_gain_z", goal_i_gain_.z(), XmlRpc::XmlRpcValue::TypeDouble);

    goal_d_gain_.x() = navigation_interface::get_config_with_default_warn<double>(
        parameters, "goal_d_gain_x", goal_d_gain_.x(), XmlRpc::XmlRpcValue::TypeDouble);
    goal_d_gain_.y() = navigation_interface::get_config_with_default_warn<double>(
        parameters, "goal_d_gain_y", goal_d_gain_.y(), XmlRpc::XmlRpcValue::TypeDouble);
    goal_d_gain_.z() = navigation_interface::get_config_with_default_warn<double>(
        parameters, "goal_d_gain_z", goal_d_gain_.z(), XmlRpc::XmlRpcValue::TypeDouble);

    if (parameters.hasMember("footprint"))
    {
        robot_footprint_ = navigation_interface::get_point_list(parameters, "footprint");
    }
    else
    {
        robot_footprint_ = {{0.480, 0.000},  {0.398, -0.395}, {-0.398, -0.395},
                            {-0.480, 0.000}, {-0.398, 0.395}, {0.398, 0.395}};
    }

    debug_viz_ = navigation_interface::get_config_with_default_warn<bool>(parameters, "debug_viz", debug_viz_,
                                                                          XmlRpc::XmlRpcValue::TypeBoolean);
    if (debug_viz_)
    {
        ros::NodeHandle nh("~");
        target_state_pub_ = nh.advertise<visualization_msgs::Marker>("target_state", 100);
        footprint_pub_ = nh.advertise<visualization_msgs::Marker>("footprint", 100);
        future_footprint_pub_ = nh.advertise<visualization_msgs::Marker>("future_footprint", 100);
    }
}

// cppcheck-suppress unusedFunction
void PurePursuitController::onMapDataChanged()
{
}
}  // namespace pure_pursuit_controller
