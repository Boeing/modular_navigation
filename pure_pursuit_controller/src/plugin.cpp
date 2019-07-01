#include <navigation_interface/params.h>
#include <pure_pursuit_controller/plugin.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(pure_pursuit_controller::PurePursuitController, navigation_interface::Controller)

namespace pure_pursuit_controller
{

namespace
{

bool robotInCollision(const gridmap::MapData& map_data, const int map_x, const int map_y, const double robot_radius)
{
    const int robot_radius_cells = static_cast<int>(robot_radius / map_data.grid.dimensions().resolution());
    const int r2 = robot_radius_cells * robot_radius_cells;
    for (int j = -robot_radius_cells; j <= robot_radius_cells; ++j)
    {
        for (int i = -robot_radius_cells; i <= robot_radius_cells; ++i)
        {
            if (j * j + i * i < r2)
            {
                if (map_data.grid.occupied({map_x + i, map_y + j}))
                {
                    return true;
                }
            }
        }
    }
    return false;
}

std::pair<std::size_t, double> targetState(const Eigen::Isometry2d& pose,
                                           const navigation_interface::Trajectory& trajectory, const double lookahead)
{
    std::vector<double> distances;
    std::transform(trajectory.states.begin(), trajectory.states.end(), std::back_inserter(distances),
                   [&pose](const navigation_interface::KinodynamicState& state) {
                       return (state.pose.translation() - pose.translation()).norm();
                   });
    auto it = std::min_element(distances.begin(), distances.end());
    long dist = std::distance(distances.begin(), it);
    {
        while (*it < lookahead && dist < static_cast<int>(trajectory.states.size()) - 1)
        {
            it++;
            dist = std::distance(distances.begin(), it);
        }

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
}

PurePursuitController::PurePursuitController()
{
}

PurePursuitController::~PurePursuitController()
{
}

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

void PurePursuitController::clearTrajectory()
{
    trajectory_.reset();
}

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
    // Find target i
    //
    auto closest_i = targetState(robot_state.pose, *trajectory_, look_ahead_);
    std::size_t target_i = closest_i.first;

    const double dist_to_goal = (trajectory_->states.back().pose.translation() - robot_state.pose.translation()).norm();
    const double angle_to_goal =
        std::abs(Eigen::Rotation2Dd(robot_state.pose.linear().inverse() * trajectory_->states.back().pose.linear())
                     .smallestAngle());

    if (dist_to_goal < goal_radius_)
    {
        target_i = trajectory_->states.size() - 1;
    }

    ROS_ASSERT(robot_state.pose.linear().allFinite());
    ROS_ASSERT(robot_state.pose.translation().allFinite());

    const Eigen::Vector2d goal_vec = trajectory_->states[target_i].pose.translation() - robot_state.pose.translation();
    const Eigen::Vector2d goal_vec_wrt_robot = robot_state.pose.linear().inverse() * goal_vec;

    const auto rot_diff = robot_state.pose.linear().inverse() * trajectory_->states[target_i].pose.linear();

    Eigen::Vector3d control_error;
    control_error.topRows(2) = goal_vec_wrt_robot;
    control_error[2] = Eigen::Rotation2Dd(rot_diff).smallestAngle();

    ROS_ASSERT(control_error.allFinite());

    const Eigen::Vector2d goal_direction = goal_vec.normalized();
    const Eigen::Vector2d goal_direction_wrt_robot = robot_state.pose.linear().inverse() * goal_direction;

    Eigen::Vector3d target_velocity = Eigen::Vector3d::Zero();
    target_velocity.topRows(2) = (trajectory_->states[target_i].velocity.topRows(2).norm() * goal_direction_wrt_robot);
    const double finish_time = goal_vec.norm() / target_velocity.topRows(2).norm();
    target_velocity[2] = control_error[2] / finish_time;

    const Eigen::Vector3d control_dot_ = (control_error - control_error_) / dt;
    control_error_ = control_error;

    ROS_ASSERT(control_error_.allFinite());

    //
    // Check immediate collisions
    //
    {
        auto lock = map_data_->grid.getLock();

        const Eigen::Isometry2d map_robot_pose = map_to_odom * robot_state.pose;
        const Eigen::Isometry2d map_goal_pose = map_to_odom * trajectory_->states[target_i].pose;

        const Eigen::Array2i robot_map_index = map_data_->grid.dimensions().getCellIndex(map_robot_pose.translation());
        const Eigen::Array2i goal_map_index = map_data_->grid.dimensions().getCellIndex(map_goal_pose.translation());

        if (robotInCollision(*map_data_, robot_map_index.x(), robot_map_index.y(), robot_radius_))
        {
            ROS_WARN_STREAM("Current robot pose is in collision!");
            result.outcome = navigation_interface::Controller::Outcome::FAILED;
            last_update_ = ros::SteadyTime(0);
            return result;
        }
        if (robotInCollision(*map_data_, goal_map_index.x(), goal_map_index.y(), robot_radius_))
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

        const double acc_factor_xy = std::max(acc_factor_x, acc_factor_y);

        ddx /= acc_factor_xy;
        ddy /= acc_factor_xy;
        ddw /= acc_factor_w;
    }

    //
    // Integrate new velocity command
    //
    Eigen::Vector3d x_dot_command;
    x_dot_command[0] = robot_state.velocity[0] + ddx * dt;
    x_dot_command[1] = robot_state.velocity[1] + ddy * dt;
    x_dot_command[2] = robot_state.velocity[2] + ddw * dt;

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

        const double velocity_factor_xy = std::max(velocity_factor_x, velocity_factor_y);
        x_dot_command.x() /= velocity_factor_xy;
        x_dot_command.y() /= velocity_factor_xy;
        x_dot_command.z() /= velocity_factor_w;
    }

    result.command = x_dot_command;
    result.outcome = navigation_interface::Controller::Outcome::SUCCESSFUL;

    return result;
}

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
}

void PurePursuitController::onMapDataChanged()
{
}
}
