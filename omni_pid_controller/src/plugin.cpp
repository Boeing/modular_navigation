#include <omni_pid_controller/plugin.h>
#include <navigation_interface/params.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(omni_pid_controller, OmniPIDController, omni_pid_controller::OmniPIDController,
                        navigation_interface::Controller)


namespace omni_pid_controller
{

OmniPIDController::OmniPIDController() : costmap_(nullptr)
{
}

OmniPIDController::~OmniPIDController()
{
}

bool OmniPIDController::setTrajectory(const navigation_interface::Trajectory& trajectory)
{
    if (trajectory.states.empty())
        return false;

    trajectory_.reset(new navigation_interface::Trajectory(trajectory));
    target_i_ = 0;
    control_error_ = Eigen::Vector3d::Zero();
    velocity_integral_ = Eigen::Vector3d::Zero();

    return true;
}

void OmniPIDController::clearTrajectory()
{
    trajectory_.reset();
}

boost::optional<std::string> OmniPIDController::trajectoryId() const
{
    if (trajectory_)
        return boost::optional<std::string>(trajectory_->id);
    else
        return {};
}

boost::optional<navigation_interface::Trajectory> OmniPIDController::trajectory() const
{
    if (trajectory_)
        return boost::optional<navigation_interface::Trajectory>(*trajectory_);
    else
        return {};
}

navigation_interface::Controller::Result OmniPIDController::control(const ros::SteadyTime& time, const navigation_interface::KinodynamicState& robot_state, const Eigen::Isometry2d& map_to_odom)
{
    navigation_interface::Controller::Result result;
    result.command = Eigen::Vector3d::Zero();

    if (!trajectory_)
    {
        result.outcome = navigation_interface::Controller::Outcome::FAILED;
        return result;
    }

    double dt = time.toSec() - last_update_.toSec();
    if (last_update_.toSec() < 0.001)
        dt = 0.1;
    last_update_ = time;

//    ROS_INFO_STREAM("robot_state: " << robot_state.pose.translation().transpose());
//    ROS_INFO_STREAM("robot_state: " << Eigen::Rotation2Dd(robot_state.pose.linear()).angle());

    //
    // Calculate the control error
    //
    Eigen::Vector3d control_error;

    Eigen::Vector3d target_velocity = Eigen::Vector3d::Zero();
    if (trajectory_->states.size() == 1)
    {
        control_error.topRows(2) = robot_state.pose.linear() * (trajectory_->states[0].pose.translation() - robot_state.pose.translation());
        const auto diff = robot_state.pose.linear().inverse() * trajectory_->states[0].pose.linear();
        control_error[2] = Eigen::Rotation2Dd(diff).angle();
    }
    else
    {
        const auto diff = robot_state.pose.linear().inverse() * trajectory_->states[1].pose.linear();
        control_error[2] = Eigen::Rotation2Dd(diff).angle();

        const Eigen::ParametrizedLine<double, 2> line = Eigen::ParametrizedLine<double, 2>::Through(
                    trajectory_->states[0].pose.translation(),
                    trajectory_->states[1].pose.translation()
                );

        const Eigen::Vector2d point_along_segment = line.projection(robot_state.pose.translation());
        control_error.topRows(2) = robot_state.pose.linear() * (point_along_segment - robot_state.pose.translation());

        const Eigen::Vector2d goal_direction = (trajectory_->states[1].pose.translation() - robot_state.pose.translation()).normalized();
        const Eigen::Vector2d goal_direction_wrt_robot = robot_state.pose.linear().inverse() * goal_direction;

        target_velocity.topRows(2) = (trajectory_->states[1].velocity.topRows(2).norm() * goal_direction_wrt_robot);

//        ROS_INFO_STREAM("goal_direction: " << goal_direction.transpose());
//        ROS_INFO_STREAM("goal_direction_wrt_robot: " << goal_direction_wrt_robot.transpose());
//        ROS_INFO_STREAM("target_velocity: " << target_velocity.transpose());

        Eigen::Hyperplane<double, 2> plane(line.direction(), trajectory_->states[1].pose.translation());
        const double dist_to_finish = -plane.signedDistance(robot_state.pose.translation());
//        ROS_INFO_STREAM("dist_to_finish: " << dist_to_finish);
    }

    //
    // Calculate velocity error
    //
    const Eigen::Vector3d velocity_error = target_velocity - robot_state.velocity;
    velocity_integral_ += velocity_error;

    //
    // Calculate error rate of change
    //
    const Eigen::Vector3d error_dot = (control_error_ - control_error) / dt;
    control_error_ = control_error;

    //
    // Goal condition
    //
    if (trajectory_->states.size() == 1)
    {
        // TODO also check velcity has gone to zero

        const double dist_to_goal = control_error.topRows(2).norm();
        const double angle_to_goal = std::abs(control_error[2]);
        if (angle_to_goal < yaw_goal_tolerance_ && dist_to_goal < xy_goal_tolerance_)
        {
            ROS_INFO_STREAM("Goal complete: delta: " << control_error.transpose());
            result.outcome = navigation_interface::Controller::Outcome::COMPLETE;
            last_update_ = ros::SteadyTime(0);
            return result;
        }
    }

    //
    // Control Law
    //
    double ddx = 0;
    double ddy = 0;
    double ddw = 0;
    if (trajectory_->states.size() == 1)
    {
        const Eigen::Vector3d desired_x_dot = control_gains_.goal_p_gain.cwiseProduct(control_error) + control_gains_.goal_d_gain.cwiseProduct(error_dot);

        ddx = (desired_x_dot[0] - robot_state.velocity[0]) / dt;
        ddy = (desired_x_dot[1] - robot_state.velocity[1]) / dt;
        ddw = (desired_x_dot[2] - robot_state.velocity[2]) / dt;
    }
    else
    {
        const Eigen::Vector3d desired_x_dot = control_gains_.control_p_gain.cwiseProduct(control_error) + control_gains_.control_d_gain.cwiseProduct(error_dot);
        const Eigen::Vector3d desired_acc = control_gains_.control_vp_gain.cwiseProduct(velocity_error) + control_gains_.control_vi_gain.cwiseProduct(velocity_integral_);

//        ROS_INFO_STREAM("velocity_error: " << velocity_error.transpose());
//        ROS_INFO_STREAM("desired_x_dot: " << desired_x_dot.transpose());
//        ROS_INFO_STREAM("desired_acc: " << desired_acc.transpose());

//        ROS_INFO_STREAM("e0: " << (desired_x_dot[0] - robot_state.velocity[0]) / dt);
//        ROS_INFO_STREAM("e1: " << (desired_x_dot[1] - robot_state.velocity[1]) / dt);
//        ROS_INFO_STREAM("e2: " << (desired_x_dot[2] - robot_state.velocity[2]) / dt);

        ddx = (desired_x_dot[0] - robot_state.velocity[0]) / dt + desired_acc[0];
        ddy = (desired_x_dot[1] - robot_state.velocity[1]) / dt + desired_acc[1];
        ddw = (desired_x_dot[2] - robot_state.velocity[2]) / dt + desired_acc[2];

        ddx = (desired_acc[0] - robot_state.velocity[0]) / dt;
        ddy = (desired_acc[1] - robot_state.velocity[1]) / dt;
        ddw = (desired_x_dot[2] - robot_state.velocity[2]) / dt + desired_acc[2];

//        ROS_INFO_STREAM("ddx: " << ddx);
//        ROS_INFO_STREAM("ddy: " << ddy);
//        ROS_INFO_STREAM("ddw: " << ddw);
    }

    //
    // Max acceleration check
    //
    {
        double acc_factor_x = 1.0;
        if (std::abs(ddx) > max_acceleration_x_ && std::signbit(robot_state.velocity.x()) == std::signbit(ddx))
        {
//            ROS_WARN_STREAM("Limiting maximum acceleration in X: " << ddx << " > " << max_acceleration_x_);
            acc_factor_x = std::abs(ddx / max_acceleration_x_);
        }

        double acc_factor_y = 1.0;
        if (std::abs(ddy) > max_acceleration_y_ && std::signbit(robot_state.velocity.y()) == std::signbit(ddy))
        {
//            ROS_WARN_STREAM("Limiting maximum acceleration in Y: " << ddy << " > " << max_acceleration_y_);
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
    x_dot_command[0] = target_velocity[0]; // robot_state.velocity[0] + ddx * dt;
    x_dot_command[1] = target_velocity[1]; // robot_state.velocity[1] + ddy * dt;
    x_dot_command[2] = robot_state.velocity[2] + ddw * dt;

    //
    // Max velocity check
    //
    {
        double velocity_factor_x = 1.0;
        if (std::abs(x_dot_command.x()) > max_velocity_x_)
        {
//            ROS_WARN_STREAM("Velocity in X exceeds limit: " << x_dot_command.x() << " > " << max_velocity_x_);
            velocity_factor_x = std::abs(x_dot_command.x() / max_velocity_x_);
        }

        double velocity_factor_y = 1.0;
        if (std::abs(x_dot_command.y()) > max_velocity_y_)
        {
//            ROS_WARN_STREAM("Velocity in Y exceeds limit: " << x_dot_command.y() << " > " << max_velocity_y_);
            velocity_factor_y = std::abs(x_dot_command.y() / max_velocity_y_);
        }

        const double velocity_factor = std::max(velocity_factor_x, velocity_factor_y);
        x_dot_command.x() /= velocity_factor;
        x_dot_command.y() /= velocity_factor;

        if (std::abs(x_dot_command.z()) > max_velocity_w_)
        {
//            ROS_WARN_STREAM("Velocity in W exceeds limit: " << x_dot_command.z() << " > " << max_velocity_w_);
            const double scale = max_velocity_w_ / std::abs(x_dot_command.z());
            x_dot_command.z() *= scale;
        }
    }

//    ROS_INFO_STREAM("ddx: " << ddx);
//    ROS_INFO_STREAM("ddy: " << ddy);
//    ROS_INFO_STREAM("ddw: " << ddw);
//    ROS_INFO_STREAM("target_velocity: " << target_velocity.transpose());
//    ROS_INFO_STREAM("velocity: " << robot_state.velocity.transpose());
//    ROS_INFO_STREAM("x_dot_command: " << x_dot_command.transpose());

    result.command = x_dot_command;
    result.outcome = navigation_interface::Controller::Outcome::SUCCESSFUL;

    return result;
}

void OmniPIDController::initialize(const XmlRpc::XmlRpcValue& parameters,
                        const std::shared_ptr<const costmap_2d::Costmap2D>& costmap)
{
    costmap_ = costmap;

    max_velocity_x_ = navigation_interface::get_config_with_default_warn<double>(parameters, "max_velocity_x", max_velocity_x_, XmlRpc::XmlRpcValue::TypeDouble);
    max_velocity_y_ = navigation_interface::get_config_with_default_warn<double>(parameters, "max_velocity_y", max_velocity_y_, XmlRpc::XmlRpcValue::TypeDouble);
    max_velocity_w_ = navigation_interface::get_config_with_default_warn<double>(parameters, "max_velocity_w", max_velocity_w_, XmlRpc::XmlRpcValue::TypeDouble);
    max_acceleration_x_ = navigation_interface::get_config_with_default_warn<double>(parameters, "max_acceleration_x", max_acceleration_x_, XmlRpc::XmlRpcValue::TypeDouble);
    max_acceleration_y_ = navigation_interface::get_config_with_default_warn<double>(parameters, "max_acceleration_y", max_acceleration_y_, XmlRpc::XmlRpcValue::TypeDouble);
    max_acceleration_w_ = navigation_interface::get_config_with_default_warn<double>(parameters, "max_acceleration_w", max_acceleration_w_, XmlRpc::XmlRpcValue::TypeDouble);
    goal_radius_ = navigation_interface::get_config_with_default_warn<double>(parameters, "goal_radius", goal_radius_, XmlRpc::XmlRpcValue::TypeDouble);
    xy_goal_tolerance_ = navigation_interface::get_config_with_default_warn<double>(parameters, "xy_goal_tolerance", xy_goal_tolerance_, XmlRpc::XmlRpcValue::TypeDouble);
    yaw_goal_tolerance_ = navigation_interface::get_config_with_default_warn<double>(parameters, "yaw_goal_tolerance", yaw_goal_tolerance_, XmlRpc::XmlRpcValue::TypeDouble);
}

}
