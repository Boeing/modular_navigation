#include <eband_local_planner/eband_trajectory_controller.h>

#include <tf2/utils.h>

#include <algorithm>
#include <string>
#include <vector>

namespace eband_local_planner
{

namespace
{

geometry_msgs::Twist subtract(const geometry_msgs::Pose& frame2, const geometry_msgs::Pose& frame1)
{
    const geometry_msgs::Pose2D frame1_ = convert(frame1);
    const geometry_msgs::Pose2D frame2_ = convert(frame2);

    geometry_msgs::Twist frame_diff;
    frame_diff.linear.x = frame2_.x - frame1_.x;
    frame_diff.linear.y = frame2_.y - frame1_.y;
    frame_diff.linear.z = 0.0;
    frame_diff.angular.x = 0.0;
    frame_diff.angular.y = 0.0;
    frame_diff.angular.z = normalize_angle(frame2_.theta - frame1_.theta);

    return frame_diff;
}

geometry_msgs::Twist transformTwist(const geometry_msgs::Twist& curr_twist, const geometry_msgs::Pose& frame)
{
    const geometry_msgs::Pose2D frame_ = convert(frame);

    geometry_msgs::Twist tmp_transformed = curr_twist;
    tmp_transformed.linear.x = curr_twist.linear.x * cos(frame_.theta) + curr_twist.linear.y * sin(frame_.theta);
    tmp_transformed.linear.y = -curr_twist.linear.x * sin(frame_.theta) + curr_twist.linear.y * cos(frame_.theta);
    return tmp_transformed;
}

double pseudoDistance(const geometry_msgs::Twist& twist, const double robot_radius)
{
    double ang_pseudo_dist = twist.angular.z * robot_radius;
    return std::sqrt((twist.linear.x * twist.linear.x) + (twist.linear.y * twist.linear.y) +
                     (ang_pseudo_dist * ang_pseudo_dist));
}

geometry_msgs::Twist controlError(const std::vector<Bubble>& band, const double robot_radius)
{
    if (band.empty() || band.size() == 1)
        return {};

    double distance_sum = 0;
    std::vector<std::pair<geometry_msgs::Twist, double>> controls;
    for (std::size_t i = 1; i < band.size(); ++i)
    {
        const geometry_msgs::Twist diff = subtract(band[i].center, band.front().center);
        double distance = pseudoDistance(diff, robot_radius);
        if (i == 1 || distance < band.front().expansion)
        {
            distance_sum += distance;
            controls.push_back({diff, distance});
        }
        else
        {
            break;
        }
    }

    geometry_msgs::Twist error;
    for (std::size_t i = 0; i < controls.size(); ++i)
    {
        error.linear.x += controls[i].first.linear.x * (controls[i].second / distance_sum);
        error.linear.y += controls[i].first.linear.y * (controls[i].second / distance_sum);
        error.angular.z += controls[i].first.angular.z * (controls[i].second / distance_sum);
    }

    return error;
}
}

EBandController::EBandController(const std::shared_ptr<costmap_2d::Costmap2DROS>& local_costmap,
                                 const double max_velocity_x, const double max_velocity_y, const double max_velocity_w,
                                 const double max_acceleration_x, const double max_acceleration_y,
                                 const double max_acceleration_w, const double goal_radius,
                                 const double xy_goal_tolerance, const double yaw_goal_tolerance, const double k_prop,
                                 const double k_damp)
    : local_costmap_(local_costmap), max_velocity_x_(max_velocity_x), max_velocity_y_(max_velocity_y),
      max_velocity_w_(max_velocity_w), max_acceleration_x_(max_acceleration_x), max_acceleration_y_(max_acceleration_y),
      max_acceleration_w_(max_acceleration_w), goal_radius_(goal_radius), xy_goal_tolerance_(xy_goal_tolerance),
      yaw_goal_tolerance_(yaw_goal_tolerance), k_prop_(k_prop), k_damp_(k_damp),
      robot_radius_(getCircumscribedRadius(*local_costmap_)), last_update_(ros::SteadyTime(0))
{
}

EBandController::~EBandController()
{
}

nav_core::Control EBandController::computeControl(const std::vector<Bubble>& elastic_band,
                                                  const ros::SteadyTime& steady_time, const ros::Time&,
                                                  const nav_msgs::Odometry& odom)
{
    ROS_ASSERT(odom.header.frame_id == local_costmap_->getGlobalFrameID());
    ROS_ASSERT(elastic_band.size() > 1);

    double time_step = steady_time.toSec() - last_update_.toSec();
    if (last_update_.toSec() < 0.001)
        time_step = std::numeric_limits<double>::min();
    last_update_ = steady_time;

    nav_core::Control control;

    control.cmd_vel.linear.x = 0.0;
    control.cmd_vel.linear.y = 0.0;
    control.cmd_vel.linear.z = 0.0;
    control.cmd_vel.angular.x = 0.0;
    control.cmd_vel.angular.y = 0.0;
    control.cmd_vel.angular.z = 0.0;

    const geometry_msgs::Twist goal_error = subtract(elastic_band.back().center, elastic_band.front().center);
    const geometry_msgs::Twist goal_error_wrt_robot = transformTwist(goal_error, elastic_band.front().center);

    const geometry_msgs::Twist control_error = controlError(elastic_band, robot_radius_);
    const geometry_msgs::Twist control_error_wrt_robot = transformTwist(control_error, elastic_band.front().center);

    //
    // Goal condition
    //
    const double dist_to_goal =
        std::sqrt(goal_error.linear.x * goal_error.linear.x + goal_error.linear.y * goal_error.linear.y);
    if (std::abs(goal_error_wrt_robot.angular.z) < yaw_goal_tolerance_ && dist_to_goal < xy_goal_tolerance_)
    {
        ROS_INFO_STREAM("Goal complete: dx: " << goal_error.linear.x << " dy: " << goal_error.linear.y
                                              << " dw: " << goal_error_wrt_robot.angular.z);
        control.state = nav_core::ControlState::COMPLETE;
        goal_x_pid_.reset();
        goal_y_pid_.reset();
        goal_w_pid_.reset();
        last_update_ = ros::SteadyTime(0);
        return control;
    }

    double ddx = 0;
    double ddy = 0;
    double ddw = 0;
    if (dist_to_goal < goal_radius_)
    {
        //
        // PID goal control
        //
        const double dx = goal_x_pid_.compute(goal_error_wrt_robot.linear.x, time_step);
        const double dy = goal_y_pid_.compute(goal_error_wrt_robot.linear.y, time_step);
        const double dw = goal_w_pid_.compute(goal_error_wrt_robot.angular.z, time_step);
        ddx = dx - odom.twist.twist.linear.x;
        ddy = dy - odom.twist.twist.linear.y;
        ddw = dw - odom.twist.twist.angular.z;
    }
    else
    {
        //
        // PD path control
        //
        const double dx = k_prop_ * control_error_wrt_robot.linear.x;
        const double dy = k_prop_ * control_error_wrt_robot.linear.y;
        const double dw = k_prop_ * control_error_wrt_robot.angular.z;
        ddx = k_damp_ * (dx - odom.twist.twist.linear.x);
        ddy = k_damp_ * (dy - odom.twist.twist.linear.y);
        ddw = k_damp_ * (dw - odom.twist.twist.angular.z);
    }

    //
    // Max acceleration check
    //
    {
        double acc_factor_x = 1.0;
        if (std::abs(ddx) > max_acceleration_x_ && std::signbit(odom.twist.twist.linear.x) == std::signbit(ddx))
        {
            ROS_WARN_STREAM("Limiting maximum acceleration in X: " << ddx << " > " << max_acceleration_x_);
            acc_factor_x = std::abs(ddx / max_acceleration_x_);
        }

        double acc_factor_y = 1.0;
        if (std::abs(ddy) > max_acceleration_y_ && std::signbit(odom.twist.twist.linear.y) == std::signbit(ddy))
        {
            ROS_WARN_STREAM("Limiting maximum acceleration in Y: " << ddy << " > " << max_acceleration_y_);
            acc_factor_y = std::abs(ddy / max_acceleration_y_);
        }

        double acc_factor_w = 1.0;
        if (std::abs(ddw) > max_acceleration_w_ && std::signbit(odom.twist.twist.angular.z) == std::signbit(ddw))
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
    geometry_msgs::Twist velocity;
    velocity.linear.x = odom.twist.twist.linear.x + ddx * time_step;
    velocity.linear.y = odom.twist.twist.linear.y + ddy * time_step;
    velocity.angular.z = odom.twist.twist.angular.z + ddw * time_step;

    //
    // Max velocity check
    //
    {
        double velocity_factor_x = 1.0;
        if (std::abs(velocity.linear.x) > max_velocity_x_)
        {
            ROS_WARN_STREAM("Velocity in X exceeds limit: " << velocity.linear.x << " > " << max_velocity_x_);
            velocity_factor_x = std::abs(velocity.linear.x / max_velocity_x_);
        }

        double velocity_factor_y = 1.0;
        if (std::abs(velocity.linear.y) > max_velocity_y_)
        {
            ROS_WARN_STREAM("Velocity in Y exceeds limit: " << velocity.linear.y << " > " << max_velocity_y_);
            velocity_factor_y = std::abs(velocity.linear.y / max_velocity_y_);
        }

        const double velocity_factor = std::max(velocity_factor_x, velocity_factor_y);
        velocity.linear.x /= velocity_factor;
        velocity.linear.y /= velocity_factor;

        if (std::abs(velocity.angular.z) > max_velocity_w_)
        {
            ROS_WARN_STREAM("Velocity in W exceeds limit: " << velocity.angular.z << " > " << max_velocity_w_);
            const double scale = max_velocity_w_ / std::abs(velocity.angular.z);
            velocity.angular.z *= scale;
        }
    }

    control.cmd_vel = velocity;
    control.state = nav_core::ControlState::RUNNING;
    return control;
}
}
