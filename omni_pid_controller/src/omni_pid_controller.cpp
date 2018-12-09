#include <omni_pid_controller/omni_pid_controller.h>

#include <algorithm>

#include <costmap_2d/cost_values.h>

#include <tf2/utils.h>

#include <pluginlib/class_list_macros.h>

#include <ompl/geometric/PathSimplifier.h>

PLUGINLIB_DECLARE_CLASS(omni_pid_controller, OmniPIDController, omni_pid_controller::OmniPIDController,
                        nav_core::BaseLocalPlanner)

namespace omni_pid_controller
{

namespace
{

double getYaw(const Eigen::Quaterniond& q)
{
    double yaw;

    double sqw;
    double sqx;
    double sqy;
    double sqz;

    sqx = q.x() * q.x();
    sqy = q.y() * q.y();
    sqz = q.z() * q.z();
    sqw = q.w() * q.w();

    // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
    double sarg =
        -2 * (q.x() * q.z() - q.w() * q.y()) / (sqx + sqy + sqz + sqw); /* normalization added from urdfom_headers */

    if (sarg <= -0.99999)
    {
        yaw = -2 * atan2(q.y(), q.x());
    }
    else if (sarg >= 0.99999)
    {
        yaw = 2 * atan2(q.y(), q.x());
    }
    else
    {
        yaw = atan2(2 * (q.x() * q.y() + q.w() * q.z()), sqw + sqx - sqy - sqz);
    }
    return yaw;
};

bool isDiff(const Eigen::Isometry2d& t1, const Eigen::Isometry2d& t2)
{
    const bool trans = (t1.translation() - t2.translation()).norm() > 1e-4;
    const bool rot = Eigen::Rotation2D<double>((t2.inverse() * t1).rotation()).angle() > 1e-4;
    return trans || rot;
}

bool isDiff(const Eigen::Isometry3d& t1, const Eigen::Isometry3d& t2)
{
    const bool trans = (t1.translation() - t2.translation()).norm() > 1e-4;
    const bool rot = Eigen::AngleAxisd((t2.inverse() * t1).rotation()).angle() > 1e-4;
    return trans || rot;
}
}

unsigned char getCost(const costmap_2d::Costmap2D& costmap, const double x, const double y)
{
    unsigned int cell_x, cell_y;
    unsigned char cost;
    if (!costmap.worldToMap(x, y, cell_x, cell_y))
    {
        // probably at the edge of the costmap - this value should be recovered soon
        cost = 1;
    }
    else
    {
        // get cost for this cell
        cost = costmap.getCost(cell_x, cell_y);
    }

    return cost;
}

double getDistanceToCollision(const costmap_2d::Costmap2D& costmap, const double x, const double y,
                              const double inflation_weight)
{
    return getDistanceToCollision(getCost(costmap, x, y), inflation_weight);
}

double getDistanceToCollision(const unsigned char cost, const double inflation_weight)
{
    if (cost >= costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
        return 0.0;
    }
    else
    {
        const double c =
            (cost != costmap_2d::FREE_SPACE && cost != costmap_2d::NO_INFORMATION) ? static_cast<double>(cost) : 1.0;
        const double factor = static_cast<double>(c) / (costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1);
        return -log(factor) / inflation_weight;
    }
}

OmniPIDController::OmniPIDController() : costmap_ros_(nullptr)
{
}

OmniPIDController::~OmniPIDController()
{
}

nav_core::Control OmniPIDController::computeControl(const ros::SteadyTime& steady_time, const ros::Time& ros_time,
                                                    const nav_msgs::Odometry& odom)
{
    nav_core::Control control;

    std::lock_guard<std::mutex> lock(control_mutex_);

    //
    // Update tracking state
    //
    const ros::SteadyTime last_time_step = control_data_->last_time_step;
    control_data_->last_time_step = steady_time;

    const bool has_control = bool(control_data_);
    if (!has_control)
    {
        control.state = nav_core::ControlState::COMPLETE;
        goal_x_pid_.reset();
        goal_y_pid_.reset();
        tracking_x_pid_.reset();
        tracking_y_pid_.reset();
        rotation_pid_.reset();
        return control;
    }

    const std::string robot_frame = costmap_ros_->getBaseFrameID();
    const std::string local_frame = costmap_ros_->getGlobalFrameID();

    // Get the global costmap to local costmap transform
    try
    {
        const std::string global_frame = control_data_->global_path.front().header.frame_id;
        const geometry_msgs::TransformStamped tr =
            tf_buffer_->lookupTransform(global_frame, local_frame, ros_time, ros::Duration(0.10));
        const Eigen::Quaterniond qt = Eigen::Quaterniond(tr.transform.rotation.w, tr.transform.rotation.x,
                                                         tr.transform.rotation.y, tr.transform.rotation.z);
        const Eigen::Translation3d p =
            Eigen::Translation3d(tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z);
        control_data_->global_to_local = p * qt;
    }
    catch (const tf2::TransformException& e)
    {
        ROS_WARN_STREAM("Failed to get global->local costmap transform: " << e.what());
        control.state = nav_core::ControlState::FAILED;
        goal_x_pid_.reset();
        goal_y_pid_.reset();
        tracking_x_pid_.reset();
        tracking_y_pid_.reset();
        rotation_pid_.reset();
        return control;
    }

    //
    // Transform the global path into the local frame
    //
    control_data_->local_trajectory.clear();
    for (const geometry_msgs::PoseStamped& _pose : control_data_->global_path)
    {
        const Eigen::Quaterniond qt = Eigen::Quaterniond(_pose.pose.orientation.w, _pose.pose.orientation.x,
                                                         _pose.pose.orientation.y, _pose.pose.orientation.z);
        const Eigen::Translation3d p =
            Eigen::Translation3d(_pose.pose.position.x, _pose.pose.position.y, _pose.pose.position.z);
        const Eigen::Isometry3d pose = p * qt;
        const Eigen::Isometry3d transformed_pose = control_data_->global_to_local * pose;
        control_data_->local_trajectory.push_back(transformed_pose);
    }

    //
    // Make sure odom is not too old
    //
    const double maximum_odom_time_delay = 0.1;
    ROS_INFO_STREAM("ros_time: " << ros_time);
    ROS_INFO_STREAM("odom: " << odom.header.stamp);
    const double odom_time_delay = (ros_time - odom.header.stamp).toSec();
    if (odom_time_delay > maximum_odom_time_delay)
    {
        ROS_WARN_STREAM("Odometry is too old: delay = " << odom_time_delay);
        control.state = nav_core::ControlState::FAILED;
        control.error = nav_core::ControlFailure::BAD_ODOMETRY;
        goal_x_pid_.reset();
        goal_y_pid_.reset();
        tracking_x_pid_.reset();
        tracking_y_pid_.reset();
        rotation_pid_.reset();
        return control;
    }

    //
    // Get the current robot pose from odom
    //
    const Eigen::Quaterniond qt = Eigen::Quaterniond(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                                                     odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
    const Eigen::Translation3d p =
        Eigen::Translation3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
    const Eigen::Isometry3d robot_pose = p * qt;

    //
    // Find the point currently executing
    //
    std::size_t min_i = control_data_->execution_index;
    double linear_distance = std::numeric_limits<double>::max();
    for (std::size_t i = control_data_->execution_index; i < control_data_->local_trajectory.size(); ++i)
    {
        const Eigen::Isometry3d _pose = control_data_->local_trajectory[i];
        const double linear_delta = (_pose.translation() - robot_pose.translation()).norm();

        // Waypoint reached
        if (linear_delta < goal_radius_)
            continue;

        if (linear_delta < linear_distance)
        {
            min_i = i;
            linear_distance = linear_delta;
        }
    }
    control_data_->execution_index = min_i;
    assert(control_data_->execution_index > 0);
    const bool last_waypoint = min_i >= control_data_->local_trajectory.size() - 1;
    ROS_INFO_STREAM("Tracking point: " << (min_i + 1) << " of " << control_data_->local_trajectory.size());

    //
    // Determine the tracking error
    //
    const Eigen::Isometry3d start_p = control_data_->local_trajectory.at(min_i - 1);
    const Eigen::Isometry3d end_p = control_data_->local_trajectory.at(min_i);
    const Eigen::Vector3d segment = end_p.translation() - start_p.translation();
    const Eigen::Vector3d segment_wrtb = robot_pose.rotation().inverse() * segment;
    const Eigen::ParametrizedLine<double, 3> line =
        Eigen::ParametrizedLine<double, 3>::Through(start_p.translation(), end_p.translation());
    const Eigen::Vector3d point_along_segment = line.projection(robot_pose.translation());
    const Eigen::Vector3d linear_error = point_along_segment - robot_pose.translation();
    const Eigen::Vector3d goal_error = end_p.translation() - robot_pose.translation();
    const Eigen::Vector3d linear_error_wrtb = robot_pose.rotation().inverse() * linear_error;
    const Eigen::Vector3d goal_error_wrtb = robot_pose.rotation().inverse() * goal_error;

    //
    // Determine the rotational tracking error
    //
    const double segment_rot =
        getYaw(Eigen::Quaterniond(start_p.rotation()).inverse() * Eigen::Quaterniond(end_p.rotation()));
    const double angular_error =
        getYaw(Eigen::Quaterniond(robot_pose.rotation()).inverse() * Eigen::Quaterniond(end_p.rotation()));

    //
    // Termination criteria
    //
    if (last_waypoint && goal_error.norm() < goal_linear_threshold && angular_error < goal_angular_threshold)
    {
        ROS_INFO_STREAM("linear_error: " << linear_error.transpose());
        ROS_INFO_STREAM("linear_error.norm(): " << linear_error.norm());
        ROS_INFO_STREAM("goal_error: " << goal_error.transpose());
        ROS_INFO_STREAM("goal_error.norm(): " << goal_error.transpose());
        ROS_INFO_STREAM("angular_error: " << angular_error);
        control.state = nav_core::ControlState::COMPLETE;
        goal_x_pid_.reset();
        goal_y_pid_.reset();
        tracking_x_pid_.reset();
        tracking_y_pid_.reset();
        rotation_pid_.reset();
        return control;
    }

    //
    // Failure conditions
    //
    if (linear_error.norm() > max_linear_tracking_error_)
    {
        ROS_WARN_STREAM("Exceeded maximum linear tracking error: " << linear_error.norm() << " > "
                                                                   << max_linear_tracking_error_);
        control.state = nav_core::ControlState::FAILED;
        control.error = nav_core::ControlFailure::TRACKING_LIMIT_EXCEEDED;
        goal_x_pid_.reset();
        goal_y_pid_.reset();
        tracking_x_pid_.reset();
        tracking_y_pid_.reset();
        rotation_pid_.reset();
        return control;
    }

    //
    // Check current position is collision free
    //
    const unsigned char cost =
        getCost(*costmap_ros_->getCostmap(), robot_pose.translation().x(), robot_pose.translation().y());
    if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
        ROS_WARN_STREAM("Robot in collision!");
        control.state = nav_core::ControlState::EMERGENCY_BRAKING;
        goal_x_pid_.reset();
        goal_y_pid_.reset();
        tracking_x_pid_.reset();
        tracking_y_pid_.reset();
        rotation_pid_.reset();
        return control;
    }

    //
    // Check the remaining trajectory is collision free
    //
    const double minimum_distance_to_collision = 0.25;
    double accum_distance = 0.0;
    for (std::size_t i = control_data_->execution_index; i < control_data_->local_trajectory.size(); ++i)
    {
        const Eigen::Isometry3d& p = control_data_->local_trajectory[i];
        const Eigen::Isometry3d& prev_p = control_data_->local_trajectory[i - 1];

        accum_distance += (p.translation() - prev_p.translation()).norm();

        const unsigned char cost = getCost(*costmap_ros_->getCostmap(), p.translation().x(), p.translation().y());

        //
        // Execute evasive action if in collision
        //
        if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
            ROS_WARN_STREAM("Collision at point: " << i << " of " << control_data_->local_trajectory.size());
            if (accum_distance < minimum_distance_to_collision)
            {
                ROS_WARN_STREAM("Imminent collision in " << accum_distance << "m");
                goal_x_pid_.reset();
                goal_y_pid_.reset();
                tracking_x_pid_.reset();
                tracking_y_pid_.reset();
                rotation_pid_.reset();
                control.state = nav_core::ControlState::EMERGENCY_BRAKING;
                return control;
            }
        }
    }

    double time_step = steady_time.toSec() - last_time_step.toSec();
    if (time_step < 0)
        time_step = std::numeric_limits<double>::min();
    ROS_INFO_STREAM("time_step: " << time_step);

    //
    // Determine target speed (based on cost of goal and current cost)
    //
    const unsigned char target_cost =
        getCost(*costmap_ros_->getCostmap(), end_p.translation().x(), end_p.translation().y());
    const double speed_factor =
        std::max(0.2, (static_cast<double>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE - std::max(cost, target_cost)) /
                       static_cast<double>(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)));

    const double w_time = std::abs(segment_rot / (max_velocity_w_ / 2.0));
    const double x_time = std::abs(segment_wrtb[0] / max_velocity_x_);
    const double y_time = std::abs(segment_wrtb[1] / max_velocity_y_);
    const double max_time = std::max(w_time, std::max(x_time, y_time));

    ROS_INFO_STREAM("w_time: " << w_time);
    ROS_INFO_STREAM("x_time: " << x_time);
    ROS_INFO_STREAM("y_time: " << y_time);
    ROS_INFO_STREAM("max_time: " << max_time);

    const double speed_x = max_velocity_x_ * x_time / max_time;
    const double speed_y = max_velocity_y_ * y_time / max_time;
    // const double speed_w = max_velocity_w_ * w_time / max_time;

    //
    // Slow down at end
    //
    const double slow_distance = 0.2;
    double slow_factor = 1.0;
    if (accum_distance < 0.2)
    {
        slow_factor *= std::max(0.2, (accum_distance / slow_distance));
    }

    ROS_INFO_STREAM("speed_factor: " << speed_factor);
    ROS_INFO_STREAM("slow_factor: " << slow_factor);

    ROS_INFO_STREAM("segment_rot: " << segment_rot);
    ROS_INFO_STREAM("segment_wrtb: " << segment_wrtb.transpose());

    ROS_INFO_STREAM("angular_error: " << angular_error);
    ROS_INFO_STREAM("goal_error_wrtb: " << goal_error_wrtb.transpose());
    ROS_INFO_STREAM("linear_error_wrtb: " << linear_error_wrtb.transpose());

    //
    // If we are within the goal radius then control on absolute error
    // If we are outside the goal radius then control on tracking error + segment velocity
    //
    double vx = 0;
    double vy = 0;
    if (goal_error_wrtb.norm() < goal_radius_)
    {
        ROS_INFO("TRACKING TO GOAL");

        vx = goal_x_pid_.compute(goal_error_wrtb[0], time_step);
        vy = goal_y_pid_.compute(goal_error_wrtb[1], time_step);
    }
    else
    {
        ROS_INFO_STREAM("TRACKING ALONG LINE");

        vx = tracking_x_pid_.compute(linear_error_wrtb[0], time_step);
        vy = tracking_y_pid_.compute(linear_error_wrtb[1], time_step);

        Eigen::Vector3d goal_dir_wrtb =
            robot_pose.rotation().inverse() * (end_p.translation() - robot_pose.translation());
        goal_dir_wrtb.normalize();

        ROS_INFO_STREAM("goal_dir_wrtb: " << goal_dir_wrtb.transpose());

        ROS_INFO_STREAM("speed_x: " << speed_x);
        ROS_INFO_STREAM("speed_y: " << speed_y);

        const double goal_x = goal_dir_wrtb[0] * speed_x * speed_factor * slow_factor;
        const double goal_y = goal_dir_wrtb[1] * speed_y * speed_factor * slow_factor;

        ROS_INFO_STREAM("PID X: " << vx);
        ROS_INFO_STREAM("PID Y: " << vy);

        ROS_INFO_STREAM("GOAL X: " << goal_x);
        ROS_INFO_STREAM("GOAL Y: " << goal_y);

        vx += goal_x;
        vy += goal_y;

        ROS_INFO("after PID");
        ROS_INFO_STREAM("vx: " << vx);
        ROS_INFO_STREAM("vy: " << vy);
    }

    if (angular_error > max_angular_tracking_error_)
    {
        ROS_WARN_STREAM("Exceeded maximum angular tracking error: " << angular_error << " > "
                                                                    << max_angular_tracking_error_);
        vx = 0;
        vy = 0;
    }

    ROS_INFO_STREAM("previous v");
    ROS_INFO_STREAM("old_vx: " << odom.twist.twist.linear.x);
    ROS_INFO_STREAM("old_vy: " << odom.twist.twist.linear.y);

    const double d_vx = vx - odom.twist.twist.linear.x;
    const double d_vy = vy - odom.twist.twist.linear.y;

    double acc_factor_x = 1.0;
    if (std::abs(d_vx) > max_acceleration_x_ && std::signbit(odom.twist.twist.linear.x) == std::signbit(d_vx))
    {
        ROS_WARN_STREAM("Limiting maximum acceleration in X: " << d_vx << " > " << max_acceleration_x_);
        acc_factor_x = std::abs(d_vx / max_acceleration_x_);
    }

    double acc_factor_y = 1.0;
    if (std::abs(d_vy) > max_acceleration_y_ && std::signbit(odom.twist.twist.linear.y) == std::signbit(d_vy))
    {
        ROS_WARN_STREAM("Limiting maximum acceleration in Y: " << d_vy << " > " << max_acceleration_y_);
        acc_factor_y = std::abs(d_vy / max_acceleration_y_);
    }

    const double acc_factor = std::max(acc_factor_x, acc_factor_y);
    vx = odom.twist.twist.linear.x + d_vx / acc_factor;
    vy = odom.twist.twist.linear.y + d_vy / acc_factor;

    ROS_INFO_STREAM("after acceleration limiting: " << acc_factor);
    ROS_INFO_STREAM("vx: " << vx);
    ROS_INFO_STREAM("vy: " << vy);

    double velocity_factor_x = 1.0;
    if (std::abs(vx) > max_velocity_x_)
    {
        ROS_WARN_STREAM("Velocity in X exceeds limit: " << vx << " > " << max_velocity_x_);
        velocity_factor_x = std::abs(vx / max_velocity_x_);
    }

    double velocity_factor_y = 1.0;
    if (std::abs(vy) > max_velocity_y_)
    {
        ROS_WARN_STREAM("Velocity in Y exceeds limit: " << vy << " > " << max_velocity_y_);
        velocity_factor_y = std::abs(vy / max_velocity_y_);
    }

    const double velocity_factor = std::max(velocity_factor_x, velocity_factor_y);
    vx /= velocity_factor;
    vy /= velocity_factor;

    ROS_INFO_STREAM("after velocity limiting: " << velocity_factor);
    ROS_INFO_STREAM("vx: " << vx);
    ROS_INFO_STREAM("vy: " << vy);

    double vw = rotation_pid_.compute(angular_error, time_step);
    double d_vw = vw - odom.twist.twist.angular.z;

    double acc_factor_w = 1.0;
    if (std::abs(d_vw) > max_acceleration_w_)
    {
        ROS_WARN_STREAM("Limiting maximum acceleration in W: " << d_vw << " > " << max_acceleration_w_);
        acc_factor_w = std::abs(d_vw / max_acceleration_w_);
    }

    vw = odom.twist.twist.angular.z + d_vw / acc_factor;

    double velocity_factor_w = 1.0;
    if (std::abs(vw) > max_velocity_w_)
    {
        ROS_WARN_STREAM("Velocity in W exceeds limit: " << vw << " > " << max_velocity_w_);
        velocity_factor_w = std::abs(vw / max_velocity_w_);
    }

    vw /= velocity_factor_w;

    control.cmd_vel.linear.x = vx;
    control.cmd_vel.linear.y = vy;
    control.cmd_vel.angular.z = vw;

    ROS_INFO_STREAM("v_x: " << control.cmd_vel.linear.x);
    ROS_INFO_STREAM("v_y: " << control.cmd_vel.linear.y);
    ROS_INFO_STREAM("v_w: " << control.cmd_vel.angular.z);

    control.state = nav_core::ControlState::RUNNING;
    return control;
}

bool OmniPIDController::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    std::lock_guard<std::mutex> lock(control_mutex_);

    control_data_ = std::unique_ptr<ControlData>(new ControlData());
    control_data_->global_path = plan;
    control_data_->execution_index = 1;
    goal_x_pid_.reset();
    goal_y_pid_.reset();
    tracking_x_pid_.reset();
    tracking_y_pid_.reset();
    rotation_pid_.reset();

    return true;
}

bool OmniPIDController::clearPlan()
{
    std::lock_guard<std::mutex> c_lock(control_mutex_);
    control_data_.reset(nullptr);

    return true;
}

void OmniPIDController::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    tf_buffer_ = tf;
    costmap_ros_ = costmap_ros;

    // Assert the costmap is in odom
    const std::string local_frame = costmap_ros_->getGlobalFrameID();
    if (local_frame != "odom")
    {
        throw std::runtime_error("Local costmap must be in odom frame for robust trajectory control");
    }
}
}
