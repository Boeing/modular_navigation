#include <gridmap/operations/rasterize.h>
#include <navigation_interface/params.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_list_macros.h>
#include <pure_pursuit_controller/plugin.h>
#include <visualization_msgs/MarkerArray.h>

PLUGINLIB_EXPORT_CLASS(pure_pursuit_controller::PurePursuitController, navigation_interface::Controller)

namespace pure_pursuit_controller
{

namespace
{

struct CollisionCheck
{
    bool in_collision;
    double min_distance_to_collision;
    visualization_msgs::Marker marker;
};

CollisionCheck robotInCollision(const gridmap::OccupancyGrid& grid, const Eigen::Isometry2d& robot_pose,
                                const std::vector<Eigen::Vector2d>& footprint, const float alpha)
{
    int min_x = std::numeric_limits<int>::max();
    int max_x = 0;

    int min_y = std::numeric_limits<int>::max();
    int max_y = 0;

    std::vector<Eigen::Array2i> map_footprint;
    for (const Eigen::Vector2d& p : footprint)
    {
        const Eigen::Vector2d world_point = robot_pose.translation() + robot_pose.rotation() * p;
        const auto map_point = grid.dimensions().getCellIndex(world_point);
        map_footprint.push_back(map_point);
        min_x = std::min(map_point.x(), min_x);
        max_x = std::max(map_point.x(), max_x);
        min_y = std::min(map_point.y(), min_y);
        max_y = std::max(map_point.y(), max_y);
    }
    map_footprint.push_back(map_footprint.front());

    const auto connected_poly = gridmap::connectPolygon(map_footprint);

    double min_distance_to_collision = std::numeric_limits<double>::max();
    {
        const cv::Mat cv_im = cv::Mat(grid.dimensions().size().y(), grid.dimensions().size().x(), CV_8U,
                                      reinterpret_cast<void*>(const_cast<uint8_t*>(grid.cells().data())));

        // Invert the costmap data such that all objects are considered zeros
        cv::Mat inv_cv_im;
        cv::bitwise_not(cv_im, inv_cv_im);

        // Calculate the distance transform to all objects (zero pixels)
        cv::Mat dist;
        cv::distanceTransform(inv_cv_im, dist, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);

        for (const auto& p : connected_poly)
        {
            const float f = dist.at<float>(p.y(), p.x());
            const double d = grid.dimensions().resolution() * f;
            min_distance_to_collision = std::min(min_distance_to_collision, d);
        }
    }

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
    auto append_raster = [&grid, &marker, &in_collision, alpha](const int x, const int y) {
        const Eigen::Array2i p{x, y};

        const Eigen::Vector2d w = grid.dimensions().getCellCenter(p);
        geometry_msgs::Point mp;
        mp.x = w.x();
        mp.y = w.y();
        mp.z = 0.0;
        marker.points.push_back(mp);
        std_msgs::ColorRGBA c;
        c.a = alpha;
        if (grid.dimensions().contains(p) && grid.occupied(p))
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

    // rasterPolygonFill is not properly including all edges
    for (const auto& p : connected_poly)
        append_raster(p.x(), p.y());

    return {in_collision, min_distance_to_collision, marker};
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

std::size_t findClosest(const double look_ahead_time, const std::vector<navigation_interface::KinodynamicState>& nodes,
                        const Eigen::Isometry2d& pose, const Eigen::Vector3d& velocity, const std::size_t path_index)
{
    if (nodes.size() <= 1)
        return 0;

    std::vector<double> distances;
    std::transform(nodes.begin(), nodes.end(), std::back_inserter(distances),
                   [&pose](const navigation_interface::KinodynamicState& state) {
                       const double alpha = 0.1;
                       const Eigen::Rotation2Dd rot(state.pose.linear().inverse() * pose.linear());
                       return ((state.pose.translation() - pose.translation()).norm()) +
                              alpha * std::abs(rot.smallestAngle());
                   });
    auto it_pose = std::min_element(distances.begin() + static_cast<long>(path_index), distances.end());
    const long dist_pose = std::distance(distances.begin(), it_pose);

    // Look at where the robot will be in the future
    Eigen::Isometry2d future_state = pose;
    future_state.pretranslate(look_ahead_time * Eigen::Vector2d(velocity.topRows(2)));
    future_state.rotate(Eigen::Rotation2Dd(look_ahead_time * velocity[2]));

    std::vector<double> future_distances;
    std::transform(nodes.begin(), nodes.end(), std::back_inserter(future_distances),
                   [&future_state](const navigation_interface::KinodynamicState& state) {
                       const double alpha = 0.1;
                       const Eigen::Rotation2Dd rot(state.pose.linear().inverse() * future_state.linear());
                       return ((state.pose.translation() - future_state.translation()).norm()) +
                              alpha * std::abs(rot.smallestAngle());
                   });
    auto it_fut = std::min_element(future_distances.begin() + dist_pose, future_distances.end());
    const long dist_fut = std::distance(future_distances.begin(), it_fut);

    return static_cast<std::size_t>(dist_fut);
}

navigation_interface::KinodynamicState targetState(const Eigen::Isometry2d& robot_pose,
                                                   const navigation_interface::Trajectory& trajectory,
                                                   const std::size_t path_index)
{
    double linear_distance = 0;
    double angular_distance = 0;
    for (std::size_t i = path_index; i < trajectory.states.size(); ++i)
    {
        const Eigen::Vector2d seg_vec =
            trajectory.states[i + 1].pose.translation() - trajectory.states[i].pose.translation();
        const Eigen::Vector2d seg_vec_wrt_robot = trajectory.states[i].pose.linear().inverse() * seg_vec;
        const Eigen::Vector3d seg_dist = {
            seg_vec_wrt_robot[0], seg_vec_wrt_robot[1],
            Eigen::Rotation2Dd(trajectory.states[i].pose.linear().inverse() * trajectory.states[i + 1].pose.linear())
                .smallestAngle()};
        linear_distance += seg_dist.topRows(2).norm();
        angular_distance += std::abs(seg_dist[2]);

        const double linear_d_from_robot = (robot_pose.translation() - trajectory.states[i].pose.translation()).norm();
        const double angular_d_from_robot = std::abs(
            Eigen::Rotation2Dd(trajectory.states[i].pose.linear().inverse() * robot_pose.linear()).smallestAngle());

        // force the target state to be at least a certain distance from the robot
        if (linear_d_from_robot < 0.04 && angular_d_from_robot < 0.2)
            continue;

        // only exit if the target state is a certain distance along the path from the closest point
        if (linear_distance > 0.04 || angular_distance > 0.2)
            return trajectory.states[i];
    }
    return trajectory.states.back();
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

    path_index_ = 1;  // first pose is under robot
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
    path_index_ = 0;
    control_integral_ = Eigen::Vector3d::Zero();
    control_error_ = Eigen::Vector3d::Zero();
    last_update_ = ros::SteadyTime(0);
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
    PurePursuitController::control(const ros::SteadyTime& time, const gridmap::AABB& local_region,
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

    const double dist_to_goal = (trajectory_->states.back().pose.translation() - robot_state.pose.translation()).norm();
    const double angle_to_goal =
        std::abs(Eigen::Rotation2Dd(robot_state.pose.linear().inverse() * trajectory_->states.back().pose.linear())
                     .smallestAngle());

    //
    // Goal condition
    //
    if (angle_to_goal < yaw_goal_tolerance_ && dist_to_goal < xy_goal_tolerance_)
    {
        result.outcome = navigation_interface::Controller::Outcome::COMPLETE;
        last_update_ = ros::SteadyTime(0);
        return result;
    }

    const bool final_pid_control = dist_to_goal < goal_radius_ && angle_to_goal < M_PI / 2;

    //
    // Find target
    //
    navigation_interface::KinodynamicState target_state;
    if (final_pid_control)
    {
        path_index_ = trajectory_->states.size() - 1;
        target_state = trajectory_->states.back();
    }
    else
    {
        path_index_ =
            findClosest(look_ahead_time_, trajectory_->states, robot_state.pose, robot_state.velocity, path_index_);
        target_state = targetState(robot_state.pose, *trajectory_, path_index_);
    }

    //
    // Check immediate collisions
    //
    double min_distance_to_collision;
    {
        // TODO need a more intelligent way to lock read access without waiting for path planning
        // auto lock = map_data_->grid.getLock();
        gridmap::OccupancyGrid local_grid(map_data_->grid, local_region);
        // lock.unlock();

        const Eigen::Isometry2d map_robot_pose = map_to_odom * robot_state.pose;
        const Eigen::Isometry2d map_goal_pose = map_to_odom * target_state.pose;

        const CollisionCheck cc = robotInCollision(local_grid, map_robot_pose, robot_footprint_, 1.f);
        min_distance_to_collision = cc.min_distance_to_collision;
        if (debug_viz_)
            footprint_pub_.publish(cc.marker);

        const CollisionCheck f_cc = robotInCollision(local_grid, map_goal_pose, robot_footprint_, 0.2f);
        if (debug_viz_)
            future_footprint_pub_.publish(f_cc.marker);

        if (cc.in_collision)
        {
            ROS_WARN_STREAM("Current robot pose is in collision!");
            result.outcome = navigation_interface::Controller::Outcome::FAILED;
            last_update_ = ros::SteadyTime(0);
            return result;
        }

        if (f_cc.in_collision)
        {
            ROS_WARN_STREAM("Target robot pose is in collision!");
            result.outcome = navigation_interface::Controller::Outcome::FAILED;
            last_update_ = ros::SteadyTime(0);
            return result;
        }
    }

    if (debug_viz_)
        target_state_pub_.publish(buildMarker(robot_state, target_state));

    ROS_ASSERT(robot_state.pose.linear().allFinite());
    ROS_ASSERT(robot_state.pose.translation().allFinite());

    const Eigen::Vector2d target_vec_wrt_robot =
        robot_state.pose.linear().inverse() * (target_state.pose.translation() - robot_state.pose.translation());
    const Eigen::Vector3d control_error = {
        target_vec_wrt_robot[0], target_vec_wrt_robot[1],
        Eigen::Rotation2Dd(robot_state.pose.linear().inverse() * target_state.pose.linear()).smallestAngle()};
    ROS_ASSERT(control_error.allFinite());

    Eigen::Vector3d target_velocity;
    if (final_pid_control)
    {
        control_integral_ += control_error;

        const Eigen::Vector3d control_dot_ = (control_error - control_error_) / dt;
        control_error_ = control_error;
        ROS_ASSERT(control_error_.allFinite());

        target_velocity = goal_p_gain_.cwiseProduct(control_error) + goal_i_gain_.cwiseProduct(control_integral_) +
                          goal_d_gain_.cwiseProduct(control_dot_);
    }
    else
    {
        const auto max_vel =
            max_velocity_ * std::max(0.20, std::min(1.0, std::min(dist_to_goal, min_distance_to_collision)));
        target_velocity = control_error.normalized().cwiseProduct(max_vel);
        control_integral_ = Eigen::Vector3d::Zero();
        control_error_ = Eigen::Vector3d::Zero();
    }
    ROS_ASSERT_MSG(target_velocity.allFinite(), "%f %f %f", target_velocity[0], target_velocity[1], target_velocity[2]);

    Eigen::Vector3d accel = (target_velocity - robot_state.velocity) / dt;

    //
    // Max acceleration check
    //
    {
        double acc_factor = 1.0;
        for (long i = 0; i < 3; ++i)
        {
            if (std::abs(accel[i]) > max_acceleration_[i])
            {
                acc_factor = std::max(acc_factor, std::abs(accel[i] / max_acceleration_[i]));
            }
        }

        accel /= acc_factor;
        ROS_ASSERT(accel.allFinite());
    }

    //
    // Integrate new velocity command
    //
    Eigen::Vector3d vel_command = robot_state.velocity + accel * dt;
    ROS_ASSERT(vel_command.allFinite());

    //
    // Max velocity check
    //
    {
        double vel_factor = 1.0;
        for (long i = 0; i < 3; ++i)
        {
            if (std::abs(vel_command[i]) > max_velocity_[i])
            {
                vel_factor = std::max(vel_factor, std::abs(vel_command[i] / max_velocity_[i]));
            }
        }

        vel_command /= vel_factor;
        ROS_ASSERT(vel_command.allFinite());
    }

    result.command = vel_command;
    result.outcome = navigation_interface::Controller::Outcome::SUCCESSFUL;

    return result;
}

// cppcheck-suppress unusedFunction
void PurePursuitController::onInitialize(const XmlRpc::XmlRpcValue& parameters)
{
    look_ahead_time_ = navigation_interface::get_config_with_default_warn<double>(
        parameters, "look_ahead_time", look_ahead_time_, XmlRpc::XmlRpcValue::TypeDouble);
    max_velocity_[0] = navigation_interface::get_config_with_default_warn<double>(
        parameters, "max_velocity_x", max_velocity_[0], XmlRpc::XmlRpcValue::TypeDouble);
    max_velocity_[1] = navigation_interface::get_config_with_default_warn<double>(
        parameters, "max_velocity_y", max_velocity_[1], XmlRpc::XmlRpcValue::TypeDouble);
    max_velocity_[2] = navigation_interface::get_config_with_default_warn<double>(
        parameters, "max_velocity_w", max_velocity_[2], XmlRpc::XmlRpcValue::TypeDouble);

    max_acceleration_[0] = navigation_interface::get_config_with_default_warn<double>(
        parameters, "max_acceleration_x", max_acceleration_[0], XmlRpc::XmlRpcValue::TypeDouble);
    max_acceleration_[1] = navigation_interface::get_config_with_default_warn<double>(
        parameters, "max_acceleration_y", max_acceleration_[1], XmlRpc::XmlRpcValue::TypeDouble);
    max_acceleration_[2] = navigation_interface::get_config_with_default_warn<double>(
        parameters, "max_acceleration_w", max_acceleration_[2], XmlRpc::XmlRpcValue::TypeDouble);

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
        robot_footprint_ = {{0.480, 0.000},  {0.380, -0.395}, {-0.380, -0.395},
                            {-0.480, 0.000}, {-0.380, 0.395}, {0.380, 0.395}};
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