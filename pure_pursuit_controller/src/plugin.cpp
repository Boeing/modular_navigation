#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
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

typedef boost::geometry::model::d2::point_xy<double> Point;  // Point type for boost polygon library

typedef boost::geometry::model::polygon<Point,           // Point type
                                        true,            // Orientation is clockwise
                                        false,           // Closure is implied (last point != first point)
                                        std::vector,     // Store for points (efficient if all points known in advance)
                                        std::vector,     // Store for rings (efficient if all holes known in advance)
                                        std::allocator,  // Point allocator
                                        std::allocator   // Ring allocator
                                        >
    Polygon;

struct CollisionCheck
{
    bool in_collision;
    double min_distance_to_collision;
    visualization_msgs::Marker marker;
};

CollisionCheck robotInCollision(const gridmap::OccupancyGrid& grid, const Eigen::Isometry2d& robot_pose,
                                const Eigen::Isometry2d& future_pose, const std::vector<Eigen::Vector2d>& footprint,
                                const float alpha)
{
    // Want to interpolate footprint between current robot pose and future robot pose
    const double linear_step = 0.01;
    const double angular_step = 0.04;
    const size_t max_steps = 20;
    const size_t min_steps = 2;

    // First create interpolated poses
    std::vector<Eigen::Isometry2d> interpolated;
    interpolated.push_back(robot_pose);

    const Eigen::Vector2d linear_dir = future_pose.translation() - robot_pose.translation();
    const double linear_dist = linear_dir.norm();
    const Eigen::Rotation2Dd rot(robot_pose.linear());
    const Eigen::Rotation2Dd future_rot(future_pose.linear());
    const Eigen::Rotation2Dd rot_dir(future_rot.inverse() * rot);
    const double rot_dist = std::abs(rot_dir.smallestAngle());

    const std::size_t steps =
        std::min(max_steps, std::max(min_steps, static_cast<std::size_t>(
                                                    std::max((linear_dist / linear_step), (rot_dist / angular_step)))));

    for (std::size_t j = 1; j < steps; j++)
    {
        const double fraction = static_cast<double>(j + 1) / static_cast<double>(steps);
        const Eigen::Isometry2d pose =
            Eigen::Translation2d(robot_pose.translation() + fraction * linear_dir) * rot.slerp(fraction, future_rot);
        interpolated.push_back(pose);
    }

    // Create vector of polygons. Polygon at each pose starting from robot pose
    std::vector<Polygon> polygons;
    for (const Eigen::Isometry2d& pose : interpolated)
    {
        Polygon polygon;
        for (const Eigen::Vector2d& p : footprint)
        {
            const Eigen::Vector2d world_point = pose.translation() + pose.rotation() * p;
            Point point(world_point[0], world_point[1]);
            boost::geometry::append(polygon.outer(), point);
        }
        polygons.push_back(polygon);
    }

    // Union the footprints
    Polygon union_output = polygons.front();
    for (std::size_t i = 1; i < polygons.size(); i++)
    {
        std::vector<Polygon> temp;
        boost::geometry::union_(union_output, polygons[i], temp);
        ROS_ASSERT_MSG(temp.size() == 1, "Footprint union failed. More than 1 resulting polygons");
        union_output = temp[0];
    }

    int min_x = std::numeric_limits<int>::max();
    int max_x = 0;

    int min_y = std::numeric_limits<int>::max();
    int max_y = 0;

    std::vector<Eigen::Array2i> map_footprint;
    for (const Point& p : union_output.outer())
    {
        const auto map_point = grid.dimensions().getCellIndex(Eigen::Vector2d(p.x(), p.y()));
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
        cv::Mat distance;
        cv::distanceTransform(inv_cv_im, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);

        for (const auto& p : connected_poly)
        {
            const double f = static_cast<double>(distance.at<float>(p.y(), p.x()));
            min_distance_to_collision = std::min(min_distance_to_collision, f);
        }
        min_distance_to_collision *= grid.dimensions().resolution();
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

// We can treat rotation like a Z axis, so we can calculate a 3D norm between any two poses
double dist(const Eigen::Isometry2d& pose_1, const Eigen::Isometry2d& pose_2, const double a_weight)
{
    const double diff_x = pose_2.translation().x() - pose_1.translation().x();
    const double diff_y = pose_2.translation().y() - pose_1.translation().y();
    const double diff_a = Eigen::Rotation2Dd(pose_1.linear().inverse() * pose_2.linear()).smallestAngle();

    return (diff_x * diff_x) + (diff_y * diff_y) + a_weight * (diff_a * diff_a);
}

// Find the node on the path that is closest to the robot.
// If the robot is between two nodes, it will always pick the one in front to prevent back-tracking
// Because the nodes aren't evenly spaced, this is actually not a trivial problem
std::size_t findClosest(const std::vector<navigation_interface::KinodynamicState>& nodes, const Eigen::Isometry2d& pose)
{
    if (nodes.size() <= 2)
        return nodes.size() - 1;

    std::vector<double> distances;
    std::transform(
        nodes.begin(), nodes.end(), std::back_inserter(distances),
        [&pose](const navigation_interface::KinodynamicState& state) { return dist(state.pose, pose, 0.1); });
    const auto closest_it = std::min_element(distances.begin(), distances.end());
    const std::size_t closest_i = std::distance(distances.begin(), closest_it);

    if (closest_i == 0 || closest_i == nodes.size() - 1)
    {
        return static_cast<std::size_t>(closest_i);
    }

    // Resolve ambiguity by checking points slightly before and after the closest node
    // Find midpoints  -----A------M1---closest_i---M2-------B------->
    Eigen::Isometry2d m_1;
    Eigen::Isometry2d m_2;
    {
        // Calculate pose of M1
        const Eigen::Vector2d translation_vec =
            nodes[closest_i].pose.translation() - nodes[closest_i - 1].pose.translation();
        const Eigen::Rotation2Dd rot(nodes[closest_i - 1].pose.linear());
        const Eigen::Rotation2Dd target_rot(nodes[closest_i].pose.linear());

        m_1 = Eigen::Translation2d(nodes[closest_i - 1].pose.translation() + 0.99 * translation_vec) *
              rot.slerp(0.99, target_rot);
    }

    {
        // Calculate pose of M2
        const Eigen::Vector2d translation_vec =
            nodes[closest_i + 1].pose.translation() - nodes[closest_i].pose.translation();
        const Eigen::Rotation2Dd rot(nodes[closest_i].pose.linear());
        const Eigen::Rotation2Dd target_rot(nodes[closest_i + 1].pose.linear());

        m_2 = Eigen::Translation2d(nodes[closest_i].pose.translation() + 0.01 * translation_vec) *
              rot.slerp(0.01, target_rot);
    }

    if (dist(m_1, pose, 0.1) < dist(m_2, pose, 0.1))
    {
        return static_cast<std::size_t>(closest_i);
    }
    else
    {
        return static_cast<std::size_t>(closest_i + 1);
    }
}

// Performs a lookahead to give the robot a "carrot" to chase
navigation_interface::KinodynamicState lookAhead(const std::vector<navigation_interface::KinodynamicState>& nodes,
                                                 const Eigen::Isometry2d& robot_pose, const Eigen::Vector3d& velocity,
                                                 const std::size_t path_index, const double look_ahead_time,
                                                 const unsigned int interpolation_steps)
{
    Eigen::Isometry2d future_pose = robot_pose;
    future_pose.pretranslate(look_ahead_time * Eigen::Vector2d(velocity.topRows(2)));
    future_pose.rotate(Eigen::Rotation2Dd(look_ahead_time * velocity[2]));

    // To prevent the robot from getting stuck with 0 lookahead, set a minimum of 5cm
    // Note: these are squared distances!
    const double required_distance = std::max(0.0025, dist(robot_pose, future_pose, 0.1));

    for (std::size_t i = path_index; i < nodes.size(); ++i)
    {
        const double dist_to_node = dist(robot_pose, nodes[i].pose, 0.1);

        // Found first node that is far enough. Now interpolate between previous node and this node.
        if (dist_to_node > required_distance)
        {
            const Eigen::Isometry2d start_pose = (i == 0) ? robot_pose : nodes[i - 1].pose;
            const Eigen::Vector2d translation_vec = nodes[i].pose.translation() - start_pose.translation();
            const Eigen::Rotation2Dd rot(start_pose.linear());
            const Eigen::Rotation2Dd target_rot(nodes[i].pose.linear());

            Eigen::Isometry2d prev_interpolated_pose = nodes[i].pose;

            for (int j = interpolation_steps; j >= 0; j--)
            {
                // Step from front to back to avoid finding a point behind the robot.
                const double fraction = static_cast<double>(j) / static_cast<double>(interpolation_steps);
                const Eigen::Isometry2d interpolated_pose =
                    Eigen::Translation2d(start_pose.translation() + fraction * translation_vec) *
                    rot.slerp(fraction, target_rot);

                // Found pose just within range. Return the previous one (just outside)
                if (dist(interpolated_pose, robot_pose, 0.1) < required_distance)
                {
                    navigation_interface::KinodynamicState target_state = nodes[i];
                    target_state.pose = prev_interpolated_pose;
                    return target_state;
                }

                prev_interpolated_pose = interpolated_pose;
            }

            // None of the interpolated poses are within required_distance. Just use the node itself.
            return nodes[i];
        }
    }
    return nodes.back();
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
    last_update_ = ros::SteadyTime::now();

    return true;
}

// cppcheck-suppress unusedFunction
void PurePursuitController::clearTrajectory()
{
    trajectory_.reset();
    path_index_ = 0;
    control_integral_ = Eigen::Vector3d::Zero();
    control_error_ = Eigen::Vector3d::Zero();
    last_update_ = ros::SteadyTime::now();
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

    double dt = time.toSec() - last_update_.toSec();
    ROS_ASSERT_MSG(dt > 0.0, "Pure Pursuit - Negative time step!");
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
        ROS_INFO_STREAM("Control Complete! angle_to_goal: " << angle_to_goal << " dist_to_goal: " << dist_to_goal);
        result.outcome = navigation_interface::Controller::Outcome::COMPLETE;
        last_update_ = ros::SteadyTime::now();
        return result;
    }

    //
    // Find target
    //
    // path_index is the index closest to where the robot is currently
    // target_state is the pose the robot is aiming for
    navigation_interface::KinodynamicState target_state;
    path_index_ = findClosest(trajectory_->states, robot_state.pose);

    const double dist_to_closest = dist(robot_state.pose, trajectory_->states[path_index_].pose, 0.1);
    if (dist_to_closest > tracking_error_)
    {
        ROS_WARN_STREAM("Tracking error. Distance to trajectory: " << dist_to_closest << " > " << tracking_error_);
        result.outcome = navigation_interface::Controller::Outcome::FAILED;
        last_update_ = ros::SteadyTime::now();
        return result;
    }

    target_state = lookAhead(trajectory_->states, robot_state.pose, robot_state.velocity, path_index_, look_ahead_time_,
                             interpolation_steps_);

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

        const CollisionCheck cc = robotInCollision(local_grid, map_robot_pose, map_goal_pose, robot_footprint_, 1.f);
        min_distance_to_collision = cc.min_distance_to_collision;
        if (debug_viz_)
            footprint_pub_.publish(cc.marker);

        if (cc.in_collision)
        {
            ROS_WARN_STREAM("Robot is in collision!");
            result.outcome = navigation_interface::Controller::Outcome::FAILED;
            last_update_ = ros::SteadyTime::now();
            return result;
        }
    }

    if (debug_viz_)
        target_state_pub_.publish(buildMarker(robot_state, target_state));

    ROS_ASSERT(robot_state.pose.linear().allFinite());
    ROS_ASSERT(robot_state.pose.translation().allFinite());

    //
    // PD control (PID when close to goal)
    //
    const Eigen::Vector2d target_vec_wrt_robot =
        robot_state.pose.linear().inverse() * (target_state.pose.translation() - robot_state.pose.translation());
    const Eigen::Vector3d control_error = {
        target_vec_wrt_robot[0], target_vec_wrt_robot[1],
        Eigen::Rotation2Dd(robot_state.pose.linear().inverse() * target_state.pose.linear()).smallestAngle()};
    ROS_ASSERT(control_error.allFinite());

    Eigen::Vector3d target_velocity;

    const Eigen::Vector3d control_dot_ = (control_error - control_error_) / dt;
    control_error_ = control_error;
    ROS_ASSERT(control_error_.allFinite());

    const bool final_pid_control = path_index_ == trajectory_->states.size() - 1;
    if (final_pid_control)
    {
        control_integral_ += control_error;

        // Integrator wind-up prevention clamp
        for (unsigned int i = 0; i < 3; ++i)
        {
            control_integral_[i] =
                std::min(std::max(-control_integral_max_[i], control_integral_[i]), control_integral_max_[i]);
        }

        target_velocity = p_gain_.cwiseProduct(control_error) + i_gain_.cwiseProduct(control_integral_) +
                          d_gain_.cwiseProduct(control_dot_);
    }
    else
    {
        target_velocity = p_gain_.cwiseProduct(control_error) + d_gain_.cwiseProduct(control_dot_);
    }

    ROS_ASSERT_MSG(target_velocity.allFinite(), "%f %f %f", target_velocity[0], target_velocity[1], target_velocity[2]);

    //
    // Max acceleration check
    //
    Eigen::Vector3d vel_command = target_velocity;

    const Eigen::Vector2d translation_dv(target_velocity[0] - prev_cmd_vel[0], target_velocity[1] - prev_cmd_vel[1]);
    const double rotation_dv = target_velocity[2] - prev_cmd_vel[2];

    // We limit the X and Y accelerations together to preserve the direction
    const double translation_accel = translation_dv.norm() / dt;
    if (translation_accel > max_translation_accel_)
    {
        // ROS_INFO_STREAM("Excessive translation acceleration: " << translation_accel << " > " <<
        // max_translation_accel_);
        vel_command[0] = prev_cmd_vel[0] + translation_dv[0] * (max_translation_accel_ / translation_accel);
        vel_command[1] = prev_cmd_vel[1] + translation_dv[1] * (max_translation_accel_ / translation_accel);
    }

    const double rotation_accel = std::abs(rotation_dv) / dt;
    if (rotation_accel > max_rotation_accel_)
    {
        // ROS_INFO_STREAM("Excessive rotation acceleration: " << rotation_accel << " > " << max_rotation_accel_);
        vel_command[2] = prev_cmd_vel[2] + rotation_dv * (max_rotation_accel_ / rotation_accel);
    }

    ROS_ASSERT(vel_command.allFinite());

    prev_cmd_vel = vel_command;

    //
    // Max velocity check
    //

    // Limit max velocity based on distance to nearest obstacle
    const double max_avoid_distance = 0.6;
    const double d = std::min(max_avoid_distance, min_distance_to_collision);
    const double velocity_scale = std::max(0.20, d / max_avoid_distance);

    Eigen::Vector3d augmented_max_vel = {std::min(max_velocity_[0], max_velocity_[0] * velocity_scale),
                                         std::min(max_velocity_[1], max_velocity_[1] * velocity_scale),
                                         std::min(max_velocity_[2], max_velocity_[2] * velocity_scale)};

    {
        double vel_factor = 1.0;
        for (long i = 0; i < 3; ++i)
        {
            if (std::abs(vel_command[i]) > augmented_max_vel[i])
            {
                vel_factor = std::max(vel_factor, std::abs(vel_command[i] / augmented_max_vel[i]));
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
void PurePursuitController::onInitialize(const YAML::Node& parameters)
{
    look_ahead_time_ = parameters["look_ahead_time"].as<double>(look_ahead_time_);

    max_velocity_[0] = parameters["max_velocity_x"].as<double>(max_velocity_[0]);
    max_velocity_[1] = parameters["max_velocity_y"].as<double>(max_velocity_[1]);
    max_velocity_[2] = parameters["max_velocity_w"].as<double>(max_velocity_[2]);

    max_translation_accel_ = parameters["max_translation_accel"].as<double>(max_translation_accel_);
    max_rotation_accel_ = parameters["max_rotation_accel"].as<double>(max_rotation_accel_);

    xy_goal_tolerance_ = parameters["xy_goal_tolerance"].as<double>(xy_goal_tolerance_);
    yaw_goal_tolerance_ = parameters["yaw_goal_tolerance"].as<double>(yaw_goal_tolerance_);

    p_gain_[0] = parameters["p_gain_x"].as<double>(p_gain_[0]);
    p_gain_[1] = parameters["p_gain_y"].as<double>(p_gain_[1]);
    p_gain_[2] = parameters["p_gain_w"].as<double>(p_gain_[2]);

    i_gain_[0] = parameters["i_gain_x"].as<double>(i_gain_[0]);
    i_gain_[1] = parameters["i_gain_y"].as<double>(i_gain_[1]);
    i_gain_[2] = parameters["i_gain_w"].as<double>(i_gain_[2]);

    d_gain_[0] = parameters["d_gain_x"].as<double>(d_gain_[0]);
    d_gain_[1] = parameters["d_gain_y"].as<double>(d_gain_[1]);
    d_gain_[2] = parameters["d_gain_w"].as<double>(d_gain_[2]);

    control_integral_max_[0] = parameters["control_integral_max_x"].as<double>(control_integral_max_[0]);
    control_integral_max_[1] = parameters["control_integral_max_y"].as<double>(control_integral_max_[1]);
    control_integral_max_[2] = parameters["control_integral_max_w"].as<double>(control_integral_max_[2]);

    robot_footprint_ = navigation_interface::get_point_list(
        parameters, "footprint",
        {{0.480, 0.000}, {0.380, -0.395}, {-0.380, -0.395}, {-0.480, 0.000}, {-0.380, 0.395}, {0.380, 0.395}});

    debug_viz_ = parameters["debug_viz"].as<bool>(debug_viz_);
    if (debug_viz_)
    {
        ros::NodeHandle nh("~");
        target_state_pub_ = nh.advertise<visualization_msgs::Marker>("target_state", 100);
        footprint_pub_ = nh.advertise<visualization_msgs::Marker>("footprint", 100);
    }
}

// cppcheck-suppress unusedFunction
void PurePursuitController::onMapDataChanged()
{
}
}  // namespace pure_pursuit_controller
