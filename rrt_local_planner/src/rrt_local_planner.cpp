#include <rrt_local_planner/rrt_local_planner.h>

#include <algorithm>

#include <costmap_2d/cost_values.h>

#include <tf2/utils.h>

#include <pluginlib/class_list_macros.h>

#include <ompl/geometric/PathSimplifier.h>

PLUGINLIB_DECLARE_CLASS(rrt_local_planner, RRTLocalPlanner, rrt_local_planner::RRTLocalPlanner,
                        nav_core::BaseLocalPlanner)

namespace rrt_local_planner
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

bool isDiff(const Eigen::Isometry3d& t1, const Eigen::Isometry3d& t2)
{
    const bool trans = (t1.translation() - t2.translation()).norm() > 1e-4;
    const bool rot = Eigen::AngleAxisd((t2.inverse() * t1).rotation()).angle() > 1e-4;
    return trans || rot;
}

LocalTrajectory convert(const ompl::geometric::PathGeometric& path)
{
    LocalTrajectory trajectory;

    for (unsigned int i = 0; i < path.getStateCount(); ++i)
    {
        const ompl::base::State* state = path.getState(i);
        const auto* se2state = state->as<ompl::base::SE2StateSpace::StateType>();

        const double x = se2state->getX();
        const double y = se2state->getY();
        const double theta = se2state->getYaw();

        tf2::Quaternion qt;
        qt.setRPY(0, 0, theta);

        geometry_msgs::Pose pose;
        geometry_msgs::Twist twist;
        pose.position.x = x;
        pose.position.y = y;
        pose.orientation.w = qt.w();
        pose.orientation.x = qt.x();
        pose.orientation.y = qt.y();
        pose.orientation.z = qt.z();

        trajectory.states.push_back({pose, twist});
    }

    return trajectory;
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

RRTLocalPlanner::RRTLocalPlanner() : tf_buffer_(nullptr), costmap_ros_(nullptr)
{
}

RRTLocalPlanner::~RRTLocalPlanner()
{
}

nav_core::Control RRTLocalPlanner::computeControl(const ros::SteadyTime& steady_time, const ros::Time& ros_time,
                                                  const nav_msgs::Odometry& odom)
{
    nav_core::Control control;

    bool has_global_path = true;
    //    {
    //        std::lock_guard<std::mutex> lock(global_mutex_);
    //        has_global_path = bool(global_path_);
    //    }
    std::lock_guard<std::mutex> lock(control_mutex_);
    const bool has_control = bool(control_data_);

    if (!has_global_path)
    {
        control.state = nav_core::ControlState::COMPLETE;
        goal_x_pid_.reset();
        goal_y_pid_.reset();
        tracking_x_pid_.reset();
        tracking_y_pid_.reset();
        rotation_pid_.reset();
        return control;
    }

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

    //
    // Update tracking state
    //
    const ros::SteadyTime last_time_step = control_data_->last_time_step;
    control_data_->last_time_step = steady_time;

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
    for (std::size_t i = control_data_->execution_index; i < control_data_->trajectory.states.size(); ++i)
    {
        const geometry_msgs::Pose _pose = control_data_->trajectory.states[i].pose;
        const Eigen::Vector3d p = Eigen::Vector3d(_pose.position.x, _pose.position.y, _pose.position.z);
        const double linear_delta = (p - robot_pose.translation()).norm();

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
    const bool last_waypoint = min_i >= control_data_->trajectory.states.size() - 1;
    ROS_INFO_STREAM("Tracking point: " << (min_i + 1) << " of " << control_data_->trajectory.states.size());

    //
    // Determine the tracking error
    //
    const auto start_p = control_data_->trajectory.states.at(min_i - 1);
    const auto end_p = control_data_->trajectory.states.at(min_i);
    const Eigen::Quaterniond start_qt(start_p.pose.orientation.w, start_p.pose.orientation.x,
                                      start_p.pose.orientation.y, start_p.pose.orientation.z);
    const Eigen::Quaterniond end_qt(end_p.pose.orientation.w, end_p.pose.orientation.x, end_p.pose.orientation.y,
                                    end_p.pose.orientation.z);
    const Eigen::Vector3d start = Eigen::Vector3d(start_p.pose.position.x, start_p.pose.position.y, 0);
    const Eigen::Vector3d end = Eigen::Vector3d(end_p.pose.position.x, end_p.pose.position.y, 0);
    const Eigen::Vector3d segment = end - start;
    const Eigen::Vector3d segment_wrtb = robot_pose.rotation().inverse() * segment;
    const Eigen::ParametrizedLine<double, 3> line = Eigen::ParametrizedLine<double, 3>::Through(start, end);
    const Eigen::Vector3d point_along_segment = line.projection(robot_pose.translation());
    const Eigen::Vector3d linear_error = point_along_segment - robot_pose.translation();
    const Eigen::Vector3d goal_error = end - robot_pose.translation();
    const Eigen::Vector3d linear_error_wrtb = robot_pose.rotation().inverse() * linear_error;
    const Eigen::Vector3d goal_error_wrtb = robot_pose.rotation().inverse() * goal_error;

    //
    // Determine the rotational tracking error
    //
    const double segment_rot = getYaw(start_qt.inverse() * end_qt);
    const double angular_error = getYaw(Eigen::Quaterniond(robot_pose.rotation()).inverse() * end_qt);

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
        control_data_->execution_complete = true;
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
    if (angular_error > max_angular_tracking_error_)
    {
        ROS_WARN_STREAM("Exceeded maximum angular tracking error: " << angular_error << " > "
                                                                    << max_angular_tracking_error_);
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
    for (std::size_t i = control_data_->execution_index; i < control_data_->trajectory.states.size(); ++i)
    {
        const geometry_msgs::Pose& p = control_data_->trajectory.states[i].pose;
        const geometry_msgs::Pose& prev_p = control_data_->trajectory.states[i - 1].pose;

        const double dx = prev_p.position.x - p.position.x;
        const double dy = prev_p.position.y - p.position.y;
        accum_distance += std::sqrt(dx * dx + dy * dy);

        const unsigned char cost = getCost(*costmap_ros_->getCostmap(), p.position.x, p.position.y);

        //
        // Execute evasive action if in collision
        //
        if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
            ROS_WARN_STREAM("Collision at point: " << i << " of " << control_data_->trajectory.states.size());
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
        getCost(*costmap_ros_->getCostmap(), end_p.pose.position.x, end_p.pose.position.y);
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

        Eigen::Vector3d goal_dir_wrtb = robot_pose.rotation().inverse() * (end - robot_pose.translation());
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

    if (std::abs(d_vw) > max_acceleration_w_)
    {
        ROS_WARN_STREAM("Limiting maximum acceleration in W: " << d_vw << " > " << max_acceleration_w_);
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

bool RRTLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    std::lock_guard<std::mutex> lock(global_mutex_);
    std::lock_guard<std::mutex> t_lock(trajectory_mutex_);

    global_path_ = std::unique_ptr<GlobalPath>(new GlobalPath());
    global_path_->path = plan;
    global_path_->execution_index = 0;

    // Remove the last planned trajectory (forcing a re-plan)
    trajectory_result_.reset(nullptr);

    return true;
}

bool RRTLocalPlanner::clearPlan()
{
    {
        std::lock_guard<std::mutex> lock(global_mutex_);
        global_path_.reset(nullptr);
    }

    {
        std::lock_guard<std::mutex> t_lock(trajectory_mutex_);
        trajectory_result_.reset(nullptr);
    }

    {
        std::lock_guard<std::mutex> c_lock(control_mutex_);
        control_data_.reset(nullptr);
    }

    return true;
}

void RRTLocalPlanner::initialize(const std::string& name, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                 const std::shared_ptr<costmap_2d::Costmap2DROS>& local_costmap)
{
    tf_buffer_ = tf_buffer.get();
    costmap_ros_ = local_costmap.get();

    // Assert the costmap is in odom
    const std::string local_frame = costmap_ros_->getGlobalFrameID();
    if (local_frame != "odom")
    {
        throw std::runtime_error("Local costmap must be in odom frame for robust trajectory control");
    }

    rrt_viz_.reset(new rviz_visual_tools::RvizVisualTools(costmap_ros_->getGlobalFrameID(), name + "/rrt"));
    trajectory_viz_.reset(
        new rviz_visual_tools::RvizVisualTools(costmap_ros_->getGlobalFrameID(), name + "/trajectory"));

    //
    // Setup OMPL
    //
    se2_space_ = ompl::base::StateSpacePtr(new ompl::base::SE2StateSpace());
    se2_space_->setLongestValidSegmentFraction(0.005);
    si_ = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(se2_space_));
    si_->setStateValidityChecker(std::make_shared<ValidityChecker>(si_, costmap_ros_->getCostmap(), costmap_weight_));

    // Optimize criteria
    ompl::base::OptimizationObjectivePtr cost_objective(new CostMapObjective(si_, costmap_ros_->getCostmap()));
    ompl::base::OptimizationObjectivePtr length_objective(new ompl::base::PathLengthOptimizationObjective(si_));
    ompl::base::MultiOptimizationObjective* objective = new ompl::base::MultiOptimizationObjective(si_);
    // Highest cost is 252 so you would have 252 per m
    objective->addObjective(cost_objective, 1.0);
    // Highest cost would be 1 per m
    objective->addObjective(length_objective, 50.0);
    objective_ = ompl::base::OptimizationObjectivePtr(objective);

    planning_thread_ = std::thread(&RRTLocalPlanner::trajectoryPlanningThread, this);
}


TrajectoryPlanResult RRTLocalPlanner::planLocalTrajectory(const Eigen::Isometry2d& start, const Eigen::Isometry2d& goal,
                                                          const double threshold)
{
    const double search_window =
        std::min(costmap_ros_->getCostmap()->getSizeInCellsX() * costmap_ros_->getCostmap()->getResolution(),
                 costmap_ros_->getCostmap()->getSizeInCellsY() * costmap_ros_->getCostmap()->getResolution()) /
        2.0;

    TrajectoryPlanResult result;
    result.start = start;
    result.goal = goal;

    //
    // Update XY sample bounds
    //
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, start.translation().x() - search_window);
    bounds.setHigh(0, start.translation().x() + search_window);
    bounds.setLow(1, start.translation().y() - search_window);
    bounds.setHigh(1, start.translation().y() + search_window);
    se2_space_->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

    // Define problem
    ompl::base::ScopedState<> ompl_start(se2_space_);
    ompl_start[0] = start.translation().x();
    ompl_start[1] = start.translation().y();
    ompl_start[2] = Eigen::Rotation2D<double>(start.rotation()).angle();

    ompl::base::ScopedState<> ompl_goal(se2_space_);
    ompl_goal[0] = goal.translation().x();
    ompl_goal[1] = goal.translation().y();
    ompl_goal[2] = Eigen::Rotation2D<double>(goal.rotation()).angle();

    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si_));
    pdef->setOptimizationObjective(objective_);

    // A copy of the start and goal is made
    pdef->setStartAndGoalStates(ompl_start, ompl_goal, threshold);

    ROS_INFO("Problem defined, running planner");
    auto rrt = new ompl::geometric::RRTstar(si_);
    rrt->setGoalBias(0.1);
    rrt->setRange(0.1);

    result.planner = ompl::base::PlannerPtr(rrt);
    result.planner->setProblemDefinition(pdef);
    result.planner->setup();

    ompl::base::PlannerStatus solved;

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*costmap_ros_->getCostmap()->getMutex());

    solved = result.planner->solve(0.1);

    result.pd = std::unique_ptr<ompl::base::PlannerData>(new ompl::base::PlannerData(si_));
    result.planner->getPlannerData(*result.pd);

    visualisePlannerData(*result.pd);

    if (ompl::base::PlannerStatus::StatusType(solved) == ompl::base::PlannerStatus::EXACT_SOLUTION)
    {
        result.success = true;
        result.cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()).value();
        result.length = pdef->getSolutionPath()->length();

        ROS_INFO_STREAM(result.planner->getName() << " found a solution of length " << result.length
                                                  << " with an optimization objective value of " << result.cost);

        ompl::base::PathPtr path_ptr = pdef->getSolutionPath();
        ompl::geometric::PathGeometric result_path = static_cast<ompl::geometric::PathGeometric&>(*path_ptr);

        ompl::geometric::PathSimplifier simplifier(si_);
        // simplifier.smoothBSpline(result_path, 5, 0.005);
        simplifier.simplify(result_path, 0.1);
        result_path.interpolate();

        result.cost = result_path.cost(pdef->getOptimizationObjective()).value();

        result.trajectory =
            std::unique_ptr<ompl::geometric::PathGeometric>(new ompl::geometric::PathGeometric(result_path));
    }
    else
    {
        result.success = false;
        result.cost = 0;
        result.length = 0;
    }

    return result;
}

void RRTLocalPlanner::visualisePlannerData(const ompl::base::PlannerData& pd)
{
    rrt_viz_->deleteAllMarkers();

    const rviz_visual_tools::colors color = rviz_visual_tools::BLUE;
    const rviz_visual_tools::scales scale = rviz_visual_tools::XXSMALL;

    auto get_pose = [](const ompl::base::PlannerData& pd, unsigned int vertex_id) {
        const ompl::base::PlannerDataVertex& v = pd.getVertex(vertex_id);
        const ompl::base::State* state = v.getState();
        const auto* se2state = state->as<ompl::base::SE2StateSpace::StateType>();

        const double x = se2state->getX();
        const double y = se2state->getY();
        const double theta = se2state->getYaw();

        tf2::Quaternion qt;
        qt.setRPY(0, 0, theta);

        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.orientation.w = qt.w();
        pose.orientation.x = qt.x();
        pose.orientation.y = qt.y();
        pose.orientation.z = qt.z();

        return pose;
    };

    for (unsigned int i = 0; i < pd.numVertices(); ++i)
    {
        const geometry_msgs::Pose pose = get_pose(pd, i);

        rrt_viz_->publishAxis(pose, scale);

        // Draw edges
        {
            std::vector<unsigned int> edge_list;
            pd.getEdges(i, edge_list);
            for (unsigned int e : edge_list)
            {
                const geometry_msgs::Pose e_pose = get_pose(pd, e);
                rrt_viz_->publishLine(pose.position, e_pose.position, color, scale);
            }
        }
    }

    rrt_viz_->trigger();
}

void RRTLocalPlanner::visualisePathGeometric(const ompl::geometric::PathGeometric& path)
{
    std::vector<Eigen::Isometry3d> poses;

    for (unsigned int i = 0; i < path.getStateCount(); ++i)
    {
        const ompl::base::State* state = path.getState(i);
        const auto* se2state = state->as<ompl::base::SE2StateSpace::StateType>();

        const double x = se2state->getX();
        const double y = se2state->getY();
        const double theta = se2state->getYaw();

        const Eigen::Quaterniond qt = Eigen::Quaterniond(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
        const Eigen::Translation3d tr = Eigen::Translation3d(x, y, 0);
        const Eigen::Isometry3d pose = tr * qt;

        poses.push_back(pose);
    }

    trajectory_viz_->deleteAllMarkers();

    const rviz_visual_tools::colors color = rviz_visual_tools::RED;
    const rviz_visual_tools::scales scale = rviz_visual_tools::SMALL;

    for (unsigned int i = 0; i < poses.size() - 1; ++i)
    {
        trajectory_viz_->publishAxis(poses[i], scale);
        trajectory_viz_->publishLine(poses[i].translation(), poses[i + 1].translation(), color, scale);
    }
    if (!poses.empty())
        trajectory_viz_->publishAxis(poses.back(), scale);

    trajectory_viz_->trigger();
}

void RRTLocalPlanner::updateTrajectory(const TrajectoryPlanResult& result)
{
    std::lock_guard<std::mutex> t_lock(trajectory_mutex_);
    std::lock_guard<std::mutex> c_lock(control_mutex_);

    trajectory_result_ = std::unique_ptr<TrajectoryPlanResult>(new TrajectoryPlanResult(result));
    visualisePathGeometric(*trajectory_result_->trajectory);

    control_data_ = std::unique_ptr<ControlData>(new ControlData());
    control_data_->trajectory = convert(*result.trajectory);
    control_data_->execution_index = 1;
    control_data_->execution_complete = false;
    tracking_x_pid_.reset();
    tracking_y_pid_.reset();
}

double RRTLocalPlanner::getRemainingTrajectoryCost()
{
    // TODO double mutex deadlocks are here
    ROS_INFO("getRemainingTrajectoryCost costmap_ros_");
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*costmap_ros_->getCostmap()->getMutex());
    ROS_INFO("getRemainingTrajectoryCost trajectory_mutex_");
    std::lock_guard<std::mutex> t_lock(trajectory_mutex_);
    ROS_INFO("getRemainingTrajectoryCost control_mutex_");
    std::lock_guard<std::mutex> c_lock(control_mutex_);

    if (!trajectory_result_ || !control_data_)
        return std::numeric_limits<double>::max();

    ompl::geometric::PathGeometric trajectory = *trajectory_result_->trajectory;
    const unsigned int i = std::max(1U, static_cast<unsigned int>(control_data_->execution_index)) - 1;
    trajectory.keepAfter(trajectory.getState(i));

    ROS_INFO("getRemainingTrajectoryCost DONE!");
    if (!trajectory.check())
        return std::numeric_limits<double>::max();

    return trajectory.cost(objective_).value();
}

PlannerState RRTLocalPlanner::getPlannerState()
{
    std::lock_guard<std::mutex> lock(global_mutex_);

    PlannerState planner_state;
    planner_state.path_exists = false;
    planner_state.path_waypoint_changed = false;
    planner_state.path_last_waypoint = false;
    planner_state.global_to_local_changed = true;

    if (!global_path_)
    {
        ROS_INFO("No global path");
        return planner_state;
    }

    // Check if empty
    if (global_path_->path.empty())
    {
        global_path_.reset(nullptr);
        return planner_state;
    }

    planner_state.path_exists = true;

    const std::string robot_frame = costmap_ros_->getBaseFrameID();
    const std::string local_frame = costmap_ros_->getGlobalFrameID();
    const ros::Time now = ros::Time::now();

    //
    // Get the current robot pose
    //
    try
    {
        const geometry_msgs::TransformStamped tr =
            tf_buffer_->lookupTransform(local_frame, robot_frame, now, ros::Duration(0.1));
        const Eigen::Quaterniond qt = Eigen::Quaterniond(tr.transform.rotation.w, tr.transform.rotation.x,
                                                         tr.transform.rotation.y, tr.transform.rotation.z);
        const Eigen::Translation3d p =
            Eigen::Translation3d(tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z);
        planner_state.robot_pose = p * qt;
    }
    catch (const tf2::TransformException& e)
    {
        ROS_WARN_STREAM("Failed to get robot position: " << e.what());
        return planner_state;
    }

    // Get the global costmap to local costmap transform
    try
    {
        const std::string global_frame = global_path_->path.front().header.frame_id;
        const geometry_msgs::TransformStamped tr =
            tf_buffer_->lookupTransform(global_frame, local_frame, now, ros::Duration(0.1));
        const Eigen::Quaterniond qt = Eigen::Quaterniond(tr.transform.rotation.w, tr.transform.rotation.x,
                                                         tr.transform.rotation.y, tr.transform.rotation.z);
        const Eigen::Translation3d p =
            Eigen::Translation3d(tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z);
        planner_state.global_to_local = p * qt;
    }
    catch (const tf2::TransformException& e)
    {
        ROS_WARN_STREAM("Failed to get global->local costmap transform: " << e.what());
        return planner_state;
    }

    // Check if the map->odom transform has changed
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        if (trajectory_result_)
        {
            planner_state.global_to_local_changed =
                isDiff(planner_state.global_to_local, trajectory_result_->global_to_local);
        }
    }

    // Iterate the execution index
    const std::size_t remaining_waypoints = global_path_->path.size() - global_path_->execution_index;
    std::vector<double> distances(remaining_waypoints);
    for (std::size_t i = 0; i < remaining_waypoints; ++i)
    {
        const std::size_t offset = global_path_->execution_index + i;
        const geometry_msgs::Pose _pose = global_path_->path[offset].pose;
        const Eigen::Quaterniond qt =
            Eigen::Quaterniond(_pose.orientation.w, _pose.orientation.x, _pose.orientation.y, _pose.orientation.z);
        const Eigen::Translation3d p = Eigen::Translation3d(_pose.position.x, _pose.position.y, _pose.position.z);
        const Eigen::Isometry3d pose = p * qt;
        const Eigen::Isometry3d transformed_pose = planner_state.global_to_local * pose;
        const Eigen::Vector3d d = transformed_pose.translation() - planner_state.robot_pose.translation();
        distances.at(i) = d.norm();
    }
    std::size_t min_i = static_cast<std::size_t>(
        std::distance(distances.begin(), std::min_element(distances.begin(), distances.end())));

    // If not at end of path lookahead to the next waypoint preemptively
    const double lookahead_distance = 0.5;
    if (min_i < remaining_waypoints - 1 && distances[min_i] < lookahead_distance)
    {
        min_i++;
    }

    const std::size_t target_waypoint = global_path_->execution_index + min_i;

    planner_state.path_waypoint_changed = !bool(global_path_->execution_index == target_waypoint);
    global_path_->execution_index = target_waypoint;
    planner_state.path_last_waypoint = bool(global_path_->execution_index >= global_path_->path.size() - 1);

    // Check for completion
    if (planner_state.path_last_waypoint)
    {
        std::lock_guard<std::mutex> t_lock(trajectory_mutex_);
        std::lock_guard<std::mutex> c_lock(control_mutex_);
        if (control_data_ && control_data_->execution_complete)
        {
            ROS_INFO("Global path complete");
            global_path_.reset(nullptr);
            trajectory_result_.reset(nullptr);
            control_data_.reset(nullptr);
            planner_state.path_exists = false;
            return planner_state;
        }
    }

    ROS_INFO_STREAM("Targeting waypoint: " << global_path_->execution_index + 1 << " of " << global_path_->path.size());

    // Get goal in the local costmap frame
    {
        const geometry_msgs::Pose _pose = global_path_->path[global_path_->execution_index].pose;
        const Eigen::Quaterniond qt =
            Eigen::Quaterniond(_pose.orientation.w, _pose.orientation.x, _pose.orientation.y, _pose.orientation.z);
        const Eigen::Translation3d p = Eigen::Translation3d(_pose.position.x, _pose.position.y, _pose.position.z);
        const Eigen::Isometry3d pose = p * qt;
        planner_state.goal = planner_state.global_to_local * pose;
    }

    planner_state.stard_2d =
        Eigen::Translation2d(planner_state.robot_pose.translation().x(), planner_state.robot_pose.translation().y()) *
        Eigen::Rotation2D<double>(getYaw(Eigen::Quaterniond(planner_state.robot_pose.rotation())));
    planner_state.goal_2d =
        Eigen::Translation2d(planner_state.goal.translation().x(), planner_state.goal.translation().y()) *
        Eigen::Rotation2D<double>(getYaw(Eigen::Quaterniond(planner_state.goal.rotation())));

    return planner_state;
}

void RRTLocalPlanner::trajectoryPlanningThread()
{
    ros::Rate rate(10);
    while (ros::ok())
    {
        //
        // Get the global path goal
        //
        const PlannerState planner_state = getPlannerState();

        if (planner_state.path_exists)
        {
            //
            // Run the trajectory planner
            //
            const double threshold = planner_state.path_last_waypoint ? 0.001 : 0.2;
            TrajectoryPlanResult result = planLocalTrajectory(planner_state.stard_2d, planner_state.goal_2d, threshold);
            result.global_to_local = planner_state.global_to_local;

            //
            // Get the new global path goal (after time taken to plan)
            //
            const PlannerState new_planner_state = getPlannerState();

            if (result.success && new_planner_state.path_exists)
            {
                // Compare with current trajectory
                if (!planner_state.path_waypoint_changed && !planner_state.global_to_local_changed)
                {
                    const double current_cost = getRemainingTrajectoryCost();
                    ROS_INFO_STREAM("current_cost: " << current_cost << " new_cost: " << result.cost);
                    // at least a 10% cost saving
                    if (result.cost * 1.10 < current_cost)
                    {
                        ROS_INFO("Updating trajectory");
                        updateTrajectory(result);
                    }
                }
                else
                {
                    updateTrajectory(result);
                }
            }
        }

        rate.sleep();
    }
}

}  // end namespace global_planner
