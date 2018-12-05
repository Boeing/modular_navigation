#include <rrt_local_planner/rrt_local_planner.h>

#include <algorithm>

#include <costmap_2d/cost_values.h>

#include <tf2/utils.h>

#include <pluginlib/class_list_macros.h>

#include <ompl/geometric/PathSimplifier.h>

PLUGINLIB_DECLARE_CLASS(rrt_local_planner, RRTLocalPlanner, rrt_local_planner::RRTLocalPlanner, nav_core::BaseLocalPlanner)

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
    double sarg = -2 * (q.x()*q.z() - q.w()*q.y()) / (sqx + sqy + sqz + sqw); /* normalization added from urdfom_headers */

    if (sarg <= -0.99999) {
        yaw   = -2 * atan2(q.y(), q.x());
    } else if (sarg >= 0.99999) {
        yaw   = 2 * atan2(q.y(), q.x());
    } else {
        yaw   = atan2(2 * (q.x()*q.y() + q.w()*q.z()), sqw + sqx - sqy - sqz);
    }
    return yaw;
};

LocalTrajectory convert(const ompl::geometric::PathGeometric& path)
{
    LocalTrajectory trajectory;

    for (unsigned int i=0; i<path.getStateCount(); ++i)
    {
        const ompl::base::State* state = path.getState(i);
        const auto* compound_state = state->as<ompl::base::CompoundStateSpace::StateType>();
        const auto* se2state = compound_state->as<ompl::base::SE2StateSpace::StateType>(0);

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

RRTLocalPlanner::RRTLocalPlanner()
    : costmap_ros_(nullptr)
{
}

RRTLocalPlanner::~RRTLocalPlanner()
{

}

bool RRTLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    std::lock_guard<std::mutex> lock(control_mutex_);

    if (!control_data_)
    {
        ROS_INFO("No control data!");
        return true;
    }

    return true;

//    ROS_INFO("executing...");

    //
    // Get the current robot pose
    //
    geometry_msgs::PoseStamped robot_pose;
    costmap_ros_->getRobotPose(robot_pose);

    //
    // Find the point where currently executing
    //
    std::size_t min_i = 0;
    double euc_distance = std::numeric_limits<double>::max();
    for (std::size_t i=control_data_->execution_index; i<control_data_->trajectory.states.size(); ++i)
    {
        const double dx = control_data_->trajectory.states[i].pose.position.x - robot_pose.pose.position.x;
        const double dy = control_data_->trajectory.states[i].pose.position.y - robot_pose.pose.position.y;
        const double d = std::sqrt(dx*dx + dy*dy);

        if (d < 0.01)
            continue;

        if (d < euc_distance)
        {
            min_i = i;
            euc_distance = d;
        }
    }
    control_data_->execution_index = min_i;

    //
    // Termination criteria
    //
    if (min_i >= control_data_->trajectory.states.size() - 1 && euc_distance < 0.02)
    {
        ROS_INFO("Trajectory complete!");
        control_data_->execution_complete = true;
        return true;
    }

//    ROS_INFO_STREAM("Tracking point: " << min_i << " of " << control_data_->trajectory.states.size());

    //
    // Check the local plan is collision free
    //
    for (std::size_t i=0; i<control_data_->trajectory.states.size(); ++i)
    {
        const geometry_msgs::Pose& p = control_data_->trajectory.states[i].pose;

        unsigned int cell_x, cell_y;
        unsigned char cost;
        if (!costmap_ros_->getCostmap()->worldToMap(p.position.x, p.position.y, cell_x, cell_y))
        {
            cost = 1;
        }
        else
        {
            cost = costmap_ros_->getCostmap()->getCost(cell_x, cell_y);
        }

        //
        // Execute evasive action if in collision
        //
        if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
            ROS_WARN_STREAM("Collision at point: " << i << " of " << control_data_->trajectory.states.size());
            return true;
        }
    }

    const auto start_p = control_data_->trajectory.states[min_i-1];
    const auto end_p = control_data_->trajectory.states[min_i];

    const Eigen::Vector3d x = Eigen::Vector3d(robot_pose.pose.position.x, robot_pose.pose.position.y, 0);
    const Eigen::Vector3d start = Eigen::Vector3d(start_p.pose.position.x, start_p.pose.position.y, 0);
    const Eigen::Vector3d end = Eigen::Vector3d(end_p.pose.position.x, end_p.pose.position.y, 0);

    const Eigen::Quaterniond x_qt(robot_pose.pose.orientation.w, robot_pose.pose.orientation.x, robot_pose.pose.orientation.y, robot_pose.pose.orientation.z);
    const Eigen::Quaterniond end_qt(end_p.pose.orientation.w, end_p.pose.orientation.x, end_p.pose.orientation.y, end_p.pose.orientation.z);

    Eigen::ParametrizedLine<double, 3> line = Eigen::ParametrizedLine<double, 3>::Through(start, end);

    const Eigen::Vector3d point_along_segment = line.projection(x);
    const Eigen::Vector3d error_vector = point_along_segment - x;

    const double control_p_gain = 2;
    const double speed = 0.1;
    const Eigen::Vector3d target_velocity = speed * line.direction();

//    ROS_INFO_STREAM("target_velocity: " << target_velocity.transpose());

    const Eigen::Vector3d segment_vel = x_qt.inverse() * (target_velocity + error_vector * control_p_gain);

    const double e_theta = getYaw(x_qt.inverse() * end_qt);

    ROS_INFO_STREAM("e_x: " << error_vector[0]);
    ROS_INFO_STREAM("e_y: " << error_vector[1]);
    ROS_INFO_STREAM("e_w: " << e_theta);

    if (error_vector.norm() > 0.1)
    {
        ROS_WARN_STREAM("Tracking error exceeds limit!");
        return false;
    }

    // compute PID
    cmd_vel.linear.x = segment_vel[0];
    cmd_vel.linear.y = segment_vel[1];
    cmd_vel.angular.z = e_theta * control_p_gain;

//    ROS_INFO_STREAM("v_x: " << cmd_vel.linear.x);
//    ROS_INFO_STREAM("v_y: " << cmd_vel.linear.y);
//    ROS_INFO_STREAM("v_w: " << cmd_vel.angular.z);

    return true;
}

bool RRTLocalPlanner::isGoalReached()
{
    std::lock_guard<std::mutex> lock(global_mutex_);
    return !bool(global_path_);
}

bool RRTLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    std::lock_guard<std::mutex> lock(global_mutex_);

    if (global_path_)
        return true;

    global_path_ = std::unique_ptr<GlobalPath>(new GlobalPath());
    global_path_->path = plan;
    global_path_->execution_index = 0;

    // Remove the last planned trajectory (forcing a re-plan)
    std::lock_guard<std::mutex> t_lock(trajectory_mutex_);
    trajectory_result_.reset(nullptr);

    return true;
}

void RRTLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    tf_buffer_ = tf;
    costmap_ros_ = costmap_ros;

    ros::NodeHandle nh;
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 1, &RRTLocalPlanner::odomCallback, this);

    rrt_viz_.reset(new rviz_visual_tools::RvizVisualTools(costmap_ros_->getGlobalFrameID(), name + "/rrt"));
    trajectory_viz_.reset(new rviz_visual_tools::RvizVisualTools(costmap_ros_->getGlobalFrameID(), name + "/trajectory"));

    //
    // Setup OMPL
    //
    se2_space_ = ompl::base::StateSpacePtr(new ompl::base::SE2StateSpace());
    velocity_space_ = ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(3));
    space_ = ompl::base::StateSpacePtr(se2_space_ + velocity_space_);
    cspace_ = ompl::control::ControlSpacePtr(new ompl::control::RealVectorControlSpace(space_, 3));

    ompl::base::RealVectorBounds velocity_bounds(3);
    velocity_bounds.setLow(0, -0.1);  // velocity x control (acceleration)
    velocity_bounds.setHigh(0, 0.2);
    velocity_bounds.setLow(1, -0.1);  // velocity y control (acceleration)
    velocity_bounds.setHigh(1, 0.2);
    velocity_bounds.setLow(2, -0.1);  // theta control (acceleration)
    velocity_bounds.setHigh(2, 0.1);
    velocity_space_->as<ompl::base::RealVectorStateSpace>()->setBounds(velocity_bounds);

    ompl::base::RealVectorBounds cbounds(3);
    cbounds.setLow(0, -0.1);  // velocity x control (acceleration)
    cbounds.setHigh(0, 0.2);
    cbounds.setLow(1, -0.1);  // velocity y control (acceleration)
    cbounds.setHigh(1, 0.2);
    cbounds.setLow(2, -0.1);  // theta control (acceleration)
    cbounds.setHigh(2, 0.1);
    cspace_->as<ompl::control::RealVectorControlSpace>()->setBounds(cbounds);

    // Create space information:
    si_ = ompl::control::SpaceInformationPtr(new ompl::control::SpaceInformation(space_, cspace_));
    si_->setStatePropagator(boost::bind(&RRTLocalPlanner::propagate, this, _1, _2, _3, _4));
    si_->setStateValidityChecker(boost::bind(&RRTLocalPlanner::isStateValid, this, si_.get(), _1));

    // Optimize criteria
    ompl::base::OptimizationObjectivePtr cost_objective(new CostMapObjective(*this, si_));
    ompl::base::OptimizationObjectivePtr length_objective(new ompl::base::PathLengthOptimizationObjective(si_));
    ompl::base::MultiOptimizationObjective* objective = new ompl::base::MultiOptimizationObjective(si_);
    objective->addObjective(cost_objective, 1.0);
    //objective->addObjective(length_objective, 1.0);
    objective_ = ompl::base::OptimizationObjectivePtr(objective);

    planning_thread_ = std::thread(&RRTLocalPlanner::trajectoryPlanningThread, this);
}


TrajectoryPlanResult RRTLocalPlanner::planLocalTrajectory(const Eigen::Isometry2d& start, const Eigen::Isometry2d& goal, const double threshold)
{
    ROS_INFO("planLocalTrajectory...");

    ROS_INFO_STREAM("start: " << start.translation().transpose());
    ROS_INFO_STREAM("goal: " << goal.translation().transpose());
    ROS_INFO_STREAM("threshold: " << threshold);

    const double search_window = std::min(costmap_ros_->getCostmap()->getSizeInCellsX() * costmap_ros_->getCostmap()->getResolution(),
                                          costmap_ros_->getCostmap()->getSizeInCellsY() * costmap_ros_->getCostmap()->getResolution()) / 2.0;

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
    ompl::base::ScopedState<> ompl_start(space_);
    ompl_start[0] = start.translation().x();
    ompl_start[1] = start.translation().y();
    ompl_start[2] = Eigen::Rotation2D<double>(start.rotation()).angle();
    ompl_start[3] = 0;  // Speed X
    ompl_start[4] = 0;  // Speed Y
    ompl_start[5] = 0;  // Speed W

    ompl::base::ScopedState<> ompl_goal(space_);
    ompl_goal[0] = goal.translation().x();
    ompl_goal[1] = goal.translation().y();
    ompl_goal[2] = Eigen::Rotation2D<double>(goal.rotation()).angle();
    ompl_goal[3] = 0;  // Speed X
    ompl_goal[4] = 0;  // Speed Y
    ompl_goal[5] = 0;  // Speed W

    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si_));
    pdef->setOptimizationObjective(objective_);

    // A copy of the start and goal is made
    pdef->setStartAndGoalStates(ompl_start, ompl_goal, threshold);

    ROS_INFO("Problem defined, running planner");
    auto rrt = new ompl::geometric::RRTstar(si_);
    rrt->setGoalBias(0.1);
    rrt->setRange(0.05);

    ompl::base::PlannerPtr planner(rrt);
    planner->setProblemDefinition(pdef);
    planner->setup();
    ompl::base::PlannerStatus solved = planner->solve(0.1);

    TrajectoryPlanResult result;

    result.pd = std::unique_ptr<ompl::base::PlannerData>(new ompl::base::PlannerData(si_));
    planner->getPlannerData(*result.pd);

    visualisePlannerData(*result.pd);

    if (solved)
    {
        ROS_INFO("Planning successful");

        ompl::base::PathPtr path_ptr = pdef->getSolutionPath();
        ompl::geometric::PathGeometric result_path = static_cast<ompl::geometric::PathGeometric&>(*path_ptr);

//        ompl::geometric::PathSimplifier simplifier(si_);
//        simplifier.smoothBSpline(result_path, 5, 0.01);

        result.trajectory = std::unique_ptr<ompl::geometric::PathGeometric>(new ompl::geometric::PathGeometric(result_path));
    }
    else
    {
        ROS_INFO("Planning failed!");
    }

    return result;
}

unsigned char RRTLocalPlanner::stateCost(const ompl::base::State* state)
{
    const ompl::base::CompoundStateSpace::StateType* compound_state = state->as<ompl::base::CompoundStateSpace::StateType>();
    const ompl::base::SE2StateSpace::StateType* se2state = compound_state->as<ompl::base::SE2StateSpace::StateType>(0);

    const double x = se2state->getX();
    const double y = se2state->getY();

    unsigned int cell_x, cell_y;
    unsigned char cost;
    if (!costmap_ros_->getCostmap()->worldToMap(x, y, cell_x, cell_y))
    {
        // probably at the edge of the costmap - this value should be recovered soon
        cost = 1;
    }
    else
    {
        // get cost for this cell
        cost = costmap_ros_->getCostmap()->getCost(cell_x, cell_y);
    }

    return cost;
}

bool RRTLocalPlanner::isStateValid(const ompl::control::SpaceInformation* si, const ompl::base::State* state)
{
    if (!si->satisfiesBounds(state))
    {
        return false;
    }

    // Get the cost of the footprint at the current location:
    unsigned char cost = stateCost(state);

    // Too high cost:
    if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
        return false;
    }

    // TODO check local costmap bounds
/*
    const ompl::base::CompoundStateSpace::StateType* compound_state = state->as<ompl::base::CompoundStateSpace::StateType>();
    const ompl::base::RealVectorStateSpace::StateType* v_state = compound_state->as<ompl::base::RealVectorStateSpace::StateType>(1);

    const double velocity_x = (*v_state)[0];
    const double velocity_y = (*v_state)[1];

    const double euc_vel = std::sqrt(velocity_x*velocity_x + velocity_y+velocity_y);

    // Velocity stopping distance check...
    const double distance_to_collision = costToDistance(cost);

    // Must stop within 1 second
    if (distance_to_collision < euc_vel)
    {
        ROS_WARN_STREAM("Failed stopping condition check");
        return false;
    }
*/
    return true;
}

double RRTLocalPlanner::costToDistance(const unsigned char cost) const
{
    if (cost >= costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
        return 0.0;
    }
    else
    {
        const double c = (cost != costmap_2d::FREE_SPACE && cost != costmap_2d::NO_INFORMATION) ? static_cast<double>(cost) : 1.0;
        const double factor = static_cast<double>(c) / (costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1);
        return -log(factor) / costmap_weight_;
    }
}

void RRTLocalPlanner::propagate(const ompl::base::State* start, const ompl::control::Control* control, const double duration, ompl::base::State* result)
{
    const ompl::base::CompoundStateSpace::StateType* compound_state = start->as<ompl::base::CompoundStateSpace::StateType>();
    const ompl::base::SE2StateSpace::StateType* se2state = compound_state->as<ompl::base::SE2StateSpace::StateType>(0);
    const ompl::base::RealVectorStateSpace::StateType* v_state = compound_state->as<ompl::base::RealVectorStateSpace::StateType>(1);

    // Get the values:
    const double x = se2state->getX();
    const double y = se2state->getY();
    const double theta = se2state->getYaw();

    const double velocity_x = (*v_state)[0];
    const double velocity_y = (*v_state)[1];
    const double velocity_w = (*v_state)[2];

    // scale acceleration base on costmap
    const unsigned char cost = stateCost(start);
    const double distance_to_collision = costToDistance(cost);
    const double acc_factor = std::min(distance_to_collision*distance_to_collision, 1.0);

    const double* ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double acc_x = ctrl[0]; //  * acc_factor;
    const double acc_y = ctrl[1]; //  * acc_factor;
    const double acc_w = ctrl[2]; //  * acc_factor;

    const double x_n = x + velocity_x * duration;
    const double y_n = y + velocity_y * duration;
    const double theta_n = theta + velocity_w * duration;

    const double velocity_x_n = velocity_x + acc_x*duration;
    const double velocity_y_n = velocity_y + acc_y*duration;
    const double velocity_w_n = velocity_w + acc_w*duration;

    // Store new state in result
    ompl::base::CompoundStateSpace::StateType* res_compound_state = result->as<ompl::base::CompoundStateSpace::StateType>();
    ompl::base::SE2StateSpace::StateType* res_se2state = res_compound_state->as<ompl::base::SE2StateSpace::StateType>(0);
    ompl::base::RealVectorStateSpace::StateType* res_v_state = res_compound_state->as<ompl::base::RealVectorStateSpace::StateType>(1);

    // Set values:
    res_se2state->setX(x_n);
    res_se2state->setY(y_n);
    res_se2state->setYaw(theta_n);
    (*res_v_state)[0] = velocity_x_n;
    (*res_v_state)[1] = velocity_y_n;
    (*res_v_state)[2] = velocity_w_n;

    // Make sure angle is (-pi,pi]:
    const ompl::base::SO2StateSpace* SO2 = se2_space_->as<ompl::base::SE2StateSpace>()->as<ompl::base::SO2StateSpace>(1);
    ompl::base::SO2StateSpace::StateType* so2 = res_se2state->as<ompl::base::SO2StateSpace::StateType>(1);
    SO2->enforceBounds(so2);
}

void RRTLocalPlanner::visualisePlannerData(const ompl::base::PlannerData& pd)
{
    rrt_viz_->deleteAllMarkers();

    const rviz_visual_tools::colors color = rviz_visual_tools::BLUE;
    const rviz_visual_tools::scales scale = rviz_visual_tools::XXSMALL;

    auto get_pose = [](const ompl::base::PlannerData& pd, unsigned int vertex_id)
    {
        const ompl::base::PlannerDataVertex& v = pd.getVertex(vertex_id);
        const ompl::base::State* state = v.getState();

        const auto* compound_state = state->as<ompl::base::CompoundStateSpace::StateType>();
        const auto* se2state = compound_state->as<ompl::base::SE2StateSpace::StateType>(0);

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

    for (unsigned int i=0; i<pd.numVertices(); ++i)
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

    for (unsigned int i=0; i<path.getStateCount(); ++i)
    {
        const ompl::base::State* state = path.getState(i);

        const auto* compound_state = state->as<ompl::base::CompoundStateSpace::StateType>();
        const auto* se2state = compound_state->as<ompl::base::SE2StateSpace::StateType>(0);

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

    for (unsigned int i=0; i<poses.size() - 1; ++i)
    {
        trajectory_viz_->publishAxis(poses[i], scale);
        trajectory_viz_->publishLine(poses[i].translation(), poses[i+1].translation(), color, scale);
    }
    if (!poses.empty())
        rrt_viz_->publishAxis(poses.back(), scale);

    trajectory_viz_->trigger();
}

void RRTLocalPlanner::updateTrajectory(const TrajectoryPlanResult& result)
{
    std::lock_guard<std::mutex> t_lock(trajectory_mutex_);
    trajectory_result_ = std::unique_ptr<TrajectoryPlanResult>(new TrajectoryPlanResult(result));
    visualisePathGeometric(*trajectory_result_->trajectory);

    // TODO transform into odom frame
    std::lock_guard<std::mutex> c_lock(control_mutex_);
    control_data_ = std::unique_ptr<ControlData>(new ControlData());
    control_data_->trajectory = convert(*result.trajectory);
    control_data_->execution_index = 0;
    control_data_->execution_complete = false;
}

double RRTLocalPlanner::remainingTrajectoryCost()
{
    std::lock_guard<std::mutex> t_lock(trajectory_mutex_);
    std::lock_guard<std::mutex> c_lock(control_mutex_);

    if (!trajectory_result_ || !control_data_)
        return 0.0;

    ompl::geometric::PathGeometric trajectory = *trajectory_result_->trajectory;
    trajectory.keepAfter(trajectory.getState(static_cast<unsigned int>(control_data_->execution_index)));

    return trajectory.cost(objective_).value();
}

void RRTLocalPlanner::trajectoryPlanningThread()
{
    while (ros::ok())
    {
        const std::string robot_frame = costmap_ros_->getBaseFrameID();
        const std::string local_frame = costmap_ros_->getGlobalFrameID();

        //
        // Get the current robot pose
        //
        Eigen::Isometry3d robot_pose;
        try
        {
            const geometry_msgs::TransformStamped tr = tf_buffer_->lookupTransform(local_frame, robot_frame, ros::Time::now(), ros::Duration(0.1));
            const Eigen::Quaterniond qt = Eigen::Quaterniond(tr.transform.rotation.w, tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z);
            const Eigen::Translation3d p = Eigen::Translation3d(tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z);
            robot_pose = p * qt;
        }
        catch (const tf2::TransformException& e)
        {
            ROS_WARN_STREAM("Failed to get robot position: " << e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }

        //
        // Get the global path goal
        //
        bool setpoint_same = false;
        bool last_waypoint = false;
        Eigen::Isometry3d goal;
        {
            std::lock_guard<std::mutex> lock(global_mutex_);

            if (!global_path_)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            // Check if empty
            if (global_path_->path.empty())
            {
                global_path_.reset(nullptr);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            // Get the global costmap to local costmap transform
            Eigen::Isometry3d global_to_local;
            try
            {
                const std::string global_frame = global_path_->path.front().header.frame_id;
                const geometry_msgs::TransformStamped tr = tf_buffer_->lookupTransform(global_frame, local_frame, ros::Time::now(), ros::Duration(0.1));
                const Eigen::Quaterniond qt = Eigen::Quaterniond(tr.transform.rotation.w, tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z);
                const Eigen::Translation3d p = Eigen::Translation3d(tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z);
                global_to_local = p * qt;
            }
            catch (const tf2::TransformException& e)
            {
                ROS_WARN_STREAM("Failed to get global->local costmap transform: " << e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }

            // Iterate the execution index
            const std::size_t remaining_waypoints = global_path_->path.size() - global_path_->execution_index;
            std::vector<double> distances(remaining_waypoints);
            for (std::size_t i=global_path_->execution_index; i<global_path_->path.size(); ++i)
            {
                const geometry_msgs::Pose _pose = global_path_->path[i].pose;
                const Eigen::Quaterniond qt = Eigen::Quaterniond(_pose.orientation.w, _pose.orientation.x, _pose.orientation.y, _pose.orientation.z);
                const Eigen::Translation3d p = Eigen::Translation3d(_pose.position.x, _pose.position.y, _pose.position.z);
                const Eigen::Isometry3d pose = p * qt;
                const Eigen::Isometry3d transformed_pose = global_to_local * pose;
                const Eigen::Vector3d d = transformed_pose.translation() - robot_pose.translation();
                distances[i] = d.norm();
            }
            std::size_t min_i = global_path_->execution_index + static_cast<std::size_t>(std::distance(distances.begin(), std::min_element(distances.begin(), distances.end())));

            // If not at end of path lookahead to the next waypoint preemptively
            const double lookahead_distance = 0.5;
            if (min_i < global_path_->path.size() - 1 && distances[min_i] < lookahead_distance)
                min_i++;

            setpoint_same = bool(global_path_->execution_index == min_i);
            global_path_->execution_index = min_i;
            last_waypoint = bool(global_path_->execution_index >= global_path_->path.size() - 1);

            // Check for completion
            if (last_waypoint)
            {
                std::lock_guard<std::mutex> c_lock(control_mutex_);
                if (control_data_ && control_data_->execution_complete)
                {
                    global_path_.reset(nullptr);
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    continue;
                }
            }

            ROS_INFO_STREAM("Targeting " << global_path_->execution_index << " of " << global_path_->path.size());

            // Get goal in the local costmap frame
            const geometry_msgs::Pose _pose = global_path_->path[global_path_->execution_index].pose;
            const Eigen::Quaterniond qt = Eigen::Quaterniond(_pose.orientation.w, _pose.orientation.x, _pose.orientation.y, _pose.orientation.z);
            const Eigen::Translation3d p = Eigen::Translation3d(_pose.position.x, _pose.position.y, _pose.position.z);
            const Eigen::Isometry3d pose = p * qt;
            goal = global_to_local * pose;
        }

        //
        // Run the trajectory planner
        //
        const Eigen::Isometry2d stard_2d = Eigen::Translation2d(robot_pose.translation().x(), robot_pose.translation().y())
                * Eigen::Rotation2D<double>(getYaw(Eigen::Quaterniond(robot_pose.rotation())));
        const Eigen::Isometry2d goal_2d = Eigen::Translation2d(goal.translation().x(), goal.translation().y())
                * Eigen::Rotation2D<double>(getYaw(Eigen::Quaterniond(goal.rotation())));
        const double threshold = last_waypoint ? 0.02 : 0.1;
        TrajectoryPlanResult result = planLocalTrajectory(stard_2d, goal_2d, 0.01);

        if (result.success)
        {
            // Compare with current trajectory
            if (setpoint_same)
            {
                const double current_cost = remainingTrajectoryCost();
                const double new_cost = result.trajectory->cost(objective_).value();  // TODO this has segfaulted

                ROS_INFO_STREAM("current_cost: " << current_cost << " new_cost: " << new_cost);
                if (new_cost < current_cost)
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
        else
        {
            ROS_WARN_STREAM("Failed to get trajectory");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

void RRTLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // lock Callback while reading data from topic
    std::lock_guard<std::mutex> lock(odom_mutex_);

    // get odometry and write it to member variable (we assume that the odometry is published in the frame of the base)
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
}

}  // end namespace global_planner
