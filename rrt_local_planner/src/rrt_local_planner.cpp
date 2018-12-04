#include <rrt_local_planner/rrt_local_planner.h>

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

double calcYaw(const geometry_msgs::Pose& pose)
{
    tf2::Transform pose_tf;
    tf2::convert(pose, pose_tf);
    return tf2::getYaw(pose_tf.getRotation());
}

void get_xy_theta_v(const ob::State* s, double& x, double& y, double& theta, double& v)
{
    const ob::CompoundStateSpace::StateType* compound_state = s->as<ob::CompoundStateSpace::StateType>();
    const ob::SE2StateSpace::StateType* se2state = compound_state->as<ob::SE2StateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType* v_state = compound_state->as<ob::RealVectorStateSpace::StateType>(1);

    // Get the values:
    x = se2state->getX();
    y = se2state->getY();
    theta = se2state->getYaw();
    v = (*v_state)[0];
}

geometry_msgs::PoseStamped getNextGoal(
            const tf2_ros::Buffer& tf_buffer,
            const std::vector<geometry_msgs::PoseStamped>& global_plan,
            const costmap_2d::Costmap2DROS& costmap,
            const std::string& global_frame)
{
    assert (!global_plan.empty());

    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

    const geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(
        global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id);
    tf2::Transform transform_;
    tf2::convert(transform.transform, transform_);

//    const geometry_msgs::TransformStamped robot_pose = tf_buffer.lookupTransform(
//        plan_pose.header.frame_id, costmap.getBaseFrameID(), ros::Time(), ros::Duration(1.0));

//    double dist_threshold = 2.0;
//    unsigned int i = 0;
//    double sq_dist_threshold = dist_threshold * dist_threshold;
//    double sq_dist = DBL_MAX;

//    while (i < global_plan.size() && sq_dist > sq_dist_threshold)
//    {
//        double x_diff = robot_pose.transform.translation.x - global_plan[i].pose.position.x;
//        double y_diff = robot_pose.transform.translation.y - global_plan[i].pose.position.y;
//        sq_dist = x_diff * x_diff + y_diff * y_diff;

//        // not yet in reach - get next frame
//        if (sq_dist > sq_dist_threshold)
//        {
//            ++i;
//        }
//        else
//        {
//            break;
//        }
//    }

    const geometry_msgs::PoseStamped& pose = global_plan.back();

    tf2::Transform pose_;
    tf2::convert(pose.pose, pose_);

    const tf2::Transform transformed_pose_ = transform_ * pose_;

    geometry_msgs::PoseStamped transformed_pose;
    transformed_pose.header.stamp = transform.header.stamp;
    transformed_pose.header.frame_id = transform.header.frame_id;

    transformed_pose.pose.position.x = transformed_pose_.getOrigin().x();
    transformed_pose.pose.position.y = transformed_pose_.getOrigin().y();
    transformed_pose.pose.position.z = transformed_pose_.getOrigin().z();

    transformed_pose.pose.orientation.w = transformed_pose_.getRotation().w();
    transformed_pose.pose.orientation.x = transformed_pose_.getRotation().x();
    transformed_pose.pose.orientation.y = transformed_pose_.getRotation().y();
    transformed_pose.pose.orientation.z = transformed_pose_.getRotation().z();

    return transformed_pose;
}

}

RRTLocalPlanner::RRTLocalPlanner()
    : costmap_ros_(nullptr),
      trajectory_complete_(false),
      local_trajectory_(nullptr),
      se2_space_(new ob::SE2StateSpace()),
      velocity_space_(new ob::RealVectorStateSpace(3)),
      space_(se2_space_ + velocity_space_)
{
}

RRTLocalPlanner::~RRTLocalPlanner()
{

}

bool RRTLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    // If no local trajectory then run RRT planner
    bool has_trajectory = false;
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        has_trajectory = bool(local_trajectory_);
    }

    if (!has_trajectory)
    {
        ROS_INFO("Replanning local trajectory");
        planLocalTrajectory();
        return true;
    }

    std::lock_guard<std::mutex> lock(trajectory_mutex_);

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
    for (std::size_t i=local_trajectory_->execution_index; i<local_trajectory_->states.size(); ++i)
    {
        const double dx = local_trajectory_->states[i].pose.position.x - robot_pose.pose.position.x;
        const double dy = local_trajectory_->states[i].pose.position.y - robot_pose.pose.position.y;
        const double d = std::sqrt(dx*dx + dy*dy);

        if (d < 0.02)
            continue;

        if (d < euc_distance)
        {
            min_i = i;
            euc_distance = d;
        }
    }
    local_trajectory_->execution_index = min_i;

    //
    // Termination criteria
    //
    if (min_i == local_trajectory_->states.size() - 1 && euc_distance < 0.02)
    {
        ROS_INFO("Trajectory complete!");
        trajectory_complete_ = true;
    }

    ROS_INFO_STREAM("Tracking point: " << min_i << " of " << local_trajectory_->states.size());

    //
    // Check the local plan is collision free
    //
    for (std::size_t i=0; i<local_trajectory_->states.size(); ++i)
    {
        const geometry_msgs::Pose& p = local_trajectory_->states[i].pose;

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
            ROS_WARN_STREAM("Collision at point: " << i << " of " << local_trajectory_->states.size());
            local_trajectory_.reset(nullptr);
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = 0;
            return true;
        }
    }

    const auto start_p = local_trajectory_->states[min_i-1];
    const auto end_p = local_trajectory_->states[min_i];

    const Eigen::Vector3d x = Eigen::Vector3d(robot_pose.pose.position.x, robot_pose.pose.position.y, 0);
    const Eigen::Vector3d start = Eigen::Vector3d(start_p.pose.position.x, start_p.pose.position.y, 0);
    const Eigen::Vector3d end = Eigen::Vector3d(end_p.pose.position.x, end_p.pose.position.y, 0);

    const Eigen::Quaterniond x_qt(robot_pose.pose.orientation.w, robot_pose.pose.orientation.x, robot_pose.pose.orientation.y, robot_pose.pose.orientation.z);
    const Eigen::Quaterniond end_qt(end_p.pose.orientation.w, end_p.pose.orientation.x, end_p.pose.orientation.y, end_p.pose.orientation.z);

    Eigen::ParametrizedLine<double, 3> line = Eigen::ParametrizedLine<double, 3>::Through(start, end);

    const Eigen::Vector3d point_along_segment = line.projection(x);
    const Eigen::Vector3d error_vector = point_along_segment - x;

    const double control_p_gain = 1;
    const double speed = 0.02;
    const Eigen::Vector3d target_velocity = speed * line.direction();

    ROS_INFO_STREAM("target_velocity: " << target_velocity.transpose());

    if (error_vector.norm() > 0.04)
    {
        ROS_WARN_STREAM("Tracking error exceeds limit!");
        return false;
    }

    const Eigen::Vector3d segment_vel = x_qt.inverse() * (target_velocity + error_vector * control_p_gain);

    const double e_theta = getYaw(x_qt.inverse() * end_qt);

    ROS_INFO_STREAM("e_x: " << error_vector[0]);
    ROS_INFO_STREAM("e_y: " << error_vector[1]);
    ROS_INFO_STREAM("e_w: " << e_theta);

    // compute PID
    cmd_vel.linear.x = segment_vel[0];
    cmd_vel.linear.y = segment_vel[1];
    cmd_vel.angular.z = e_theta * 0.1;

    ROS_INFO_STREAM("v_x: " << cmd_vel.linear.x);
    ROS_INFO_STREAM("v_y: " << cmd_vel.linear.y);
    ROS_INFO_STREAM("v_w: " << cmd_vel.angular.z);

    return true;
}

bool RRTLocalPlanner::isGoalReached()
{
    return trajectory_complete_;
}

bool RRTLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    std::lock_guard<std::mutex> lock(global_plan_mutex_);

    global_plan_ = plan;

    std::lock_guard<std::mutex> t_lock(trajectory_mutex_);
    local_trajectory_.reset(nullptr);

    return true;
}

void RRTLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    tf_buffer_ = tf;
    costmap_ros_ = costmap_ros;

    ROS_INFO("inialising RRTLocalPlanner");

    ros::NodeHandle nh;
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 1, &RRTLocalPlanner::odomCallback, this);

    rrt_viz_.reset(new rviz_visual_tools::RvizVisualTools(costmap_ros_->getGlobalFrameID(), name + "/rrt"));
    trajectory_viz_.reset(new rviz_visual_tools::RvizVisualTools(costmap_ros_->getGlobalFrameID(), name + "/trajectory"));
}


bool RRTLocalPlanner::planLocalTrajectory()
{
    geometry_msgs::PoseStamped goal_pose;

    {
        std::lock_guard<std::mutex> lock(global_plan_mutex_);
        goal_pose = getNextGoal(*tf_buffer_, global_plan_, *costmap_ros_, costmap_ros_->getGlobalFrameID());
    }

    geometry_msgs::PoseStamped robot_pose;
    costmap_ros_->getRobotPose(robot_pose);

    ROS_INFO_STREAM("start pose: " << robot_pose.pose.position.x << " " << robot_pose.pose.position.y << " frame: " << robot_pose.header.frame_id);
    ROS_INFO_STREAM("goal pose: " << goal_pose.pose.position.x << " " << goal_pose.pose.position.y << " frame: " << goal_pose.header.frame_id);

    // assert the goal is not too far away
    const double search_window = std::min(costmap_ros_->getCostmap()->getSizeInCellsX(), costmap_ros_->getCostmap()->getSizeInCellsY());
    const double dx = goal_pose.pose.position.x - robot_pose.pose.position.x;
    const double dy = goal_pose.pose.position.y - robot_pose.pose.position.y;
    const double goal_distance = std::sqrt(dx*dx + dy*dy);
    assert (goal_distance < search_window);

    ROS_INFO("Thinking about OMPL path..");
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, robot_pose.pose.position.x - search_window);
    bounds.setHigh(0, robot_pose.pose.position.x + search_window);
    bounds.setLow(1, robot_pose.pose.position.y - search_window);
    bounds.setHigh(1, robot_pose.pose.position.y + search_window);
    se2_space_->as<ob::SE2StateSpace>()->setBounds(bounds);

    ob::RealVectorBounds velocity_bounds(3);
    velocity_bounds.setLow(0, -0.1);  // velocity x control (acceleration)
    velocity_bounds.setHigh(0, 0.2);
    velocity_bounds.setLow(1, -0.1);  // velocity y control (acceleration)
    velocity_bounds.setHigh(1, 0.2);
    velocity_bounds.setLow(2, -0.1);  // theta control (acceleration)
    velocity_bounds.setHigh(2, 0.1);
    velocity_space_->as<ob::RealVectorStateSpace>()->setBounds(velocity_bounds);

    oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space_, 3));
    ob::RealVectorBounds cbounds(3);
    cbounds.setLow(0, -0.1);  // velocity x control (acceleration)
    cbounds.setHigh(0, 0.2);
    cbounds.setLow(1, -0.1);  // velocity y control (acceleration)
    cbounds.setHigh(1, 0.2);
    cbounds.setLow(2, -0.1);  // theta control (acceleration)
    cbounds.setHigh(2, 0.1);
    cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    // Create space information:
    oc::SpaceInformationPtr si(new oc::SpaceInformation(space_, cspace));
    si->setStatePropagator(boost::bind(&RRTLocalPlanner::propagate, this, _1, _2, _3, _4));
    si->setStateValidityChecker(boost::bind(&RRTLocalPlanner::isStateValid, this, si.get(), _1));

    // Define problem:
    ob::ScopedState<> ompl_start(space_);
    ompl_start[0] = robot_pose.pose.position.x;
    ompl_start[1] = robot_pose.pose.position.y;
    ompl_start[2] = calcYaw(robot_pose.pose);
    ompl_start[3] = 0;  // Speed X
    ompl_start[4] = 0;  // Speed Y
    ompl_start[5] = 0;  // Speed W

    ob::ScopedState<> ompl_goal(space_);
    ompl_goal[0] = goal_pose.pose.position.x;
    ompl_goal[1] = goal_pose.pose.position.y;
    ompl_goal[2] = calcYaw(goal_pose.pose);
    ompl_goal[3] = 0;  // Speed X
    ompl_goal[4] = 0;  // Speed Y
    ompl_goal[5] = 0;  // Speed W

    // Optimize criteria:
    ob::OptimizationObjectivePtr cost_objective(new CostMapObjective(*this, si));
    ob::OptimizationObjectivePtr length_objective(new ob::PathLengthOptimizationObjective(si));
    // ob::OptimizationObjectivePtr objective(new CostMapWorkObjective(*this, si));

    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->setStartAndGoalStates(ompl_start, ompl_goal, 0.1);
    pdef->setOptimizationObjective(cost_objective + length_objective);

    ROS_INFO("Problem defined, running planner");
    auto rrt = new og::RRTstar(si);
    rrt->setGoalBias(0.2);
    rrt->setRange(0.04);

    ob::PlannerPtr planner(rrt);
    planner->setProblemDefinition(pdef);
    planner->setup();
    ob::PlannerStatus solved = planner->solve(0.1);

    // Visualise states
    {
        ompl::base::PlannerData pd(si);
        planner->getPlannerData(pd);

        rrt_viz_->deleteAllMarkers();

        const rviz_visual_tools::colors color = rviz_visual_tools::BLUE;
        const rviz_visual_tools::scales scale = rviz_visual_tools::XXSMALL;

        auto get_pose = [](const ompl::base::PlannerData& pd, unsigned int vertex_id)
        {
            const ompl::base::PlannerDataVertex& v = pd.getVertex(vertex_id);
            const ompl::base::State* state = v.getState();

            double x, y, theta, velocity;
            get_xy_theta_v(state, x, y, theta, velocity);

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

    LocalTrajectory* local_trajectory = new LocalTrajectory;
    if (solved)
    {
        ROS_INFO("Successfull!");

        ob::PathPtr result_path1 = pdef->getSolutionPath();

        // Cast path into geometric path:
        og::PathGeometric result_path = static_cast<og::PathGeometric&>(*result_path1);

        ompl::geometric::PathSimplifier simplifier(si);

        simplifier.smoothBSpline(result_path, 5, 0.01);
        //simplifier.simplify(result_path, 0.1);

        local_trajectory->states.push_back({robot_pose.pose, geometry_msgs::Twist()});

        // Conversion loop from states to messages:
        std::vector<ob::State*>& result_states = result_path.getStates();
        for (std::vector<ob::State*>::iterator it = result_states.begin(); it != result_states.end(); ++it)
        {
            double x, y, theta, velocity;
            get_xy_theta_v(*it, x, y, theta, velocity);

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

            ROS_INFO_STREAM("velocity: " << velocity);

            local_trajectory->states.push_back({pose, twist});
        }

        local_trajectory->states.push_back({goal_pose.pose, geometry_msgs::Twist()});

        // Visualise trajectory
        {
            trajectory_viz_->deleteAllMarkers();

            const rviz_visual_tools::colors color = rviz_visual_tools::RED;
            const rviz_visual_tools::scales scale = rviz_visual_tools::SMALL;

            for (unsigned int i=0; i<local_trajectory->states.size() - 1; ++i)
            {
                trajectory_viz_->publishAxis(local_trajectory->states[i].pose, scale);
                trajectory_viz_->publishLine(local_trajectory->states[i].pose.position, local_trajectory->states[i+1].pose.position, color, scale);
            }
            rrt_viz_->publishAxis(local_trajectory->states.back().pose, scale);

            trajectory_viz_->trigger();
        }

        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            local_trajectory_.reset(local_trajectory);
        }

        return true;
    }
    else
    {
        ROS_ERROR("Failed to determine plan");

        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            local_trajectory_.reset(nullptr);
        }

        return false;
    }
}

unsigned char RRTLocalPlanner::calc_cost(const ob::State* state)
{
    double x, y, theta, velocity;
    get_xy_theta_v(state, x, y, theta, velocity);

    // Get the cost of the footprint at the current location:
    //double cost = costmap_model_->footprintCost(x, y, theta, costmap_ros_->getRobotFootprint());

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

// Calculate the cost of a motion:
double motion_cost(const ob::State* s1, const ob::State* s2)
{
    // int nd = validSegmentCount(s1, s2);
    // TODO: interpolate?
    double cst = 0;

    // cst =

    return cst;
}

// Check the current state:
bool RRTLocalPlanner::isStateValid(const oc::SpaceInformation* si, const ob::State* state)
{
    if (!si->satisfiesBounds(state))
    {
        return false;
    }

    // Get the cost of the footprint at the current location:
    unsigned char cost = calc_cost(state);

    // Too high cost:
    if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
        return false;
    }

    const ob::CompoundStateSpace::StateType* compound_state = state->as<ob::CompoundStateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType* v_state = compound_state->as<ob::RealVectorStateSpace::StateType>(1);

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

void RRTLocalPlanner::propagate(const ob::State* start, const oc::Control* control, const double duration, ob::State* result)
{
    const ob::CompoundStateSpace::StateType* compound_state = start->as<ob::CompoundStateSpace::StateType>();
    const ob::SE2StateSpace::StateType* se2state = compound_state->as<ob::SE2StateSpace::StateType>(0);
    const ob::RealVectorStateSpace::StateType* v_state = compound_state->as<ob::RealVectorStateSpace::StateType>(1);

    // Get the values:
    const double x = se2state->getX();
    const double y = se2state->getY();
    const double theta = se2state->getYaw();

    const double velocity_x = (*v_state)[0];
    const double velocity_y = (*v_state)[1];
    const double velocity_w = (*v_state)[2];

    // scale acceleration base on costmap
    const unsigned char cost = calc_cost(start);
    const double distance_to_collision = costToDistance(cost);
    const double acc_factor = std::min(distance_to_collision*distance_to_collision, 1.0);

    const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double acc_x = ctrl[0] * acc_factor;
    const double acc_y = ctrl[1] * acc_factor;
    const double acc_w = ctrl[2] * acc_factor;

    const double x_n = x + velocity_x * duration;
    const double y_n = y + velocity_y * duration;
    const double theta_n = theta + velocity_w * duration;

    const double velocity_x_n = velocity_x + acc_x*duration;
    const double velocity_y_n = velocity_y + acc_y*duration;
    const double velocity_w_n = velocity_w + acc_w*duration;

    // Store new state in result
    ob::CompoundStateSpace::StateType* res_compound_state = result->as<ob::CompoundStateSpace::StateType>();
    ob::SE2StateSpace::StateType* res_se2state = res_compound_state->as<ob::SE2StateSpace::StateType>(0);
    ob::RealVectorStateSpace::StateType* res_v_state = res_compound_state->as<ob::RealVectorStateSpace::StateType>(1);

    // Set values:
    res_se2state->setX(x_n);
    res_se2state->setY(y_n);
    res_se2state->setYaw(theta_n);
    (*res_v_state)[0] = velocity_x_n;
    (*res_v_state)[1] = velocity_y_n;
    (*res_v_state)[2] = velocity_w_n;

    // Make sure angle is (-pi,pi]:
    const ob::SO2StateSpace* SO2 = se2_space_->as<ob::SE2StateSpace>()->as<ob::SO2StateSpace>(1);
    ob::SO2StateSpace::StateType* so2 = res_se2state->as<ob::SO2StateSpace::StateType>(1);
    SO2->enforceBounds(so2);
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
