#include <omni_rrt_planner/omni_rrt_planner.h>

#include <algorithm>

#include <costmap_2d/cost_values.h>

#include <tf2/utils.h>

#include <pluginlib/class_list_macros.h>

#include <ompl/geometric/PathSimplifier.h>

PLUGINLIB_DECLARE_CLASS(omni_rrt_planner, OmniRRTPlanner, omni_rrt_planner::OmniRRTPlanner, nav_core::BaseGlobalPlanner)

namespace omni_rrt_planner
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

std::vector<geometry_msgs::PoseStamped> convert(const ompl::geometric::PathGeometric& path, const std::string& frame_id)
{
    std::vector<geometry_msgs::PoseStamped> trajectory;

    for (unsigned int i = 0; i < path.getStateCount(); ++i)
    {
        const ompl::base::State* state = path.getState(i);
        const auto* se2state = state->as<ompl::base::SE2StateSpace::StateType>();

        const double x = se2state->getX();
        const double y = se2state->getY();
        const double theta = se2state->getYaw();

        tf2::Quaternion qt;
        qt.setRPY(0, 0, theta);

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation.w = qt.w();
        pose.pose.orientation.x = qt.x();
        pose.pose.orientation.y = qt.y();
        pose.pose.orientation.z = qt.z();

        trajectory.push_back(pose);
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

OmniRRTPlanner::OmniRRTPlanner()
{
}

OmniRRTPlanner::~OmniRRTPlanner()
{
}

void OmniRRTPlanner::initialize(const std::string& name, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                const std::shared_ptr<costmap_2d::Costmap2DROS>& global_costmap,
                                const std::shared_ptr<costmap_2d::Costmap2DROS>& local_costmap)
{
    tf_buffer_ = tf_buffer;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    rrt_viz_.reset(new rviz_visual_tools::RvizVisualTools(global_costmap_->getGlobalFrameID(), name + "/rrt"));
    trajectory_viz_.reset(
        new rviz_visual_tools::RvizVisualTools(global_costmap_->getGlobalFrameID(), name + "/trajectory"));

    //
    // Setup OMPL
    //
    se2_space_ = ompl::base::StateSpacePtr(new ompl::base::SE2StateSpace());
    se2_space_->setLongestValidSegmentFraction(0.005);
    si_ = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(se2_space_));
    si_->setStateValidityChecker(
        std::make_shared<ValidityChecker>(si_, global_costmap_->getCostmap(), costmap_weight_));

    //
    // Update XY sample bounds
    //
    const double search_window =
        std::max(global_costmap_->getCostmap()->getSizeInCellsX() * global_costmap_->getCostmap()->getResolution(),
                 global_costmap_->getCostmap()->getSizeInCellsY() * global_costmap_->getCostmap()->getResolution());
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, -search_window);
    bounds.setHigh(0, +search_window);
    bounds.setLow(1, -search_window);
    bounds.setHigh(1, +search_window);
    se2_space_->as<ompl::base::SE2StateSpace>()->setBounds(bounds);
    // Optimize criteria
    ompl::base::OptimizationObjectivePtr cost_objective(new CostMapObjective(si_, global_costmap_->getCostmap()));
    ompl::base::OptimizationObjectivePtr length_objective(new ompl::base::PathLengthOptimizationObjective(si_));
    ompl::base::MultiOptimizationObjective* objective = new ompl::base::MultiOptimizationObjective(si_);
    // Highest cost is 252 so you would have 252 per m
    objective->addObjective(cost_objective, 1.0);
    // Highest cost would be 1 per m
    objective->addObjective(length_objective, 50.0);
    objective_ = ompl::base::OptimizationObjectivePtr(objective);
}

nav_core::PlanResult OmniRRTPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                              const geometry_msgs::PoseStamped& goal)
{
    const double search_window =
        std::max(global_costmap_->getCostmap()->getSizeInCellsX() * global_costmap_->getCostmap()->getResolution(),
                 global_costmap_->getCostmap()->getSizeInCellsY() * global_costmap_->getCostmap()->getResolution()) /
        2.0;

    nav_core::PlanResult result;

    //
    // Update XY sample bounds
    //
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, -search_window);
    bounds.setHigh(0, +search_window);
    bounds.setLow(1, -search_window);
    bounds.setHigh(1, +search_window);
    se2_space_->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

    const Eigen::Quaterniond start_qt = Eigen::Quaterniond(start.pose.orientation.w, start.pose.orientation.x,
                                                           start.pose.orientation.y, start.pose.orientation.z);
    const Eigen::Quaterniond goal_qt = Eigen::Quaterniond(goal.pose.orientation.w, goal.pose.orientation.x,
                                                          goal.pose.orientation.y, goal.pose.orientation.z);

    // Define problem
    ompl::base::ScopedState<> ompl_start(se2_space_);
    ompl_start[0] = start.pose.position.x;
    ompl_start[1] = start.pose.position.y;
    ompl_start[2] = getYaw(start_qt);

    ompl::base::ScopedState<> ompl_goal(se2_space_);
    ompl_goal[0] = goal.pose.position.x;
    ompl_goal[1] = goal.pose.position.y;
    ompl_goal[2] = getYaw(goal_qt);

    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si_));
    pdef->setOptimizationObjective(objective_);

    // A copy of the start and goal is made
    pdef->setStartAndGoalStates(ompl_start, ompl_goal, 0.01);

    ROS_INFO("Problem defined, running planner");
    auto rrt = new ompl::geometric::RRTstar(si_);
    rrt->setGoalBias(0.1);
    rrt->setRange(0.25);

    auto planner = ompl::base::PlannerPtr(rrt);
    planner->setProblemDefinition(pdef);
    planner->setup();

    ompl::base::PlannerStatus solved;

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*global_costmap_->getCostmap()->getMutex());

    solved = planner->solve(2.00);

    auto pd = std::make_shared<ompl::base::PlannerData>(si_);
    planner->getPlannerData(*pd);

    visualisePlannerData(*pd);

    if (ompl::base::PlannerStatus::StatusType(solved) == ompl::base::PlannerStatus::EXACT_SOLUTION)
    {
        result.success = true;
        const double length = pdef->getSolutionPath()->length();

        ROS_INFO_STREAM(planner->getName() << " found a solution of length " << length
                                           << " with an optimization objective value of " << result.cost);

        ompl::base::PathPtr path_ptr = pdef->getSolutionPath();
        ompl::geometric::PathGeometric result_path = static_cast<ompl::geometric::PathGeometric&>(*path_ptr);

        ompl::geometric::PathSimplifier simplifier(si_);
        simplifier.simplify(result_path, 0.1);
        // simplifier.smoothBSpline(result_path, 5, 0.005);
        result_path.interpolate();

        visualisePathGeometric(result_path);

        result.cost = result_path.cost(pdef->getOptimizationObjective()).value();
        result.plan = convert(result_path, global_costmap_->getGlobalFrameID());
    }
    else
    {
        result.success = false;
        result.cost = 0;
    }

    return result;
}

double OmniRRTPlanner::cost(const std::vector<geometry_msgs::PoseStamped>&)
{
    //    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*costmap_ros_->getCostmap()->getMutex());
    //    std::lock_guard<std::mutex> t_lock(trajectory_mutex_);
    //    std::lock_guard<std::mutex> c_lock(control_mutex_);

    //    if (!trajectory_result_ || !control_data_)
    //        return std::numeric_limits<double>::max();

    //    ompl::geometric::PathGeometric trajectory = *trajectory_result_->trajectory;
    //    const unsigned int i = std::max(1U, static_cast<unsigned int>(control_data_->execution_index)) - 1;
    //    trajectory.keepAfter(trajectory.getState(i));

    //    ROS_INFO("getRemainingTrajectoryCost DONE!");
    //    if (!trajectory.check())
    //        return std::numeric_limits<double>::max();

    //    return trajectory.cost(objective_).value();

    return std::numeric_limits<double>::max();
}

void OmniRRTPlanner::visualisePlannerData(const ompl::base::PlannerData& pd)
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

void OmniRRTPlanner::visualisePathGeometric(const ompl::geometric::PathGeometric& path)
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
}
