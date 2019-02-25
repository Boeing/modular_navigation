#include <eband_local_planner/eband_local_planner_ros.h>

#include <string>
#include <vector>

#include <pluginlib/class_list_macros.h>

#include <nav_core/base_local_planner.h>

PLUGINLIB_DECLARE_CLASS(eband_local_planner, EBandPlannerROS, eband_local_planner::EBandPlannerROS,
                        nav_core::BaseLocalPlanner)

namespace eband_local_planner
{

EBandPlannerROS::EBandPlannerROS() : tf_buffer_(nullptr), local_costmap_(nullptr), goal_reached_(false)
{
}

EBandPlannerROS::~EBandPlannerROS()
{
}

void EBandPlannerROS::initialize(const std::string& name, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                 const std::shared_ptr<costmap_2d::Costmap2DROS>& local_costmap)
{
    tf_buffer_ = tf_buffer;
    local_costmap_ = local_costmap;

    ros::NodeHandle gn;
    ros::NodeHandle pn("~/" + name);

    plan_pub_ = pn.advertise<nav_msgs::Path>("plan", 1);

    const int num_optim_iterations = pn.param("num_iterations_eband_optimization", 3);
    const double internal_force_gain = pn.param("eband_internal_force_gain", 1.0);
    const double external_force_gain = pn.param("eband_external_force_gain", 2.0);
    const double tiny_bubble_distance = pn.param("eband_tiny_bubble_distance", 0.01);
    const double tiny_bubble_expansion = pn.param("eband_tiny_bubble_expansion", 0.01);
    const double min_bubble_overlap = pn.param("eband_min_relative_bubble_overlap", 0.7);
    const int equilibrium_max_recursion_depth = pn.param("eband_equilibrium_approx_max_recursion_depth", 4);
    const double equilibrium_relative_overshoot = pn.param("eband_equilibrium_relative_overshoot", 0.75);
    const double significant_force = pn.param("eband_significant_force_lower_bound", 0.15);
    const double costmap_weight = pn.param("costmap_weight", 2.0);
    const double costmap_inflation_radius = pn.param("costmap_inflation_radius", 1.0) / 2.0;

    ROS_INFO_STREAM("tiny_bubble_distance: " << tiny_bubble_distance);
    ROS_INFO_STREAM("tiny_bubble_expansion: " << tiny_bubble_expansion);

    eband_ = std::shared_ptr<EBandOptimiser>(new EBandOptimiser(
        local_costmap_, num_optim_iterations, internal_force_gain, external_force_gain, tiny_bubble_distance,
        tiny_bubble_expansion, min_bubble_overlap, equilibrium_max_recursion_depth, equilibrium_relative_overshoot,
        significant_force, costmap_weight, costmap_inflation_radius));

    const double max_vel_lin = pn.param("max_vel_lin", 0.75);
    const double max_vel_th = pn.param("max_vel_th", 1.0);
    const double min_vel_lin = pn.param("min_vel_lin", 0.1);
    const double min_vel_th = pn.param("min_vel_th", 0.0);
    const double min_in_place_vel_th = pn.param("min_in_place_vel_th", 0.0);
    const double in_place_trans_vel = pn.param("in_place_trans_vel", 0.0);
    const double xy_goal_tolerance = pn.param("xy_goal_tolerance", 0.1);
    const double yaw_goal_tolerance = pn.param("yaw_goal_tolerance", 0.05);
    const double k_prop = pn.param("k_prop", 4.0);
    const double k_damp = pn.param("k_damp", 3.5);
    const double ctrl_rate = pn.param("ctrl_rate", 10.0);
    const double max_acceleration = pn.param("max_acceleration", 0.5);
    const double virtual_mass = pn.param("virtual_mass", 0.75);
    const double max_translational_acceleration = pn.param("max_translational_acceleration", 0.5);
    const double max_rotational_acceleration = pn.param("max_rotational_acceleration", 1.5);
    const double rotation_correction_threshold = pn.param("rotation_correction_threshold", 0.5);

    eband_trj_ctrl_ = std::shared_ptr<EBandTrajectoryCtrl>(new EBandTrajectoryCtrl(
        local_costmap_, max_vel_lin, max_vel_th, min_vel_lin, min_vel_th, min_in_place_vel_th, in_place_trans_vel,
        xy_goal_tolerance, yaw_goal_tolerance, k_prop, k_damp, ctrl_rate, max_acceleration, virtual_mass,
        max_translational_acceleration, max_rotational_acceleration, rotation_correction_threshold));

    eband_visual_ = std::shared_ptr<EBandVisualization>(new EBandVisualization(pn, local_costmap_));
    eband_->setVisualization(eband_visual_);
    eband_trj_ctrl_->setVisualization(eband_visual_);

    ROS_DEBUG("Elastic Band plugin initialized");
}

nav_core::Control EBandPlannerROS::computeControl(const ros::SteadyTime&, const ros::Time& now,
                                                  const nav_msgs::Odometry& odom)
{
    nav_core::Control result;

    try
    {
        const std::string robot_frame = local_costmap_->getBaseFrameID();
        const std::string local_frame = local_costmap_->getGlobalFrameID();
        const geometry_msgs::TransformStamped tr = tf_buffer_->lookupTransform(local_frame, robot_frame, now, ros::Duration(0.1));

        geometry_msgs::Pose local_robot_pose;
        local_robot_pose.position.x = tr.transform.translation.x;
        local_robot_pose.position.y = tr.transform.translation.y;
        local_robot_pose.position.z = tr.transform.translation.z;
        local_robot_pose.orientation = tr.transform.rotation;

        eband_->pruneTillPose(local_robot_pose);

        double distance = eband_->distance();
        const double max_distance = 1.0;
        const auto start_append = window_end_;
        auto end_append = window_end_;
        while (distance < max_distance && end_append != transformed_plan_.end())
        {
            ++end_append;

            if (end_append == transformed_plan_.end())
                break;

            distance += distance2D(*(end_append - 1), *end_append);
        }

        std::vector<geometry_msgs::Pose> window(start_append, end_append);
        window_end_ = end_append;
        eband_->append(window);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Moving window failed: " << e.what());
        eband_visual_->publishBand(eband_->band());
        result.state = nav_core::ControlState::FAILED;
        return result;
    }

    try
    {
        eband_->optimize();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Optimization failed: " << e.what());
        eband_visual_->publishBand(eband_->band());
        result.state = nav_core::ControlState::FAILED;
        return result;
    }

    if (!eband_trj_ctrl_->setBand(eband_->band()))
    {
        ROS_DEBUG("Failed to to set current band to Trajectory Controller");
        result.state = nav_core::ControlState::FAILED;
        return result;
    }

    if (!eband_trj_ctrl_->setOdometry(odom))
    {
        ROS_DEBUG("Failed to to set current odometry to Trajectory Controller");
        result.state = nav_core::ControlState::FAILED;
        return result;
    }

    geometry_msgs::Twist cmd_twist;
    if (!eband_trj_ctrl_->getTwist(cmd_twist, goal_reached_))
    {
        ROS_DEBUG("Failed to calculate Twist from band in Trajectory Controller");
        result.state = nav_core::ControlState::FAILED;
        return result;
    }

    ROS_DEBUG_STREAM("Retrieving velocity command: " << cmd_twist.linear.x << " " << cmd_twist.linear.y << " " << cmd_twist.angular.z);
    result.cmd_vel = cmd_twist;

    {
        const std::vector<geometry_msgs::Pose> refined_plan = convert(eband_->band());
        nav_msgs::Path gui_path;
        gui_path.header.frame_id = local_costmap_->getGlobalFrameID();
        gui_path.header.stamp = now;
        for (const geometry_msgs::Pose& pose : refined_plan)
        {
            geometry_msgs::PoseStamped ps;
            ps.header = gui_path.header;
            ps.pose = pose;
            gui_path.poses.push_back(ps);
        }
        plan_pub_.publish(gui_path);
    }

    eband_visual_->publishBand(eband_->band());

    if (goal_reached_)
        result.state = nav_core::ControlState::COMPLETE;
    else
        result.state = nav_core::ControlState::RUNNING;
    return result;
}

bool EBandPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (plan.empty())
        return false;

    try
    {
        // transform to the local costmap frame
        transformed_plan_ = transform(plan, *tf_buffer_, local_costmap_->getGlobalFrameID());

        // initialise moving window iterators
        window_end_ = transformed_plan_.cbegin() + 1;

        std::vector<geometry_msgs::Pose> window(transformed_plan_.cbegin(), window_end_);

        eband_->setPlan(window);

        eband_visual_->publishBand(eband_->band());

        goal_reached_ = false;
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Failed to setPlan: " << e.what());
        return false;
    }

    return true;
}

bool EBandPlannerROS::clearPlan()
{
    return true;
}

}
