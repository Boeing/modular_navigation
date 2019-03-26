#include <eband_local_planner/eband_local_planner_ros.h>

#include <string>
#include <vector>

#include <pluginlib/class_list_macros.h>

#include <navigation_interface/base_local_planner.h>

PLUGINLIB_DECLARE_CLASS(eband_local_planner, EBandPlannerROS, eband_local_planner::EBandPlannerROS,
                        navigation_interface::BaseLocalPlanner)

namespace eband_local_planner
{

EBandPlannerROS::EBandPlannerROS() : tf_buffer_(nullptr), local_costmap_(nullptr)
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
    const double tiny_bubble_distance = pn.param("eband_tiny_bubble_distance", 0.02);
    const double tiny_bubble_expansion = pn.param("eband_tiny_bubble_expansion", 0.02);
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


    const double max_velocity_x = pn.param("max_velocity_x", 0.15);
    const double max_velocity_y = pn.param("max_velocity_y", 0.05);
    const double max_velocity_w = pn.param("max_velocity_w", 0.10);

    const double max_acceleration_x = pn.param("max_acceleration_x", 0.10);
    const double max_acceleration_y = pn.param("max_acceleration_y", 0.10);
    const double max_acceleration_w = pn.param("max_acceleration_w", 0.10);

    const double goal_radius = pn.param("goal_radius", 0.06);

    const double xy_goal_tolerance = pn.param("xy_goal_tolerance", 0.002);
    const double yaw_goal_tolerance = pn.param("yaw_goal_tolerance", 0.01);

    const double k_prop = pn.param("k_prop", 0.5);
    const double k_damp = pn.param("k_damp", 4.0);

    velocity_controller_ = std::shared_ptr<EBandController>(new EBandController(
        local_costmap_, max_velocity_x, max_velocity_y, max_velocity_w, max_acceleration_x, max_acceleration_y,
        max_acceleration_w, goal_radius, xy_goal_tolerance, yaw_goal_tolerance, k_prop, k_damp));

    eband_visual_ = std::shared_ptr<EBandVisualization>(new EBandVisualization(pn, local_costmap_));
    eband_->setVisualization(eband_visual_);

    ROS_DEBUG("Elastic Band plugin initialized");
}

navigation_interface::Control EBandPlannerROS::computeControl(const ros::SteadyTime& steady_time, const ros::Time& ros_time,
                                                  const nav_msgs::Odometry& odom)
{
    navigation_interface::Control result;

    try
    {
        const std::string robot_frame = local_costmap_->getBaseFrameID();
        const std::string local_frame = local_costmap_->getGlobalFrameID();
        const geometry_msgs::TransformStamped tr =
            tf_buffer_->lookupTransform(local_frame, robot_frame, ros_time, ros::Duration(0.1));

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
        result.state = navigation_interface::ControlState::FAILED;
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
        result.state = navigation_interface::ControlState::FAILED;
        return result;
    }

    result = velocity_controller_->computeControl(eband_->band(), steady_time, ros_time, odom);

    {
        const std::vector<geometry_msgs::Pose> refined_plan = convert(eband_->band());
        nav_msgs::Path gui_path;
        gui_path.header.frame_id = local_costmap_->getGlobalFrameID();
        gui_path.header.stamp = ros_time;
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
