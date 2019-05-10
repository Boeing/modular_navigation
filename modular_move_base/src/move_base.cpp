#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <modular_move_base/move_base.h>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

namespace move_base
{

namespace
{

std::string uuid()
{
    std::stringstream ss;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 65536 - 1);
    for(std::size_t i = 0; i < 32; i++)
    {
        const auto rc = static_cast<std::uint16_t>(dis(gen));
        ss << std::hex << int(rc);
    }
    return ss.str();
}

Eigen::Isometry2d convert(const geometry_msgs::Pose& pose)
{
    const Eigen::Quaterniond qt(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    return Eigen::Translation2d(pose.position.x, pose.position.y) * Eigen::Rotation2Dd(qt.toRotationMatrix().eulerAngles(0, 1, 2)[2]);
}

Eigen::Isometry2d convert(const geometry_msgs::Transform& tr)
{
    const Eigen::Quaterniond qt(tr.rotation.w, tr.rotation.x, tr.rotation.y, tr.rotation.z);
    return Eigen::Translation2d(tr.translation.x, tr.translation.y) * Eigen::Rotation2Dd(qt.toRotationMatrix().eulerAngles(0, 1, 2)[2]);
}

template <class PluginType>
std::shared_ptr<PluginType> load(const ros::NodeHandle& nh, const std::string& class_name, pluginlib::ClassLoader<PluginType>& loader, const std::shared_ptr<const costmap_2d::Costmap2D>& costmap)
{
    try
    {
        ROS_INFO_STREAM("Starting: " << class_name);
        std::shared_ptr<PluginType> sp(loader.createUnmanagedInstance(class_name));

        XmlRpc::XmlRpcValue params;
        nh.getParam(loader.getName(class_name), params);

        try
        {
            ROS_INFO_STREAM("Initialising: " << class_name << "...");
            sp->initialize(params, costmap);
            ROS_INFO_STREAM("Successfully initialise: " << class_name);
        }
        catch (const std::exception& e)
        {
            throw std::runtime_error("Failed to initialize: " + std::string(e.what()));
        }

        return sp;
    }
    catch (const pluginlib::PluginlibException& e)
    {
        throw std::runtime_error("Failed to create: " + std::string(e.what()));
    }

    return nullptr;
}

}

MoveBase::MoveBase()
    : nh_("~"),

      tf_buffer_(std::make_shared<tf2_ros::Buffer>()),
      tf_listener_(*tf_buffer_),

      as_(nh_, "/move_base", boost::bind(&MoveBase::executeCallback, this, _1), false),

      pp_loader_("navigation_interface", "navigation_interface::PathPlanner"),
      tp_loader_("navigation_interface", "navigation_interface::TrajectoryPlanner"),
      c_loader_("navigation_interface", "navigation_interface::Controller"),

      costmap_("costmap", *tf_buffer_),

      running_(false),
      controller_done_(false),
      current_path_(nullptr),
      current_trajectory_(nullptr),

      path_planner_frequency_(get_param_with_default_warn("~path_planner_frequency", 1.0)),
      trajectory_planner_frequency_(get_param_with_default_warn("~trajectory_planner_frequency", 10.0)),
      controller_frequency_(get_param_with_default_warn("~controller_frequency", 20.0)),
      path_swap_fraction_(get_param_with_default_warn("~path_swap_fraction", 0.8))
{
    ROS_INFO("Starting");

    as_.registerPreemptCallback(boost::bind(&MoveBase::preemptedCallback, this));

    const std::string path_planner_name = get_param_or_throw<std::string>("~path_planner");
    const std::string trajectory_planner_name = get_param_or_throw<std::string>("~trajectory_planner");
    const std::string controller_name = get_param_or_throw<std::string>("~controller");

    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 1000, &MoveBase::odomCallback, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    current_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("current_goal", 0);
    path_pub_ = nh_.advertise<nav_msgs::Path>("path", 0, true);
    trajectory_pub_ = nh_.advertise<nav_msgs::Path>("trajectory", 0, true);

    costmap_.pause();

    // Create the path planner
    path_planner_ = load<navigation_interface::PathPlanner>(nh_, path_planner_name, pp_loader_, costmap_.getCostmap());
    trajectory_planner_ = load<navigation_interface::TrajectoryPlanner>(nh_, trajectory_planner_name, tp_loader_, costmap_.getCostmap());
    controller_ = load<navigation_interface::Controller>(nh_, controller_name, c_loader_, costmap_.getCostmap());

    costmap_.start();

    as_.start();

    ROS_INFO("Successfully started");
}

MoveBase::~MoveBase()
{

}

void MoveBase::executeCallback(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
{
    ROS_INFO_STREAM("Received New Goal");

    if (move_base_goal->target_pose.header.frame_id != costmap_.getGlobalFrameID())
    {
        const std::string msg = "Goal must be in the global frame";
        ROS_WARN_STREAM(msg);
        as_.setAborted(move_base_msgs::MoveBaseResult(), msg);
        return;
    }

    current_goal_pub_.publish(move_base_goal->target_pose);

    // make sure no threads are running
    ROS_ASSERT(!running_);
    ROS_ASSERT(!path_planner_thread_);
    ROS_ASSERT(!trajectory_planner_thread_);
    ROS_ASSERT(!controller_thread_);

    controller_done_ = false;
    running_ = true;

    // start the threads
    path_planner_thread_.reset(new std::thread(&MoveBase::pathPlannerThread, this, convert(move_base_goal->target_pose.pose)));
    trajectory_planner_thread_.reset(new std::thread(&MoveBase::trajectoryPlannerThread, this));
    controller_thread_.reset(new std::thread(&MoveBase::controllerThread, this));

    // wait till all threads are done
    while (ros::ok())
    {
        if (as_.isPreemptRequested())
        {
            as_.setPreempted();
            break;
        }

        if (controller_done_)
        {
            as_.setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached");
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    running_ = false;

    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        current_path_.reset();
    }

    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        current_trajectory_.reset();
    }

    path_planner_thread_->join();
    trajectory_planner_thread_->join();
    controller_thread_->join();

    path_planner_thread_.reset();
    trajectory_planner_thread_.reset();
    controller_thread_.reset();
}

void MoveBase::preemptedCallback()
{

}

void MoveBase::pathPlannerThread(const Eigen::Isometry2d& goal)
{
    ros::WallRate rate(path_planner_frequency_);

    while (running_)
    {
        // copy the current robot state
        RobotState rs;
        {
            std::lock_guard<std::mutex> lock(robot_state_mutex_);
            rs = robot_state_;
        }
        const Eigen::Isometry2d global_robot_pose = rs.map_to_odom * rs.robot_state.pose;

        auto result = path_planner_->plan(global_robot_pose, goal);

        if (result.outcome == navigation_interface::PathPlanner::Outcome::SUCCESSFUL)
        {
            ROS_INFO_STREAM("Global path found with length: " << result.path.length() << " and cost: " << result.cost);

            bool update = false;

            std::lock_guard<std::mutex> lock(path_mutex_);
            if (current_path_)
            {
                ROS_INFO("Comparing new path with existing path");

                // trim the current path to the robot pose
                navigation_interface::Path path = *current_path_;
                const auto closest = path.closestSegment(global_robot_pose);
                if (closest.first != 0)
                    path.nodes.erase(path.nodes.begin(), path.nodes.begin() + static_cast<long>(closest.first));

                // get the current path cost
                const double cost = path_planner_->cost(result.path);

                ROS_INFO_STREAM("Current path cost: " << cost);

                if (result.cost < path_swap_fraction_ * cost)
                {
                    ROS_INFO_STREAM("New Path is better than existing - swapping");
                    update = true;
                }

                // TODO also swap if tracking error is too high or control has failed
            }
            else
            {
                ROS_INFO_STREAM("First path found");
                update = true;
            }

            if (update)
            {
                result.path.id = uuid();
                current_path_.reset(new navigation_interface::Path(result.path));

                // publish
                nav_msgs::Path gui_path;
                gui_path.header.frame_id = costmap_.getGlobalFrameID();
                for (const auto& node : result.path.nodes)
                {
                    const Eigen::Quaterniond qt(Eigen::AngleAxisd(Eigen::Rotation2Dd(node.linear()).angle(), Eigen::Vector3d::UnitZ()));

                    geometry_msgs::PoseStamped p;
                    p.header.frame_id = costmap_.getGlobalFrameID();
                    p.pose.position.x = node.translation().x();
                    p.pose.position.y = node.translation().y();
                    p.pose.orientation.w = qt.w();
                    p.pose.orientation.x = qt.x();
                    p.pose.orientation.y = qt.y();
                    p.pose.orientation.z = qt.z();

                    gui_path.poses.push_back(p);
                }
                path_pub_.publish(gui_path);
            }

        }
        else
        {
            ROS_WARN("Failed to find a path");
        }

        rate.sleep();
    }

    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        current_path_.reset();
    }
}

void MoveBase::trajectoryPlannerThread()
{
    ros::WallRate rate(trajectory_planner_frequency_);

    while (running_)
    {
        // check if a new path is available
        bool has_path = false;
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            if (current_path_)
            {
                const auto p = trajectory_planner_->pathId();
                if (!p || current_path_->id != p.get())
                {
                    trajectory_planner_->setPath(*current_path_);
                }
                has_path = true;
            }
        }

        if (!has_path)
        {
            rate.sleep();
            continue;
        }

        // copy the current robot state
        RobotState rs;
        {
            std::lock_guard<std::mutex> lock(robot_state_mutex_);
            rs = robot_state_;
        }

        // optimise path
        auto result = trajectory_planner_->plan(rs.robot_state, rs.map_to_odom);

        ROS_INFO_STREAM("Got a new trajectory: size: " << result.trajectory.states.size());

        // update trajectory for the controller
        if (result.outcome == navigation_interface::TrajectoryPlanner::Outcome::SUCCESSFUL
                | result.outcome == navigation_interface::TrajectoryPlanner::Outcome::PARTIAL)
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            result.trajectory.id = uuid();
            current_trajectory_.reset(new ControlTrajectory({trajectory_planner_->path().get().nodes.size() == result.path_end_i, result.trajectory}));

            // publish
            nav_msgs::Path gui_path;
            gui_path.header.frame_id = "odom";
            for (const auto& state : result.trajectory.states)
            {
                const Eigen::Quaterniond qt(Eigen::AngleAxisd(Eigen::Rotation2Dd(state.pose.linear()).angle(), Eigen::Vector3d::UnitZ()));

                geometry_msgs::PoseStamped p;
                p.header.frame_id = "odom";
                p.pose.position.x = state.pose.translation().x();
                p.pose.position.y = state.pose.translation().y();
                p.pose.orientation.w = qt.w();
                p.pose.orientation.x = qt.x();
                p.pose.orientation.y = qt.y();
                p.pose.orientation.z = qt.z();

                gui_path.poses.push_back(p);
            }
            trajectory_pub_.publish(gui_path);
        }

        rate.sleep();
    }

    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        current_trajectory_.reset();
    }
}

void MoveBase::controllerThread()
{
    const long period_ms = static_cast<long>(1000.0 / controller_frequency_);

    while (running_)
    {
        // wait for a new odom (or timeout)
        {
            std::unique_lock<std::mutex> lock(robot_state_mutex_);
            if (robot_state_conditional_.wait_for(lock, std::chrono::milliseconds(period_ms)) == std::cv_status::timeout)
            {
                ROS_WARN_STREAM("Did not receive an odom message at the desired frequency");
                vel_pub_.publish(geometry_msgs::Twist());
                continue;
            }
        }

        // check if a new trajectory is available
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        if (current_trajectory_)
        {
            const auto p = controller_->trajectoryId();
            if (!p || current_trajectory_->trajectory.id != p.get())
            {
                controller_->setTrajectory(current_trajectory_->trajectory);
            }
        }
        else
        {
            continue;
        }

        // copy the current robot state
        RobotState rs;
        {
            std::lock_guard<std::mutex> lock(robot_state_mutex_);
            rs = robot_state_;
        }

        // Update feedback to correspond to our current position
        const Eigen::Isometry2d global_robot_pose = rs.map_to_odom * rs.robot_state.pose;
        const Eigen::Quaterniond qt = Eigen::Quaterniond(Eigen::AngleAxisd(Eigen::Rotation2Dd(global_robot_pose.linear()).angle(), Eigen::Vector3d::UnitZ()));
        move_base_msgs::MoveBaseFeedback feedback;
        feedback.base_position.header.frame_id = costmap_.getGlobalFrameID();
        feedback.base_position.pose.position.x = global_robot_pose.translation().x();
        feedback.base_position.pose.position.y = global_robot_pose.translation().y();
        feedback.base_position.pose.orientation.w = qt.w();
        feedback.base_position.pose.orientation.x = qt.x();
        feedback.base_position.pose.orientation.y = qt.y();
        feedback.base_position.pose.orientation.z = qt.z();
        as_.publishFeedback(feedback);

        // control
        const auto result = controller_->control(rs.time, rs.robot_state, rs.map_to_odom);

        // send cmd_vel
        geometry_msgs::Twist cmd_vel;
        if (result.outcome == navigation_interface::Controller::Outcome::SUCCESSFUL)
        {
            cmd_vel.linear.x = result.command.x();
            cmd_vel.linear.y = result.command.y();
            cmd_vel.angular.z = result.command.z();
        }
        vel_pub_.publish(cmd_vel);

        // finish if end of path
        if (result.outcome == navigation_interface::Controller::Outcome::COMPLETE && current_trajectory_->goal_trajectory)
        {
            ROS_INFO("Final trajectory complete");
            break;
        }
    }

    geometry_msgs::Twist cmd_vel;
    vel_pub_.publish(cmd_vel);

    controller_done_ = true;
}

void MoveBase::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(robot_state_mutex_);

    robot_state_.time = ros::SteadyTime::now();

    robot_state_.robot_state.pose = convert(msg->pose.pose);
    robot_state_.robot_state.velocity = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.angular.z);

    const geometry_msgs::TransformStamped tr = tf_buffer_->lookupTransform(costmap_.getGlobalFrameID(), "odom", ros::Time(0));
    robot_state_.map_to_odom = convert(tr.transform);

    robot_state_conditional_.notify_all();
}

}
