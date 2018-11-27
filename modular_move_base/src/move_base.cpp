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

MoveBase::MoveBase()
    : nh_("~"), tf_listener_(tf_buffer_),
      as_(nh_, "/move_base", boost::bind(&MoveBase::executeCallback, this, _1), false),
      planner_costmap_ros_("global_costmap", tf_buffer_), controller_costmap_ros_("local_costmap", tf_buffer_),
      clear_costmaps_service_(nh_.advertiseService("clear_costmaps", &MoveBase::clearCostmapsCallback, this)),
      plan_service_(nh_.advertiseService("plan", &MoveBase::planCallback, this)),
      bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"), blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
      recovery_loader_("nav_core", "nav_core::RecoveryBehavior"), new_global_plan_(false),
      planner_frequency_(get_param_with_default_warn("~planner_frequency", 0.2)),
      controller_frequency_(get_param_with_default_warn("~controller_frequency", 20.0)),
      planner_patience_(get_param_with_default_warn("~planner_patience", 5.0)),
      controller_patience_(get_param_with_default_warn("~controller_patience", 15.0))
{
    ROS_INFO("Starting");

    const std::string global_planner = get_param_or_throw<std::string>("~base_global_planner");
    const std::string local_planner = get_param_or_throw<std::string>("~base_local_planner");

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    current_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("current_goal", 0);

    // Pause the global costmap
    planner_costmap_ros_.pause();

    // Create the global planner
    try
    {
        ROS_INFO_STREAM("Starting global planner: " << global_planner);
        planner_ = bgp_loader_.createInstance(global_planner);
        ROS_INFO_STREAM("Created global planner: " << global_planner);
        planner_->initialize(bgp_loader_.getName(global_planner), &planner_costmap_ros_);
    }
    catch (const pluginlib::PluginlibException& ex)
    {
        throw std::runtime_error("Failed to create the global planner: " + std::string(ex.what()));
    }

    // Pause the local costmap
    controller_costmap_ros_.pause();

    // Create the local planner
    try
    {
        ROS_INFO_STREAM("Starting local planner: " << local_planner);
        tc_ = blp_loader_.createInstance(local_planner);
        ROS_INFO_STREAM("Created local planner: " << local_planner);
        tc_->initialize(blp_loader_.getName(local_planner), &tf_buffer_, &controller_costmap_ros_);
    }
    catch (const pluginlib::PluginlibException& ex)
    {
        throw std::runtime_error("Failed to create the local planner: " + std::string(ex.what()));
    }

    // Start actively updating costmaps based on sensor data
    planner_costmap_ros_.start();
    controller_costmap_ros_.start();

    // Load recovery behaviors
    if (!loadRecoveryBehaviors(nh_))
    {
        throw std::runtime_error("Failed to load recovery behaviours");
    }

    state_ = MoveBaseState::PLANNING;
    recovery_index_ = 0;

    as_.start();

    planner_thread_ = std::thread(&MoveBase::planThread, this);

    ROS_INFO("Successfully started");
}

MoveBase::~MoveBase()
{
    recovery_behaviors_.clear();

    planner_cond_.notify_one();
    planner_thread_.join();

    planner_.reset();
    tc_.reset();
}

bool MoveBase::clearCostmapsCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
    ROS_INFO("Executing clear costmaps service");
    planner_costmap_ros_.resetLayers();
    controller_costmap_ros_.resetLayers();
    return true;
}

bool MoveBase::planCallback(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res)
{
    ROS_INFO("Executing plan service");

    if (!planner_costmap_ros_.getLayeredCostmap())
    {
        ROS_WARN("Failed to plan, the layered costmap was null");
        return true;
    }

    if (!planner_costmap_ros_.getLayeredCostmap()->isInitialized())
    {
        ROS_WARN("Failed to plan, got the layered costmap, but it was not initialised");
        return true;
    }

    geometry_msgs::PoseStamped goal;
    if (!goalToGlobalFrame(req.goal, goal))
    {
        ROS_WARN("Failed to transform goal into global frame");
        return true;
    }

    if (!planner_costmap_ros_.getLayeredCostmap()->isInitialized())
    {
        ROS_WARN("Failed to plan");
        return true;
    }

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_.getCostmap()->getMutex()));

    // Get the starting pose of the robot
    geometry_msgs::PoseStamped start;
    if (req.start.header.frame_id.empty())
    {
        ROS_INFO_STREAM("Empty frame_id for start pose - using current robot position");

        if (!planner_costmap_ros_.getRobotPose(start))
        {
            ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
            return true;
        }
    }
    else
    {
        start = req.start;
    }

    // Run planner
    planner_->makePlan(start, goal, res.plan.poses);

    // Run planner
    res.plan.header.frame_id = planner_costmap_ros_.getGlobalFrameID();

    return true;
}

void MoveBase::publishZeroVelocity()
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
}

bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_.getCostmap()->getMutex()));

    plan.clear();

    if (!planner_costmap_ros_.getLayeredCostmap())
    {
        ROS_WARN("Failed to plan, the layered costmap was null");
        return false;
    }

    if (!planner_costmap_ros_.getLayeredCostmap()->isInitialized())
    {
        ROS_WARN("Failed to plan, got the layered costmap, but it was not initialised");
        return false;
    }

    // Get the starting pose of the robot
    geometry_msgs::PoseStamped start;
    if (!planner_costmap_ros_.getRobotPose(start))
    {
        ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
        return false;
    }

    // If the planner fails or returns a zero length plan, planning failed
    if (!planner_->makePlan(start, goal, plan) || plan.empty())
    {
        ROS_DEBUG_STREAM("Failed to plan to (" << goal.pose.position.x << ", " << goal.pose.position.y << ")");
        return false;
    }

    ROS_INFO_STREAM("Global plan found with length: " << plan.size());

    return true;
}

bool MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg,
                                 geometry_msgs::PoseStamped& global_goal)
{
    const std::string global_frame = planner_costmap_ros_.getGlobalFrameID();

    try
    {
        ROS_INFO_STREAM("Transforming goal from " << goal_pose_msg.header.frame_id << " to " << global_frame);
        global_goal = tf_buffer_.transform(goal_pose_msg, global_frame, ros::Duration(1.0));
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN_STREAM("Failed to transform the goal pose from " << goal_pose_msg.header.frame_id << " to "
                                                                  << global_frame << " - " << ex.what());
        return false;
    }

    return true;
}

void MoveBase::wakePlanner(const ros::TimerEvent&)
{
    std::lock_guard<std::mutex> planning_lock(planner_mutex_);
    if (state_ != MoveBaseState::GOAL_COMPLETE && state_ != MoveBaseState::GOAL_FAILED)
    {
        ROS_INFO_STREAM("Requesting planner: state=" << toString(state_));
        planner_cond_.notify_one();
    }
}

void MoveBase::planThread()
{
    ROS_DEBUG_NAMED("move_base_plan_thread", "Starting planner thread...");
    ros::Timer timer;

    while (nh_.ok())
    {
        geometry_msgs::PoseStamped goal;
        {
            std::unique_lock<std::mutex> planning_lock(planner_mutex_);

            ROS_DEBUG_NAMED("move_base_plan_thread", "Waiting for plan request");
            planner_cond_.wait(planning_lock);

            if (!nh_.ok())
                break;

            goal = planner_goal_;
            new_global_plan_ = false;

            ROS_DEBUG_STREAM("Request to plan to: " << goal.header.frame_id << " " << goal.pose.position.x << " "
                                                    << goal.pose.position.y);
            const std::string global_frame = planner_costmap_ros_.getGlobalFrameID();
            assert(goal.header.frame_id == global_frame);
        }

        ros::Time start_time = ros::Time::now();

        ROS_INFO("Planning");

        // Run planner
        std::vector<geometry_msgs::PoseStamped> plan;
        if (makePlan(goal, plan))
        {
            if (!plan.empty())
            {
                ROS_INFO_STREAM("Successfully Planned");

                std::lock_guard<std::mutex> planning_lock(planner_mutex_);
                planner_plan_ = plan;
                last_valid_plan_ = ros::Time::now();
                new_global_plan_ = true;
            }
            else
            {
                ROS_WARN("Returned plan is empty");
            }
        }

        // Setup sleep notify
        {
            std::lock_guard<std::mutex> planning_lock(planner_mutex_);
            if (state_ != MoveBaseState::GOAL_COMPLETE && state_ != MoveBaseState::GOAL_FAILED &&
                planner_frequency_ > 0)
            {
                ros::Duration sleep_time = (start_time + ros::Duration(1.0 / planner_frequency_)) - ros::Time::now();
                if (sleep_time > ros::Duration(0.0))
                {
                    timer = nh_.createTimer(sleep_time, &MoveBase::wakePlanner, this);
                }
            }
            else
            {
                timer.stop();
            }
        }
    }
}

void MoveBase::executeCallback(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
{
    ROS_INFO_STREAM("Received New Goal");

    geometry_msgs::PoseStamped goal;
    if (!goalToGlobalFrame(move_base_goal->target_pose, goal))
    {
        const std::string msg = "Failed to transform goal into global frame";
        ROS_WARN_STREAM(msg);

        {
            std::lock_guard<std::mutex> planning_lock(planner_mutex_);
            state_ = MoveBaseState::GOAL_FAILED;
        }

        as_.setAborted(move_base_msgs::MoveBaseResult(), msg);
        return;
    }

    current_goal_pub_.publish(goal);

    // Update the planner goal
    {
        std::lock_guard<std::mutex> planning_lock(planner_mutex_);
        new_global_plan_ = false;
        planner_goal_ = goal;
    }

    last_valid_control_ = ros::Time::now();
    last_valid_plan_ = ros::Time::now();

    recovery_index_ = 0;
    state_ = MoveBaseState::PLANNING;

    ros::Rate rate(controller_frequency_);
    while (nh_.ok())
    {
        // For timing that gives real time even in simulation
        ros::WallTime start = ros::WallTime::now();

        if (as_.isPreemptRequested())
        {
            ROS_INFO("Preempting goal");
            publishZeroVelocity();

            {
                std::lock_guard<std::mutex> planning_lock(planner_mutex_);
                state_ = MoveBaseState::GOAL_FAILED;
            }

            as_.setPreempted();
            return;
        }

        // Update feedback to correspond to our current position
        geometry_msgs::PoseStamped current_position;
        planner_costmap_ros_.getRobotPose(current_position);
        move_base_msgs::MoveBaseFeedback feedback;
        feedback.base_position = current_position;
        as_.publishFeedback(feedback);

        // Run state machine
        MoveBaseState new_state = executeState(state_);
        {
            std::lock_guard<std::mutex> planning_lock(planner_mutex_);
            if (state_ != new_state)
            {
                ROS_INFO_STREAM("Transitioning from " << toString(state_) << " to " << toString(new_state));
                state_ = new_state;
            }

            if (state_ == MoveBaseState::GOAL_COMPLETE)
            {
                as_.setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached");
                return;
            }
        }

        ros::WallDuration t_diff = ros::WallTime::now() - start;
        ROS_DEBUG_STREAM("Cycle time: " << t_diff.toSec());

        rate.sleep();

        if (rate.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == MoveBaseState::CONTROLLING)
        {
            ROS_WARN("Control loop missed desired rate of %.4fHz... took %.4f seconds", controller_frequency_,
                     rate.cycleTime().toSec());
        }
    }

    as_.setAborted(move_base_msgs::MoveBaseResult());
    return;
}

MoveBaseState MoveBase::executeState(const MoveBaseState state)
{
    ROS_DEBUG_STREAM("Executing state: " << state);

    if (state == MoveBaseState::PLANNING)
    {
        // If we didn't get a plan before timeout
        if (ros::Time::now() > last_valid_plan_ + ros::Duration(planner_patience_))
        {
            ROS_WARN("Timeout finding a valid global plan");
            publishZeroVelocity();
            return MoveBaseState::RECOVERING;
        }

        // Check for a new plan
        std::lock_guard<std::mutex> planning_lock(planner_mutex_);
        if (new_global_plan_)
        {
            ROS_DEBUG("Got a new plan");
            new_global_plan_ = false;

            boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(
                *(controller_costmap_ros_.getCostmap()->getMutex()));

            if (!tc_->setPlan(planner_plan_))
            {
                ROS_WARN("Could not initialise local planner with global plan");
                recovery_index_ = 0;
                return MoveBaseState::RECOVERING;
            }

            last_valid_plan_ = ros::Time::now();
            return MoveBaseState::CONTROLLING;
        }
        else
        {
            // Request the planner
            planner_cond_.notify_one();

            return MoveBaseState::PLANNING;
        }
    }
    else if (state == MoveBaseState::CONTROLLING)
    {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_.getCostmap()->getMutex()));

        if (tc_->isGoalReached())
        {
            ROS_INFO("Goal reached");
            publishZeroVelocity();
            return MoveBaseState::GOAL_COMPLETE;
        }

        {
            std::unique_lock<std::mutex> lock(planner_mutex_);
            if (new_global_plan_)
            {
                ROS_INFO("Got a new plan");
                new_global_plan_ = false;

                if (!tc_->setPlan(planner_plan_))
                {
                    ROS_WARN("Could not initialise local planner with global plan");
                    recovery_index_ = 0;
                    return MoveBaseState::RECOVERING;
                }

                last_valid_plan_ = ros::Time::now();
            }
        }

        if (!controller_costmap_ros_.isCurrent())
        {
            ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",
                     ros::this_node::getName().c_str());
            publishZeroVelocity();
            return MoveBaseState::CONTROLLING;
        }

        geometry_msgs::Twist cmd_vel;
        if (tc_->computeVelocityCommands(cmd_vel))
        {
            ROS_DEBUG_STREAM("Found a valid local plan: " << cmd_vel.linear.x << " " << cmd_vel.linear.y << " "
                                                          << cmd_vel.angular.z);

            last_valid_control_ = ros::Time::now();

            vel_pub_.publish(cmd_vel);

            recovery_index_ = 0;

            return MoveBaseState::CONTROLLING;
        }
        else
        {
            ROS_WARN("Could not find a valid local plan");
            publishZeroVelocity();

            if (ros::Time::now() > last_valid_control_ + ros::Duration(controller_patience_))
            {
                ROS_WARN("Timeout finding a valid local plan");
                return MoveBaseState::RECOVERING;
            }
            else
            {
                last_valid_plan_ = ros::Time::now();
                return MoveBaseState::PLANNING;
            }
        }
    }
    else if (state == MoveBaseState::RECOVERING)
    {
        if (recovery_index_ >= recovery_behaviors_.size())
        {
            ROS_INFO("Executed all recovery behaviours - restarting from first recovery behaviour");
            recovery_index_ = 0;
        }

        ROS_INFO_STREAM("Executing behavior " << recovery_index_ + 1 << " of " << recovery_behaviors_.size());
        recovery_behaviors_[recovery_index_]->runBehavior();

        recovery_index_++;
        last_valid_plan_ = ros::Time::now();
        return MoveBaseState::PLANNING;
    }
    else if (state == MoveBaseState::GOAL_COMPLETE)
    {
        publishZeroVelocity();
        return MoveBaseState::GOAL_COMPLETE;
    }
    else
    {
        throw std::runtime_error("Unknown state: " + std::to_string(static_cast<int>(state)));
    }
}

bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node)
{
    XmlRpc::XmlRpcValue behavior_list;
    if (node.getParam("recovery_behaviors", behavior_list))
    {
        if (behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            for (int i = 0; i < behavior_list.size(); ++i)
            {
                if (behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
                {
                    if (behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type"))
                    {
                        // check for recovery behaviors with the same name
                        for (int j = i + 1; j < behavior_list.size(); j++)
                        {
                            if (behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct)
                            {
                                if (behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type"))
                                {
                                    std::string name_i = behavior_list[i]["name"];
                                    std::string name_j = behavior_list[j]["name"];
                                    if (name_i == name_j)
                                    {
                                        ROS_ERROR("A recovery behavior with the name %s already exists, this is not "
                                                  "allowed. Using the default recovery behaviors instead.",
                                                  name_i.c_str());
                                        return false;
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        ROS_ERROR("Recovery behaviors must have a name and a type and this does not");
                        return false;
                    }
                }
                else
                {
                    ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d",
                              behavior_list[i].getType());
                    return false;
                }
            }

            // if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
            for (int i = 0; i < behavior_list.size(); ++i)
            {
                try
                {
                    // check if a non fully qualified name has potentially been passed in
                    if (!recovery_loader_.isClassAvailable(behavior_list[i]["type"]))
                    {
                        std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
                        for (unsigned int i = 0; i < classes.size(); ++i)
                        {
                            if (behavior_list[i]["type"] == recovery_loader_.getName(classes[i]))
                            {
                                // if we've found a match... we'll get the fully qualified name and break out of the
                                // loop
                                ROS_WARN("Recovery behavior specifications should now include the package name. You "
                                         "are using a deprecated API. Please switch from %s to %s in your yaml file.",
                                         std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                                behavior_list[i]["type"] = classes[i];
                                break;
                            }
                        }
                    }

                    boost::shared_ptr<nav_core::RecoveryBehavior> behavior(
                        recovery_loader_.createInstance(behavior_list[i]["type"]));

                    // shouldn't be possible, but it won't hurt to check
                    if (behavior.get() == nullptr)
                    {
                        ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should "
                                  "not happen");
                        return false;
                    }

                    // initialize the recovery behavior with its name
                    behavior->initialize(behavior_list[i]["name"], &tf_buffer_, &planner_costmap_ros_,
                                         &controller_costmap_ros_);
                    recovery_behaviors_.push_back(behavior);
                }
                catch (pluginlib::PluginlibException& ex)
                {
                    ROS_ERROR("Failed to load a plugin. Error: %s", ex.what());
                    return false;
                }
            }
        }
        else
        {
            ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d",
                      behavior_list.getType());
            return false;
        }
    }
    else
    {
        return false;
    }

    return true;
}

}  // namespace move_base
