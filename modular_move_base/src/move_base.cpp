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

double getYaw(const double w, const double x, const double y, const double z)
{
    double yaw;

    const double sqw = w * w;
    const double sqx = x * x;
    const double sqy = y * y;
    const double sqz = z * z;

    // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
    double sarg = -2 * (x * z - w * y) / (sqx + sqy + sqz + sqw); /* normalization added from urdfom_headers */

    if (sarg <= -0.99999)
    {
        yaw = -2 * atan2(y, x);
    }
    else if (sarg >= 0.99999)
    {
        yaw = 2 * atan2(y, x);
    }
    else
    {
        yaw = atan2(2 * (x * y + w * z), sqw + sqx - sqy - sqz);
    }
    return yaw;
};
}

MoveBase::MoveBase()
    : nh_("~"), tf_buffer_(std::make_shared<tf2_ros::Buffer>()), tf_listener_(*tf_buffer_),
      as_(nh_, "/move_base", boost::bind(&MoveBase::executeCallback, this, _1), false),

      global_costmap_(std::make_shared<costmap_2d::Costmap2DROS>("global_costmap", *tf_buffer_)),
      local_costmap_(std::make_shared<costmap_2d::Costmap2DROS>("local_costmap", *tf_buffer_)),

      clear_costmaps_service_(nh_.advertiseService("clear_costmaps", &MoveBase::clearCostmapsCallback, this)),
      bgp_loader_("navigation_interface", "navigation_interface::BaseGlobalPlanner"),
      blp_loader_("navigation_interface", "navigation_interface::BaseLocalPlanner"),
      recovery_loader_("navigation_interface", "navigation_interface::RecoveryBehavior"), new_global_plan_(false),
      planner_frequency_(get_param_with_default_warn("~planner_frequency", 0.2)),
      controller_frequency_(get_param_with_default_warn("~controller_frequency", 20.0)),
      planner_patience_(get_param_with_default_warn("~planner_patience", 5.0)),
      controller_patience_(get_param_with_default_warn("~controller_patience", 15.0))
{
    ROS_INFO("Starting");

    const std::string global_planner = get_param_or_throw<std::string>("~base_global_planner");
    const std::string local_planner = get_param_or_throw<std::string>("~base_local_planner");

    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 1000, &MoveBase::odomCallback, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    current_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("current_goal", 0);

    // Pause the global costmap
    global_costmap_->pause();

    // Create the global planner
    try
    {
        ROS_INFO_STREAM("Starting global planner: " << global_planner);
        planner_ = bgp_loader_.createInstance(global_planner);
        ROS_INFO_STREAM("Created global planner: " << global_planner);

        try
        {
            planner_->initialize(bgp_loader_.getName(global_planner), tf_buffer_, global_costmap_, local_costmap_);
        }
        catch (const std::exception& e)
        {
            throw std::runtime_error("Failed to initialize the global planner: " + std::string(e.what()));
        }
    }
    catch (const pluginlib::PluginlibException& e)
    {
        throw std::runtime_error("Failed to create the global planner: " + std::string(e.what()));
    }

    // Pause the local costmap
    local_costmap_->pause();

    // Create the local planner
    try
    {
        ROS_INFO_STREAM("Starting local planner: " << local_planner);
        tc_ = blp_loader_.createInstance(local_planner);
        ROS_INFO_STREAM("Created local planner: " << local_planner);

        try
        {
            tc_->initialize(blp_loader_.getName(local_planner), tf_buffer_, local_costmap_);
        }
        catch (const std::exception& e)
        {
            throw std::runtime_error("Failed to initialize the global planner: " + std::string(e.what()));
        }
    }
    catch (const pluginlib::PluginlibException& e)
    {
        throw std::runtime_error("Failed to create the local planner: " + std::string(e.what()));
    }

    // Start actively updating costmaps based on sensor data
    global_costmap_->start();
    local_costmap_->start();

    // Load recovery behaviors
    if (!loadRecoveryBehaviors(nh_))
    {
        throw std::runtime_error("Failed to load recovery behaviours");
    }

    state_ = MoveBaseState::PLANNING;
    recovery_index_ = 0;

    as_.start();

    planner_thread_ = std::thread(&MoveBase::planThread, this);

    plan_service_ = nh_.advertiseService("plan", &MoveBase::planCallback, this);

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
    global_costmap_->resetLayers();
    local_costmap_->resetLayers();
    return true;
}

bool MoveBase::planCallback(modular_move_base::Plan::Request& req, modular_move_base::Plan::Response& res)
{
    ROS_INFO("Executing Plan service");

    geometry_msgs::PoseStamped goal;
    if (!goalToGlobalFrame(req.goal, goal))
    {
        ROS_WARN("Failed to transform goal into global frame");
        return true;
    }

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(global_costmap_->getCostmap()->getMutex()));

    // Get the starting pose of the robot
    geometry_msgs::PoseStamped start;
    if (req.start.header.frame_id.empty())
    {
        ROS_INFO_STREAM("Empty frame_id for start pose - using current robot position");

        if (!global_costmap_->getRobotPose(start))
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
    const navigation_interface::PlanResult result = planner_->makePlan(start, goal);
    if (result.success)
    {
        ROS_INFO_STREAM("Global plan found with length: " << result.plan.size() << " and cost: " << result.cost);
        res.plan.poses = result.plan;
        res.plan.header.frame_id = global_costmap_->getGlobalFrameID();
        res.success = true;
    }
    else
    {
        res.success = false;
    }

    ROS_INFO("Plan service complete");
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

bool MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg,
                                 geometry_msgs::PoseStamped& global_goal)
{
    const std::string global_frame = global_costmap_->getGlobalFrameID();

    try
    {
        ROS_INFO_STREAM("Transforming goal from " << goal_pose_msg.header.frame_id << " to " << global_frame);
        global_goal = tf_buffer_->transform(goal_pose_msg, global_frame, ros::Duration(1.0));
    }
    catch (const tf2::TransformException& ex)
    {
        ROS_WARN_STREAM("Failed to transform the goal pose from " << goal_pose_msg.header.frame_id << " to "
                                                                  << global_frame << " - " << ex.what());
        return false;
    }

    const double yaw = getYaw(global_goal.pose.orientation.w, global_goal.pose.orientation.x,
                              global_goal.pose.orientation.y, global_goal.pose.orientation.z);
    ROS_DEBUG_STREAM("Goal: x: " << global_goal.pose.position.x << " y: " << global_goal.pose.position.y
                                 << " yaw: " << yaw);

    return true;
}

void MoveBase::wakePlanner(const ros::TimerEvent&)
{
    std::lock_guard<std::mutex> planning_lock(planner_mutex_);
    if (state_ != MoveBaseState::GOAL_COMPLETE && state_ != MoveBaseState::GOAL_FAILED)
    {
        ROS_DEBUG_STREAM("Requesting planner: state=" << toString(state_));
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
            const std::string global_frame = global_costmap_->getGlobalFrameID();
            assert(goal.header.frame_id == global_frame);
        }

        ros::Time start_time = ros::Time::now();

        ROS_DEBUG("Planning");

        // Run planner
        {
            boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(global_costmap_->getCostmap()->getMutex()));

            // Get the starting pose of the robot
            geometry_msgs::PoseStamped start;
            if (global_costmap_->getRobotPose(start))
            {
                // If the planner fails or returns a zero length plan, planning failed
                const navigation_interface::PlanResult result = planner_->makePlan(start, goal);
                if (result.success && !result.plan.empty())
                {
                    ROS_INFO_STREAM("Global plan found with length: " << result.plan.size()
                                                                      << " and cost: " << result.cost);
                    std::lock_guard<std::mutex> planning_lock(planner_mutex_);
                    planner_plan_ = result.plan;
                    last_valid_plan_ = ros::Time::now();
                    new_global_plan_ = true;
                }
                else
                {
                    ROS_WARN("Failed to plan");
                }
            }
            else
            {
                ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
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

    last_valid_plan_ = ros::Time::now();

    recovery_index_ = 0;
    state_ = MoveBaseState::PLANNING;

    ros::Rate rate(controller_frequency_);
    while (nh_.ok())
    {
        // For timing that gives real time even in simulation
        const ros::SteadyTime steady_time = ros::SteadyTime::now();
        const ros::Time ros_time = ros::Time::now();

        if (as_.isPreemptRequested())
        {
            ROS_INFO("Preempting goal");
            publishZeroVelocity();

            {
                std::lock_guard<std::mutex> planning_lock(planner_mutex_);
                state_ = MoveBaseState::GOAL_FAILED;
            }

            as_.setPreempted();
            tc_->clearPlan();
            return;
        }

        // Run state machine
        MoveBaseState new_state = executeState(state_, steady_time, ros_time);
        {
            std::lock_guard<std::mutex> planning_lock(planner_mutex_);
            if (state_ != new_state)
            {
                ROS_DEBUG_STREAM("Transitioning from " << toString(state_) << " to " << toString(new_state));
                state_ = new_state;
            }

            if (state_ == MoveBaseState::GOAL_COMPLETE)
            {
                as_.setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached");
                tc_->clearPlan();
                return;
            }
        }

        // Update feedback to correspond to our current position
        geometry_msgs::PoseStamped current_position;
        global_costmap_->getRobotPose(current_position);
        move_base_msgs::MoveBaseFeedback feedback;
        feedback.base_position = current_position;
        as_.publishFeedback(feedback);

        const double t_diff = ros::SteadyTime::now().toSec() - steady_time.toSec();
        ROS_DEBUG_STREAM("Cycle time: " << t_diff);

        rate.sleep();

        if (rate.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == MoveBaseState::CONTROLLING)
        {
            ROS_WARN("Control loop missed desired rate of %.4fHz... took %.4f seconds", controller_frequency_,
                     rate.cycleTime().toSec());
        }
    }

    as_.setAborted(move_base_msgs::MoveBaseResult());
    tc_->clearPlan();
    return;
}

MoveBaseState MoveBase::executeState(const MoveBaseState state, const ros::SteadyTime& steady_time,
                                     const ros::Time& ros_time)
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

            boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(local_costmap_->getCostmap()->getMutex()));

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
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(local_costmap_->getCostmap()->getMutex()));

        {
            std::unique_lock<std::mutex> lock(planner_mutex_);
            if (new_global_plan_)
            {
                ROS_DEBUG("Got a new plan");
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

        if (!local_costmap_->isCurrent())
        {
            ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",
                     ros::this_node::getName().c_str());
            publishZeroVelocity();
            return MoveBaseState::CONTROLLING;
        }

        navigation_interface::Control control;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            control = tc_->computeControl(steady_time, ros_time, base_odom_);
        }

        if (control.state == navigation_interface::ControlState::RUNNING)
        {
            vel_pub_.publish(control.cmd_vel);
            recovery_index_ = 0;
            return MoveBaseState::CONTROLLING;
        }
        else if (control.state == navigation_interface::ControlState::EMERGENCY_BRAKING)
        {
            ROS_WARN_STREAM("ControlState == EMERGENCY_BRAKING");
            publishZeroVelocity();
            return MoveBaseState::CONTROLLING;
        }
        else if (control.state == navigation_interface::ControlState::COMPLETE)
        {
            publishZeroVelocity();
            return MoveBaseState::GOAL_COMPLETE;
        }
        else if (control.state == navigation_interface::ControlState::FAILED)
        {
            ROS_WARN_STREAM("ControlState == FAILED");
            publishZeroVelocity();
            return MoveBaseState::RECOVERING;
        }
        else
        {
            throw std::runtime_error("Unknown navigation_interface::ControlState: " +
                                     std::to_string(static_cast<int>(control.state)));
        }
    }
    else if (state == MoveBaseState::RECOVERING)
    {
        if (recovery_behaviors_.empty())
            return MoveBaseState::PLANNING;

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

                    boost::shared_ptr<navigation_interface::RecoveryBehavior> behavior(
                        recovery_loader_.createInstance(behavior_list[i]["type"]));

                    // shouldn't be possible, but it won't hurt to check
                    if (behavior.get() == nullptr)
                    {
                        ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should "
                                  "not happen");
                        return false;
                    }

                    // initialize the recovery behavior with its name
                    behavior->initialize(behavior_list[i]["name"], tf_buffer_, global_costmap_, local_costmap_);
                    recovery_behaviors_.push_back(behavior);
                }
                catch (const pluginlib::PluginlibException& ex)
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

    return true;
}

void MoveBase::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    base_odom_ = *msg;
}

}  // namespace move_base
