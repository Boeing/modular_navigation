#include <autonomy/autonomy.h>
#include <autonomy/math.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <boost/tokenizer.hpp>
#include <geometry_msgs/Twist.h>
#include <map_manager/GetMap.h>
#include <map_manager/GetOccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/Path.h>
#include <navigation_interface/params.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <vector>

namespace autonomy
{

namespace
{

std::string uuid()
{
    std::stringstream ss;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 65536 - 1);
    for (std::size_t i = 0; i < 32; i++)
    {
        const auto rc = static_cast<std::uint16_t>(dis(gen));
        ss << std::hex << int(rc);
    }
    return ss.str();
}

template <class PluginType>
std::shared_ptr<PluginType> load(const YAML::Node& parameters, const std::string& planner_name,
                                 pluginlib::ClassLoader<PluginType>& loader,
                                 const std::shared_ptr<const gridmap::MapData>& costmap)
{
    try
    {
        const std::string class_name = parameters[planner_name].as<std::string>();
        std::shared_ptr<PluginType> sp(loader.createUnmanagedInstance(class_name));

        const YAML::Node plugin_params = parameters[loader.getName(class_name)];

        try
        {
            ROS_INFO_STREAM("Loading plugin: " << planner_name << " type: " << class_name);
            sp->initialize(plugin_params, costmap);
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

std::vector<std::shared_ptr<gridmap::Layer>> loadMapLayers(const YAML::Node& parameters,
                                                           pluginlib::ClassLoader<gridmap::Layer>& loader,
                                                           const std::vector<Eigen::Vector2d>& robot_footprint,
                                                           const std::shared_ptr<gridmap::RobotTracker>& robot_tracker,
                                                           std::shared_ptr<gridmap::URDFTree> urdf_tree)
{
    std::vector<std::shared_ptr<gridmap::Layer>> plugin_ptrs;
    if (parameters["layers"])
    {
        const YAML::Node layers_config = parameters["layers"];
        for (YAML::const_iterator it = layers_config.begin(); it != layers_config.end(); ++it)
        {
            ROS_ASSERT(it->IsMap());

            std::string pname = (*it)["name"].as<std::string>();
            std::string type = (*it)["type"].as<std::string>();

            try
            {
                ROS_INFO_STREAM("Loading plugin: " << pname << " type: " << type);
                auto plugin_ptr = std::shared_ptr<gridmap::Layer>(loader.createUnmanagedInstance(type));
                plugin_ptr->initialize(pname, parameters[pname], robot_footprint, robot_tracker, urdf_tree);
                plugin_ptrs.push_back(plugin_ptr);
            }
            catch (const pluginlib::PluginlibException& e)
            {
                throw std::runtime_error("Exception while loading plugin '" + pname + "': " + std::string(e.what()));
            }
            catch (const std::exception& e)
            {
                throw std::runtime_error("Exception while loading plugin '" + pname + "': " + std::string(e.what()));
            }
        }
    }

    return plugin_ptrs;
}

}  // namespace

Autonomy::Autonomy()
    : nh_("~"),

      as_(nh_, "/autonomy", boost::bind(&Autonomy::goalCallback, this, _1),
          boost::bind(&Autonomy::cancelCallback, this, _1), false),

      layer_loader_("gridmap", "gridmap::Layer"),
      pp_loader_("navigation_interface", "navigation_interface::PathPlanner"),
      tp_loader_("navigation_interface", "navigation_interface::TrajectoryPlanner"),
      c_loader_("navigation_interface", "navigation_interface::Controller"),

      running_(false), execution_thread_running_(false), controller_done_(false),

      current_path_(nullptr), current_trajectory_(nullptr)
{
    ROS_INFO("Starting");

    const std::string navigation_config = get_param_or_throw<std::string>("~navigation_config");

    YAML::Node root_config;
    try
    {
        root_config = YAML::LoadFile(navigation_config);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Navigation config is not valid: " << e.what());
        throw;
    }

    map_publish_frequency_ = root_config["map_publish_frequency"].as<double>(1.0);
    clear_radius_ = root_config["clear_radius"].as<double>(2.0);
    path_planner_frequency_ = root_config["path_planner_frequency"].as<double>(0.5);
    trajectory_planner_frequency_ = root_config["trajectory_planner_frequency"].as<double>(8.0);
    controller_frequency_ = root_config["controller_frequency"].as<double>(10.0);
    path_swap_fraction_ = root_config["path_swap_fraction"].as<double>(0.40);

    const std::vector<Eigen::Vector2d> robot_footprint = navigation_interface::get_point_list(
        root_config, "footprint",
        {{+0.490, +0.000}, {+0.408, -0.408}, {-0.408, -0.408}, {-0.490, +0.000}, {-0.408, +0.408}, {+0.408, +0.408}});

    robot_tracker_.reset(new gridmap::RobotTracker());
    urdf::Model urdf;
    urdf.initParam("robot_description");
    urdf_tree_.reset(new gridmap::URDFTree(urdf));

    const YAML::Node costmap_config = root_config["costmap"];

    auto layers = loadMapLayers(costmap_config, layer_loader_, robot_footprint, robot_tracker_, urdf_tree_);
    auto base_map_layer = std::make_shared<gridmap::BaseMapLayer>();
    base_map_layer->initialize("base_map", costmap_config["base_map"], robot_footprint, robot_tracker_, urdf_tree_);
    layered_map_ = std::make_shared<gridmap::LayeredMap>(base_map_layer, layers);

    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 1000, &Autonomy::odomCallback, this,
                                                  ros::TransportHints().tcpNoDelay());
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    current_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("current_goal", 1, true);
    path_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("path_goal", 1, true);
    path_pub_ = nh_.advertise<nav_msgs::Path>("path", 0, true);
    trajectory_pub_ = nh_.advertise<nav_msgs::Path>("trajectory", 0, true);

    // Create the planners
    path_planner_ = load<navigation_interface::PathPlanner>(root_config, "path_planner", pp_loader_, nullptr);
    trajectory_planner_ =
        load<navigation_interface::TrajectoryPlanner>(root_config, "trajectory_planner", tp_loader_, nullptr);
    controller_ = load<navigation_interface::Controller>(root_config, "controller", c_loader_, nullptr);

    active_map_sub_ = nh_.subscribe<hd_map::MapInfo>("/map_manager/active_map", 10, &Autonomy::activeMapCallback, this);
    mapper_status_sub_ =
        nh_.subscribe<cartographer_ros_msgs::SystemState>("/mapper/state", 1, &Autonomy::mapperCallback, this);

    costmap_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("costmap", 1);
    costmap_updates_publisher_ = nh_.advertise<map_msgs::OccupancyGridUpdate>("costmap_updates", 1);

    as_.start();

    execution_thread_running_ = true;
    execution_thread_ = std::thread(&Autonomy::executionThread, this);

    ROS_INFO("Successfully started");
}

Autonomy::~Autonomy()
{
    if (running_)
    {
        running_ = false;
    }
    if (execution_thread_running_)
    {
        execution_thread_running_ = false;
        execution_thread_.join();
    }
}

void Autonomy::activeMapCallback(const hd_map::MapInfo::ConstPtr& map)
{
    // preempt execution
    if (running_)
    {
        running_ = false;
    }

    // wait for goal completion
    std::unique_lock<std::mutex> lock(goal_mutex_);
    ROS_ASSERT(goal_ == nullptr);

    if (!map->name.empty())
    {
        ROS_INFO_STREAM("Received map (" << map->name << ")");

        auto map_client = nh_.serviceClient<map_manager::GetMap>("/map_manager/get_map");
        map_manager::GetMapRequest map_req;
        map_req.map_name = map->name;
        map_manager::GetMapResponse map_res;
        ROS_ASSERT(map_client.call(map_req, map_res));
        ROS_ASSERT(map_res.success);

        auto og_client = nh_.serviceClient<map_manager::GetOccupancyGrid>("/map_manager/get_occupancy_grid");
        map_manager::GetOccupancyGridRequest og_req;
        og_req.map_name = map->name;
        map_manager::GetOccupancyGridResponse og_res;
        ROS_ASSERT(og_client.call(og_req, og_res));
        ROS_ASSERT(og_res.success);

        layered_map_->setMap(map_res.map, og_res.grid);
    }
    else
    {
        ROS_INFO_STREAM("Generating an empty map");

        // generate an empty map
        hd_map::Map hd_map;
        hd_map.info.name = "";
        hd_map.info.meta_data.width = 2000;
        hd_map.info.meta_data.height = 2000;
        hd_map.info.meta_data.resolution = 0.02f;
        hd_map.info.meta_data.origin.position.x =
            -static_cast<double>(hd_map.info.meta_data.width) * hd_map.info.meta_data.resolution / 2.0;
        hd_map.info.meta_data.origin.position.y =
            -static_cast<double>(hd_map.info.meta_data.height) * hd_map.info.meta_data.resolution / 2.0;
        nav_msgs::OccupancyGrid map_data;
        map_data.info = hd_map.info.meta_data;
        map_data.data = std::vector<int8_t>(hd_map.info.meta_data.width * hd_map.info.meta_data.height, 0);
        layered_map_->setMap(hd_map, map_data);
    }

    path_planner_->setMapData(layered_map_->map());
    trajectory_planner_->setMapData(layered_map_->map());
    controller_->setMapData(layered_map_->map());

    {
        nav_msgs::OccupancyGrid grid = layered_map_->map()->grid.toMsg();
        grid.header.frame_id = "map";
        grid.header.stamp = ros::Time::now();
        costmap_publisher_.publish(grid);
    }

    ROS_INFO_STREAM("Finished updating map");
}

void Autonomy::executionThread()
{
    ros::Rate rate(1);
    while (execution_thread_running_)
    {
        std::unique_lock<std::mutex> lock(goal_mutex_);
        if (goal_ || execution_condition_.wait_for(lock, std::chrono::milliseconds(100)) == std::cv_status::no_timeout)
        {
            executeGoal(*goal_);
            goal_ = nullptr;
        }

        if (layered_map_->map())
        {
            const gridmap::RobotState robot_state = robot_tracker_->robotState();

            if (robot_state.localised)
            {
                layered_map_->update();

                {
                    nav_msgs::OccupancyGrid grid = layered_map_->map()->grid.toMsg();
                    grid.header.frame_id = "map";
                    grid.header.stamp = ros::Time::now();
                    costmap_publisher_.publish(grid);
                }
            }
            else
            {
                ROS_INFO_STREAM("Robot not localised");
            }
        }

        rate.sleep();
    }
}

void Autonomy::executeGoal(GoalHandle& goal)
{
    ROS_INFO_STREAM("Executing goal: " << goal.getGoalID().id);

    current_goal_pub_.publish(goal.getGoal()->target_pose);

    // make sure no threads are running
    ROS_ASSERT(!running_);
    ROS_ASSERT(!path_planner_thread_);
    ROS_ASSERT(!trajectory_planner_thread_);
    ROS_ASSERT(!controller_thread_);
    ROS_ASSERT(layered_map_->map());

    controller_done_ = false;
    running_ = true;

    const navigation_interface::PathPlanner::GoalSampleSettings goal_sample_settings = {
        goal.getGoal()->std_x, goal.getGoal()->std_y, goal.getGoal()->std_w, goal.getGoal()->max_samples};

    // start the threads
    path_planner_thread_.reset(new std::thread(&Autonomy::pathPlannerThread, this,
                                               convert(goal.getGoal()->target_pose.pose), goal_sample_settings));
    trajectory_planner_thread_.reset(new std::thread(&Autonomy::trajectoryPlannerThread, this));
    controller_thread_.reset(new std::thread(&Autonomy::controllerThread, this));

    // wait till all threads are done
    while (running_)
    {
        if (goal.getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTED)
        {
            break;
        }

        if (controller_done_)
        {
            goal.setSucceeded(autonomy::DriveResult(), "Goal reached");
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    running_ = false;
    controller_done_ = false;

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

    ROS_INFO_STREAM("Goal " << goal_->getGoalID().id << " execution complete");
}

void Autonomy::goalCallback(GoalHandle goal)
{
    ROS_INFO_STREAM("Received goal: " << goal.getGoalID().id << " (" << goal.getGoal()->target_pose.pose.position.x
                                      << ", " << goal.getGoal()->target_pose.pose.position.y << ")");
    ROS_INFO_STREAM("Goal standard deviations: X: " << goal.getGoal()->std_x << ", Y: " << goal.getGoal()->std_y
                                                    << ", W: " << goal.getGoal()->std_w
                                                    << ", Max Samples: " << goal.getGoal()->max_samples);
    {
        if (goal.getGoal()->std_x < 0.0 || goal.getGoal()->std_y < 0.0 || goal.getGoal()->std_w < 0.0)
        {
            ROS_INFO_STREAM("Rejected new goal: " << goal.getGoalID().id << " - Bad sample settings");
            goal.setRejected();
            return;
        }

        if (goal.getGoal()->target_pose.header.frame_id != global_frame_)
        {
            goal.setRejected();
            ROS_WARN_STREAM("Goal is not in global frame: " << global_frame_);
            return;
        }

        std::unique_lock<std::mutex> lock(goal_mutex_, std::try_to_lock);
        if (!lock.owns_lock())
        {
            // A Goal is already running... preempt
            if (goal_ && goal_->getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
            {
                ROS_INFO("Goal already executing. Cancelling...");
                goal_->setCanceled();
            }
            ROS_INFO("Waiting for previous goal to finish");
            lock.lock();
        }

        if (layered_map_->map())
        {
            ROS_INFO_STREAM("Accepted new goal: " << goal.getGoalID().id);
            goal.setAccepted();
            goal_ = std::make_unique<GoalHandle>(goal);
        }
        else
        {
            ROS_INFO_STREAM("Rejected new goal: " << goal.getGoalID().id << " - No Map!");
            goal.setRejected();
            return;
        }
    }
    execution_condition_.notify_all();
}

void Autonomy::cancelCallback(GoalHandle goal)
{
    std::unique_lock<std::mutex> lock(goal_mutex_, std::try_to_lock);
    if (!lock.owns_lock())
    {
        if (goal_ && goal.getGoalID().id == goal_->getGoalID().id)
        {
            autonomy::DriveFeedback feedback;
            feedback.state = autonomy::DriveFeedback::NO_PLAN;
            goal_->publishFeedback(feedback);
            goal.setCanceled();
        }
    }
}

void Autonomy::pathPlannerThread(const Eigen::Isometry2d& goal,
                                 const navigation_interface::PathPlanner::GoalSampleSettings goal_sample_settings)
{
    ros::WallRate rate(path_planner_frequency_);
    autonomy::DriveFeedback feedback;
    feedback.state = autonomy::DriveFeedback::NO_PLAN;

    while (running_)
    {
        const gridmap::RobotState robot_state = robot_tracker_->robotState();

        if (!robot_state.localised)
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            current_path_.reset();

            // publish
            nav_msgs::Path gui_path;
            gui_path.header.frame_id = global_frame_;
            path_pub_.publish(gui_path);

            feedback.state = autonomy::DriveFeedback::NO_PLAN;
            ROS_ASSERT(goal_);
            goal_->publishFeedback(feedback);

            ROS_INFO_STREAM("Robot not localised. Unable to plan!");

            rate.sleep();

            continue;
        }

        const Eigen::Isometry2d robot_pose = robot_state.map_to_odom * robot_state.odom.pose;

        {
            const auto t0 = std::chrono::steady_clock::now();
            const bool success = layered_map_->update();

            const double map_update_duration =
                std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0)
                    .count();
            if (map_update_duration > rate.expectedCycleTime().toSec())
                ROS_WARN_STREAM("Path Planning map update took too long: " << map_update_duration << "s");

            if (!success)
            {
                ROS_ERROR("Path Planning map update failed");

                std::lock_guard<std::mutex> lock(path_mutex_);
                current_path_.reset();

                // publish
                nav_msgs::Path gui_path;
                gui_path.header.frame_id = global_frame_;
                path_pub_.publish(gui_path);

                feedback.state = autonomy::DriveFeedback::NO_PLAN;
                ROS_ASSERT(goal_);
                goal_->publishFeedback(feedback);

                rate.sleep();

                continue;
            }

            {
                nav_msgs::OccupancyGrid grid = layered_map_->map()->grid.toMsg();
                grid.header.frame_id = "map";
                grid.header.stamp = ros::Time::now();
                costmap_publisher_.publish(grid);
            }
        }

        navigation_interface::PathPlanner::Result result;
        const auto now = ros::SteadyTime::now();
        {
            const auto t0 = std::chrono::steady_clock::now();
            result = path_planner_->plan(robot_pose, goal, goal_sample_settings);

            const double plan_duration =
                std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0)
                    .count();
            if (plan_duration > rate.expectedCycleTime().toSec())
                ROS_WARN_STREAM("Path Planning took too long: " << plan_duration << "s");
        }

        if (!running_)
            break;

        if (result.outcome == navigation_interface::PathPlanner::Outcome::SUCCESSFUL)
        {
            bool update = false;
            ROS_ASSERT(!result.path.nodes.empty());

            std::lock_guard<std::mutex> lock(path_mutex_);
            if (current_path_)
            {
                // trim the current path to the robot pose
                navigation_interface::Path path = current_path_->path;
                double distance_to_old_path = 0;
                if (!path.nodes.empty())
                {
                    const auto closest = path.closestSegment(robot_pose);
                    distance_to_old_path = closest.second;
                    if (closest.first != 0)
                        path.nodes.erase(path.nodes.begin(), path.nodes.begin() + static_cast<long>(closest.first));
                    path.nodes.insert(path.nodes.begin(), robot_pose);
                }

                // get the current path cost
                const double cost = path_planner_->cost(path);
                if (cost < std::numeric_limits<double>::max())
                {
                    current_path_->last_successful_time = now;
                    current_path_->last_successful_cost = cost;
                }

                const double time_since_successful_recalc = (now - current_path_->last_successful_time).toSec();

                const bool old_path_expired = time_since_successful_recalc > path_persistence_time_;
                const bool old_path_possible = cost < std::numeric_limits<double>::max();
                const bool new_path_possible = result.cost < std::numeric_limits<double>::max();
                const bool new_path_much_better = result.cost < (path_swap_fraction_ * cost);
                const bool old_path_is_long = path.length() > 1.0;
                const bool robot_near_old_path = distance_to_old_path < 0.25;

                // if we are no longer close to the old path then update
                const bool case_1 = !robot_near_old_path && new_path_possible;

                // when path becomes impossible wait a bit before swapping to a new path
                const bool case_2 = !old_path_possible && new_path_possible && old_path_expired;

                // if the new path is much better while the old path is still possible then swap
                const bool case_3 = new_path_much_better && old_path_possible && old_path_is_long;

                update = case_1 || case_2 || case_3;

                ROS_INFO_STREAM("swap: " << update << " old_cost: " << cost << " (" << old_path_possible << ")"
                                         << " new_cost: " << result.cost << " (" << new_path_possible << ")"
                                         << " new_is_better: " << new_path_much_better << " time_invalid: "
                                         << time_since_successful_recalc << "s (" << old_path_expired << ")"
                                         << " dist_to_old: " << distance_to_old_path << "m (" << robot_near_old_path
                                         << ")");
            }
            else
            {
                ROS_INFO_STREAM("First path found");
                update = true;
            }

            if (update)
            {
                result.path.id = uuid();
                current_path_.reset(new TrackingPath{goal, now, result.cost, now, result.cost, result.path});

                // publish
                nav_msgs::Path gui_path;
                gui_path.header.frame_id = global_frame_;
                for (const auto& node : result.path.nodes)
                {
                    const Eigen::Quaterniond qt(
                        Eigen::AngleAxisd(Eigen::Rotation2Dd(node.linear()).smallestAngle(), Eigen::Vector3d::UnitZ()));

                    geometry_msgs::PoseStamped p;
                    p.header.frame_id = global_frame_;
                    p.pose.position.x = node.translation().x();
                    p.pose.position.y = node.translation().y();
                    p.pose.orientation.w = qt.w();
                    p.pose.orientation.x = qt.x();
                    p.pose.orientation.y = qt.y();
                    p.pose.orientation.z = qt.z();

                    gui_path.poses.push_back(p);
                }
                path_goal_pub_.publish(gui_path.poses.back());
                path_pub_.publish(gui_path);
            }

            feedback.state = autonomy::DriveFeedback::GOOD_PLAN;
        }
        else
        {
            ROS_WARN("Failed to find a path");

            std::lock_guard<std::mutex> lock(path_mutex_);
            const double time_since_successful_recalc = current_path_
                                                            ? (now - current_path_->last_successful_time).toSec()
                                                            : std::numeric_limits<double>::max();

            // In the situation of persistent
            if (time_since_successful_recalc > 4.0)
            {
                feedback.state = autonomy::DriveFeedback::STUCK;

                current_path_.reset();

                std::lock_guard<std::mutex> t_lock(trajectory_mutex_);
                current_trajectory_.reset();

                ROS_INFO_STREAM("Clearing radius of " << clear_radius_ << "m around robot");
                layered_map_->clearRadius(robot_pose.translation(), clear_radius_);

                ROS_INFO("Waiting for sensors to stabilise...");
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            }
        }

        // Update feedback to correspond to our current position
        ROS_ASSERT(goal_);
        goal_->publishFeedback(feedback);

        rate.sleep();
    }

    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        current_path_.reset();
    }
}

void Autonomy::trajectoryPlannerThread()
{
    ros::WallRate rate(trajectory_planner_frequency_);

    ROS_ASSERT(layered_map_);
    const int size_x = static_cast<int>(8.0 / layered_map_->map()->grid.dimensions().resolution());
    const int size_y = static_cast<int>(8.0 / layered_map_->map()->grid.dimensions().resolution());

    while (running_)
    {
        // check if a new path is available
        bool has_path = false;
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            if (current_path_)
            {
                const auto p = trajectory_planner_->pathId();
                if (!p || current_path_->path.id != p.get())
                {
                    trajectory_planner_->setPath(current_path_->path);
                }
                has_path = true;
            }
        }

        if (!has_path)
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            current_trajectory_.reset();

            // publish
            nav_msgs::Path gui_path;
            gui_path.header.frame_id = "odom";
            trajectory_pub_.publish(gui_path);

            rate.sleep();
            continue;
        }

        const gridmap::RobotState robot_state = robot_tracker_->robotState();

        if (!robot_state.localised)
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            current_trajectory_.reset();

            // publish
            nav_msgs::Path gui_path;
            gui_path.header.frame_id = "odom";
            trajectory_pub_.publish(gui_path);

            rate.sleep();

            continue;
        }

        // update costmap around robot
        const Eigen::Isometry2d robot_pose = robot_state.map_to_odom * robot_state.odom.pose;
        const Eigen::Array2i robot_map = layered_map_->map()->grid.dimensions().getCellIndex(robot_pose.translation());

        const int top_left_x = std::max(0, robot_map.x() - size_x / 2);
        const int top_left_y = std::max(0, robot_map.y() - size_y / 2);

        const int actual_size_x =
            std::min(layered_map_->map()->grid.dimensions().size().x() - 1, top_left_x + size_x) - top_left_x;
        const int actual_size_y =
            std::min(layered_map_->map()->grid.dimensions().size().y() - 1, top_left_y + size_y) - top_left_y;

        ROS_ASSERT((top_left_x + actual_size_x) <= layered_map_->map()->grid.dimensions().size().x());
        ROS_ASSERT((top_left_y + actual_size_y) <= layered_map_->map()->grid.dimensions().size().y());

        const gridmap::AABB roi{{top_left_x, top_left_y}, {actual_size_x, actual_size_y}};

        {
            const auto t0 = std::chrono::steady_clock::now();
            const bool success = layered_map_->update(roi);

            const double map_update_duration =
                std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0)
                    .count();
            if (map_update_duration > rate.expectedCycleTime().toSec())
                ROS_WARN_STREAM("Trajectory Planning map update took too long: " << map_update_duration << "s");

            if (!success)
            {
                ROS_ERROR("Trajectory Planning map update failed");

                std::lock_guard<std::mutex> lock(trajectory_mutex_);
                current_trajectory_.reset();

                // publish
                nav_msgs::Path gui_path;
                gui_path.header.frame_id = "odom";
                trajectory_pub_.publish(gui_path);

                rate.sleep();

                continue;
            }

            {
                nav_msgs::OccupancyGrid grid = layered_map_->map()->grid.toMsg(roi);
                grid.header.frame_id = "map";
                grid.header.stamp = ros::Time::now();
                costmap_publisher_.publish(grid);
            }
        }

        navigation_interface::TrajectoryPlanner::Result result;
        {
            const auto t0 = std::chrono::steady_clock::now();

            // optimise path
            const auto ks = navigation_interface::KinodynamicState{robot_state.odom.pose, robot_state.odom.velocity, 0};
            result = trajectory_planner_->plan(roi, ks, robot_state.map_to_odom);

            const double plan_duration =
                std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0)
                    .count();
            if (plan_duration > rate.expectedCycleTime().toSec())
                ROS_WARN_STREAM("Trajectory Planning took too long: " << plan_duration << "s");
        }

        // update trajectory for the controller
        if (result.outcome == navigation_interface::TrajectoryPlanner::Outcome::SUCCESSFUL ||
            result.outcome == navigation_interface::TrajectoryPlanner::Outcome::PARTIAL)
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            result.trajectory.id = uuid();
            bool final_goal = (result.outcome == navigation_interface::TrajectoryPlanner::Outcome::SUCCESSFUL) &&
                              (trajectory_planner_->path().get().nodes.size() == result.path_end_i);
            current_trajectory_.reset(new ControlTrajectory({final_goal, result.trajectory}));

            // publish
            nav_msgs::Path gui_path;
            gui_path.header.frame_id = "odom";
            for (const auto& state : result.trajectory.states)
            {
                const Eigen::Quaterniond qt(
                    Eigen::AngleAxisd(Eigen::Rotation2Dd(state.pose.linear()).angle(), Eigen::Vector3d::UnitZ()));

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
        // clear the trajectory from the controller
        else
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            current_trajectory_.reset();

            // publish
            nav_msgs::Path gui_path;
            gui_path.header.frame_id = "odom";
            trajectory_pub_.publish(gui_path);
        }

        rate.sleep();
    }

    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        current_trajectory_.reset();
    }
}

void Autonomy::controllerThread()
{
    const long period_ms = static_cast<long>(1000.0 / controller_frequency_);
    ros::Time last_odom_time(0);

    ROS_ASSERT(layered_map_);
    const int size_x = static_cast<int>(5.0 / layered_map_->map()->grid.dimensions().resolution());
    const int size_y = static_cast<int>(5.0 / layered_map_->map()->grid.dimensions().resolution());

    while (running_)
    {
        // wait for a new odom (or timeout)
        gridmap::RobotState robot_state = robot_tracker_->robotState();
        try
        {
            if (robot_state.odom.time.toNSec() == last_odom_time.toNSec())
                robot_state = robot_tracker_->waitForRobotState(2 * period_ms);
        }
        catch (const std::exception& e)
        {
            ROS_ERROR_STREAM(e.what());
            vel_pub_.publish(geometry_msgs::Twist());
        }
        last_odom_time = robot_state.odom.time;

        // check if a new trajectory is available
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        if (current_trajectory_ && robot_state.localised)
        {
            const auto p = controller_->trajectoryId();
            if (!p || current_trajectory_->trajectory.id != p.get())
            {
                controller_->setTrajectory(current_trajectory_->trajectory);
            }
        }
        else
        {
            vel_pub_.publish(geometry_msgs::Twist());
            continue;
        }

        // update costmap around robot
        const Eigen::Isometry2d robot_pose = robot_state.map_to_odom * robot_state.odom.pose;
        const Eigen::Array2i robot_map = layered_map_->map()->grid.dimensions().getCellIndex(robot_pose.translation());

        const int top_left_x = std::max(0, robot_map.x() - size_x / 2);
        const int top_left_y = std::max(0, robot_map.y() - size_y / 2);

        const int actual_size_x =
            std::min(layered_map_->map()->grid.dimensions().size().x() - 1, top_left_x + size_x) - top_left_x;
        const int actual_size_y =
            std::min(layered_map_->map()->grid.dimensions().size().y() - 1, top_left_y + size_y) - top_left_y;

        ROS_ASSERT((top_left_x + actual_size_x) <= layered_map_->map()->grid.dimensions().size().x());
        ROS_ASSERT((top_left_y + actual_size_y) <= layered_map_->map()->grid.dimensions().size().y());

        const gridmap::AABB roi{{top_left_x, top_left_y}, {actual_size_x, actual_size_y}};

        {
            const auto t0 = std::chrono::steady_clock::now();

            const bool success = layered_map_->update(roi);

            if (!success)
            {
                ROS_ERROR("Control Planning map update failed");
                vel_pub_.publish(geometry_msgs::Twist());
                continue;
            }

            const double map_update_duration =
                std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0)
                    .count();
            if (1e3 * map_update_duration > period_ms)
            {
                ROS_ERROR_STREAM("Control Planning map update took too long: " << map_update_duration << "s");
                vel_pub_.publish(geometry_msgs::Twist());
                continue;
            }
        }

        navigation_interface::Controller::Result result;
        {
            const auto t0 = std::chrono::steady_clock::now();

            // control
            const auto ks = navigation_interface::KinodynamicState{robot_state.odom.pose, robot_state.odom.velocity, 0};
            const auto t = ros::SteadyTime::now();
            result = controller_->control(t, roi, ks, robot_state.map_to_odom);

            const double control_duration =
                std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0)
                    .count();
            if (1e3 * control_duration > period_ms)
            {
                ROS_ERROR_STREAM("Control Planning took too long: " << control_duration << "s");
                vel_pub_.publish(geometry_msgs::Twist());
                continue;
            }
        }

        // send cmd_vel
        geometry_msgs::Twist cmd_vel;
        if (result.outcome == navigation_interface::Controller::Outcome::SUCCESSFUL)
        {
            ROS_ASSERT(result.command.allFinite());
            cmd_vel.linear.x = result.command.x();
            cmd_vel.linear.y = result.command.y();
            cmd_vel.angular.z = result.command.z();
        }
        vel_pub_.publish(cmd_vel);

        // finish if end of path
        if (result.outcome == navigation_interface::Controller::Outcome::COMPLETE)
        {
            if (current_trajectory_->goal_trajectory)
            {
                ROS_INFO("Final trajectory complete");
                break;
            }
        }
    }

    geometry_msgs::Twist cmd_vel;
    vel_pub_.publish(cmd_vel);

    controller_done_ = true;
}

void Autonomy::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    robot_tracker_->addOdometryData(*msg);
}

void Autonomy::mapperCallback(const cartographer_ros_msgs::SystemState::ConstPtr& msg)
{
    const bool was_localised = robot_tracker_->localised();
    robot_tracker_->addLocalisationData(*msg);
    const bool is_localised = robot_tracker_->localised();

    // reset the layers when localisation switches
    if (was_localised != is_localised)
    {
        if (is_localised)
            ROS_INFO_STREAM("Robot localised!");
        else
            ROS_INFO_STREAM("Robot not localised!");

        if (layered_map_->map())
            layered_map_->clear();
    }
}

}  // namespace autonomy
