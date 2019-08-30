#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <vector>

#include <boost/tokenizer.hpp>

#include <boost/tokenizer.hpp>

#include <autonomy/autonomy.h>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <nav_msgs/Path.h>

#include <navigation_interface/params.h>

#include <map_manager/GetMap.h>
#include <map_manager/GetOccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>

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

Eigen::Isometry2d convert(const geometry_msgs::Pose& pose)
{
    const double yaw =
        std::atan2(2.0 * (pose.orientation.z * pose.orientation.w + pose.orientation.x * pose.orientation.y),
                   -1.0 + 2.0 * (pose.orientation.w * pose.orientation.w + pose.orientation.x * pose.orientation.x));
    return Eigen::Translation2d(pose.position.x, pose.position.y) * Eigen::Rotation2Dd(yaw);
}

Eigen::Isometry2d convert(const geometry_msgs::Transform& tr)
{
    const double yaw = std::atan2(2.0 * (tr.rotation.z * tr.rotation.w + tr.rotation.x * tr.rotation.y),
                                  -1.0 + 2.0 * (tr.rotation.w * tr.rotation.w + tr.rotation.x * tr.rotation.x));
    return Eigen::Translation2d(tr.translation.x, tr.translation.y) * Eigen::Rotation2Dd(yaw);
}

template <class PluginType>
std::shared_ptr<PluginType> load(const ros::NodeHandle& nh, const std::string& class_name,
                                 pluginlib::ClassLoader<PluginType>& loader,
                                 const std::shared_ptr<const gridmap::MapData>& costmap)
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

std::vector<std::shared_ptr<gridmap::Layer>> loadMapLayers(XmlRpc::XmlRpcValue& parameters,
                                                           const std::string& global_frame,
                                                           pluginlib::ClassLoader<gridmap::Layer>& loader,
                                                           const std::vector<Eigen::Vector2d>& robot_footprint,
                                                           const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
{
    std::vector<std::shared_ptr<gridmap::Layer>> plugin_ptrs;
    if (parameters.hasMember("layers"))
    {
        XmlRpc::XmlRpcValue my_list = parameters["layers"];
        for (int32_t i = 0; i < my_list.size(); ++i)
        {
            std::string pname = static_cast<std::string>(my_list[i]["name"]);
            std::string type = static_cast<std::string>(my_list[i]["type"]);

            try
            {
                ROS_INFO_STREAM("Loading plugin: " << pname << " type: " << type);
                auto plugin_ptr = std::shared_ptr<gridmap::Layer>(loader.createUnmanagedInstance(type));
                plugin_ptr->initialize(pname, global_frame, parameters[pname], robot_footprint, tf_buffer);
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

      tf_buffer_(std::make_shared<tf2_ros::Buffer>()), tf_listener_(*tf_buffer_),

      as_(nh_, "/autonomy", boost::bind(&Autonomy::goalCallback, this, _1),
          boost::bind(&Autonomy::cancelCallback, this, _1), false),

      layer_loader_("gridmap", "gridmap::Layer"),
      pp_loader_("navigation_interface", "navigation_interface::PathPlanner"),
      tp_loader_("navigation_interface", "navigation_interface::TrajectoryPlanner"),
      c_loader_("navigation_interface", "navigation_interface::Controller"),

      running_(false), execution_thread_running_(false), controller_done_(false),

      current_path_(nullptr), current_trajectory_(nullptr),

      map_publish_frequency_(get_param_with_default_warn("~map_publish_frequency", 1.0)),

      global_frame_("map"),

      clear_radius_(get_param_with_default_warn("~clear_radius", 0.5)),

      path_planner_frequency_(get_param_with_default_warn("~path_planner_frequency", 0.5)),
      trajectory_planner_frequency_(get_param_with_default_warn("~trajectory_planner_frequency", 8.0)),
      controller_frequency_(get_param_with_default_warn("~controller_frequency", 10.0)),
      path_swap_fraction_(get_param_with_default_warn("~path_swap_fraction", 0.40)),
      localisation_timeout_(get_param_with_default_warn("~localisation_timeout", 4.0))
{
    ROS_INFO("Starting");

    XmlRpc::XmlRpcValue costmap_params;

    std::vector<Eigen::Vector2d> robot_footprint;
    if (costmap_params.hasMember("footprint"))
    {
        robot_footprint = navigation_interface::get_point_list(costmap_params, "footprint");
    }
    else
    {
        ROS_WARN("Loading default robot footprint for Ridgeback");
        robot_footprint = {{0.500, 0.000},  {0.420, -0.420}, {-0.420, -0.420},
                           {-0.500, 0.000}, {-0.420, 0.420}, {0.420, 0.420}};
    }

    nh_.getParam("costmap", costmap_params);
    auto layers = loadMapLayers(costmap_params, global_frame_, layer_loader_, robot_footprint, tf_buffer_);
    auto base_map_layer = std::make_shared<gridmap::BaseMapLayer>();
    base_map_layer->initialize("base_map", global_frame_, costmap_params["base_map"], robot_footprint, tf_buffer_);
    layered_map_ = std::make_shared<gridmap::LayeredMap>(base_map_layer, layers);

    const std::string path_planner_name = get_param_or_throw<std::string>("~path_planner");
    const std::string trajectory_planner_name = get_param_or_throw<std::string>("~trajectory_planner");
    const std::string controller_name = get_param_or_throw<std::string>("~controller");

    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 1000, &Autonomy::odomCallback, this,
                                                  ros::TransportHints().tcpNoDelay());
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    current_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("current_goal", 0);
    path_pub_ = nh_.advertise<nav_msgs::Path>("path", 0, true);
    trajectory_pub_ = nh_.advertise<nav_msgs::Path>("trajectory", 0, true);

    // Create the path planner
    path_planner_ = load<navigation_interface::PathPlanner>(nh_, path_planner_name, pp_loader_, nullptr);
    trajectory_planner_ =
        load<navigation_interface::TrajectoryPlanner>(nh_, trajectory_planner_name, tp_loader_, nullptr);
    controller_ = load<navigation_interface::Controller>(nh_, controller_name, c_loader_, nullptr);

    active_map_sub_ =
        nh_.subscribe<hd_map::MapInfo>("/map_manager/active_map", 10, &Autonomy::activeMapCallback, this);

    costmap_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("costmap", 1);
    costmap_updates_publisher_ = nh_.advertise<map_msgs::OccupancyGridUpdate>("costmap_updates", 1);

    as_.start();

    execution_thread_running_ = true;
    execution_thread_ = std::thread(&Autonomy::executionThread, this);

    ROS_INFO("Successfully started");
}

Autonomy::~Autonomy()
{
    if (execution_thread_running_)
    {
        execution_thread_running_ = false;
        execution_thread_.join();
    }
    if (running_)
    {
        running_ = false;
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
        hd_map.info.meta_data.resolution = 0.02;
        hd_map.info.meta_data.origin.position.x = -static_cast<double>(hd_map.info.meta_data.width) * hd_map.info.meta_data.resolution / 2.0;
        hd_map.info.meta_data.origin.position.y = -static_cast<double>(hd_map.info.meta_data.height) * hd_map.info.meta_data.resolution / 2.0;
        nav_msgs::OccupancyGrid map_data;
        map_data.info = hd_map.info.meta_data;
        map_data.data = std::vector<int8_t>(hd_map.info.meta_data.width*hd_map.info.meta_data.height, 0);
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
}

void Autonomy::executionThread()
{
    while (execution_thread_running_)
    {
        std::unique_lock<std::mutex> lock(goal_mutex_);
        if (execution_condition_.wait_for(lock, std::chrono::milliseconds(100)) == std::cv_status::no_timeout)
        {
            executeGoal(*goal_);
            goal_ = nullptr;
        }

        // copy the current robot state
        RobotState rs;
        {
            std::lock_guard<std::mutex> _lock(robot_state_mutex_);
            rs = robot_state_;
        }

        if (layered_map_->map())
        {
            if (rs.localised)
            {
                //                const Eigen::Isometry2d robot_pose = rs.map_to_odom * rs.robot_state.pose;
                //                layered_map_->clearRadius(robot_pose.translation(), clear_radius_);
                layered_map_->update();
            }

            {
                nav_msgs::OccupancyGrid grid = layered_map_->map()->grid.toMsg();
                grid.header.frame_id = "map";
                grid.header.stamp = ros::Time::now();
                costmap_publisher_.publish(grid);
            }
        }
    }
}

void Autonomy::executeGoal(GoalHandle& goal)
{
    ROS_INFO_STREAM("Received New Goal");

    current_goal_pub_.publish(goal.getGoal()->target_pose);

    // make sure no threads are running
    ROS_ASSERT(!running_);
    ROS_ASSERT(!path_planner_thread_);
    ROS_ASSERT(!trajectory_planner_thread_);
    ROS_ASSERT(!controller_thread_);
    ROS_ASSERT(layered_map_->map());

    controller_done_ = false;
    running_ = true;

    // start the threads
    path_planner_thread_.reset(new std::thread(&Autonomy::pathPlannerThread, this,
                                               convert(goal.getGoal()->target_pose.pose),
                                               goal.getGoal()->target_pose.header.frame_id));
    trajectory_planner_thread_.reset(new std::thread(&Autonomy::trajectoryPlannerThread, this));
    controller_thread_.reset(new std::thread(&Autonomy::controllerThread, this));

    // wait till all threads are done
    while (running_)
    {
        if (goal.getGoalStatus().status == actionlib_msgs::GoalStatus::PREEMPTED)
        {
            goal.setCanceled();
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
    {
        std::unique_lock<std::mutex> lock(goal_mutex_, std::try_to_lock);
        if (!lock.owns_lock())
        {
            // A Goal is already running... preempt
            if (goal_ && goal_->getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
            {
                ROS_INFO("Goal already executing. Cancelling...");
                goal_->setCanceled();
            }
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
            goal.setCanceled();
        }
    }
}

std::unique_ptr<Eigen::Isometry2d> Autonomy::transformGoal(const Eigen::Isometry2d& goal, const std::string& frame_id)
{
    try
    {
        const geometry_msgs::TransformStamped tr = tf_buffer_->lookupTransform(global_frame_, frame_id, ros::Time(0));

        if (tr.header.stamp > ros::Time(0) &&
            std::abs((tr.header.stamp - ros::Time::now()).toSec()) > localisation_timeout_)
        {
            ROS_WARN_STREAM_THROTTLE(1, "Robot is not localised: transform (" << global_frame_ << "->" << frame_id
                                                                              << ") is stale");
            return nullptr;
        }

        const auto transform = convert(tr.transform);
        return std::make_unique<Eigen::Isometry2d>(transform * goal);
    }
    catch (const tf2::TransformException&)
    {
        ROS_WARN_STREAM_THROTTLE(1, "Robot is not localised: transform (" << global_frame_ << "->" << frame_id
                                                                          << ") is missing");
    }

    return nullptr;
}

void Autonomy::pathPlannerThread(const Eigen::Isometry2d& goal, const std::string& frame_id)
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

        if (!rs.localised)
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            current_path_.reset();

            // publish
            nav_msgs::Path gui_path;
            gui_path.header.frame_id = global_frame_;
            path_pub_.publish(gui_path);

            rate.sleep();

            continue;
        }

        const Eigen::Isometry2d global_robot_pose = rs.map_to_odom * rs.robot_state.pose;
        const Eigen::Isometry2d robot_pose = rs.map_to_odom * rs.robot_state.pose;

        {
            const auto t0 = std::chrono::steady_clock::now();
            layered_map_->clearRadius(robot_pose.translation(), clear_radius_);
            const bool success = layered_map_->update();
            ROS_INFO_STREAM("global map update took " << std::chrono::duration_cast<std::chrono::duration<double>>(
                                                             std::chrono::steady_clock::now() - t0)
                                                             .count());

            if (!success)
            {
                ROS_ERROR("PathPlannerThread: Failed to update layered map");

                std::lock_guard<std::mutex> lock(path_mutex_);
                current_path_.reset();

                // publish
                nav_msgs::Path gui_path;
                gui_path.header.frame_id = global_frame_;
                path_pub_.publish(gui_path);

                rate.sleep();

                continue;
            }
        }

        const auto transformed_goal = transformGoal(goal, frame_id);
        if (!transformed_goal)
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            current_path_.reset();

            // publish
            nav_msgs::Path gui_path;
            gui_path.header.frame_id = global_frame_;
            path_pub_.publish(gui_path);

            rate.sleep();

            continue;
        }

        navigation_interface::PathPlanner::Result result;
        const auto now = ros::SteadyTime::now();
        {
            const auto t0 = std::chrono::steady_clock::now();
            result = path_planner_->plan(global_robot_pose, *transformed_goal);
            ROS_INFO_STREAM("Path Planner took " << std::chrono::duration_cast<std::chrono::duration<double>>(
                                                        std::chrono::steady_clock::now() - t0)
                                                        .count());
        }

        if (result.outcome == navigation_interface::PathPlanner::Outcome::SUCCESSFUL)
        {
            bool update = false;

            std::lock_guard<std::mutex> lock(path_mutex_);
            if (current_path_)
            {
                // trim the current path to the robot pose
                navigation_interface::Path path = current_path_->path;
                if (!path.nodes.empty())
                {
                    const auto closest = path.closestSegment(global_robot_pose);
                    if (closest.first != 0)
                        path.nodes.erase(path.nodes.begin(), path.nodes.begin() + static_cast<long>(closest.first));
                    path.nodes.insert(path.nodes.begin(), global_robot_pose);
                }

                // get the current path cost
                const double cost = path_planner_->cost(path);
                if (cost < std::numeric_limits<double>::max())
                {
                    current_path_->last_successful_time = now;
                    current_path_->last_successful_cost = cost;
                }

                const double time_since_successful_recalc = (now - current_path_->last_successful_time).toSec();

                // if transformed goal has changed (gt res) then force a path swap
                const double allowed_translation = layered_map_->map()->grid.dimensions().resolution() * 4;
                const double allowed_rotation = 0.1;
                const double goal_translation =
                    (transformed_goal->translation() - current_path_->goal.translation()).norm();
                const double goal_rotation =
                    std::abs(Eigen::Rotation2Dd(transformed_goal->linear().inverse() * current_path_->goal.linear())
                                 .smallestAngle());
                const bool goal_moved = goal_translation > allowed_translation || goal_rotation > allowed_rotation;

                if (goal_moved)
                {
                    ROS_INFO_STREAM("GOAL MOVED: new path cost: " << result.cost << " old path cost: " << cost);
                    update = true;
                }
                else if (time_since_successful_recalc > path_persistence_time_ && result.path.length() > 1.0 &&
                         result.cost < path_swap_fraction_ * cost)
                {
                    ROS_INFO_STREAM("NEW PATH: new path cost: " << result.cost << " old path cost: " << cost);
                    update = true;
                }
                else
                {
                    ROS_INFO_STREAM("EXISTING PATH: new path cost: " << result.cost << " old path cost: " << cost);
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
                current_path_.reset(
                    new TrackingPath{*transformed_goal, now, result.cost, now, result.cost, result.path});

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

        // copy the current robot state
        RobotState rs;
        {
            std::lock_guard<std::mutex> lock(robot_state_mutex_);
            rs = robot_state_;
        }

        if (!rs.localised)
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
        const Eigen::Isometry2d robot_pose = rs.map_to_odom * rs.robot_state.pose;
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
            layered_map_->clearRadius(robot_pose.translation(), clear_radius_);
            const bool success = layered_map_->update(roi);

            if (!success)
            {
                ROS_ERROR("TrajectoryPlannerThread: Failed to update layered map");

                std::lock_guard<std::mutex> lock(trajectory_mutex_);
                current_trajectory_.reset();

                // publish
                nav_msgs::Path gui_path;
                gui_path.header.frame_id = "odom";
                trajectory_pub_.publish(gui_path);

                rate.sleep();

                continue;
            }
        }

        {
            nav_msgs::OccupancyGrid grid = layered_map_->map()->grid.toMsg(roi);
            grid.header.frame_id = "map";
            grid.header.stamp = ros::Time::now();
            costmap_publisher_.publish(grid);
        }

        navigation_interface::TrajectoryPlanner::Result result;
        {
            const auto t0 = std::chrono::steady_clock::now();

            // optimise path
            result = trajectory_planner_->plan(roi, rs.robot_state, rs.map_to_odom);

            ROS_INFO_STREAM("Trajectory Planner took " << std::chrono::duration_cast<std::chrono::duration<double>>(
                                                              std::chrono::steady_clock::now() - t0)
                                                              .count());
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
    ros::SteadyTime last_odom_time;

    ROS_ASSERT(layered_map_);
    const int size_x = static_cast<int>(2.0 / layered_map_->map()->grid.dimensions().resolution());
    const int size_y = static_cast<int>(2.0 / layered_map_->map()->grid.dimensions().resolution());

    while (running_)
    {
        // wait for a new odom (or timeout)
        RobotState rs;
        {
            std::unique_lock<std::mutex> lock(robot_state_mutex_);
            if (robot_state_.time == last_odom_time)
            {
                const auto t0 = std::chrono::steady_clock::now();
                if (robot_state_conditional_.wait_for(lock, std::chrono::milliseconds(period_ms)) ==
                    std::cv_status::timeout)
                {
                    const double wait_time =
                        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0)
                            .count();
                    ROS_WARN_STREAM("Did not receive an odom message at the desired frequency: last: "
                                    << robot_state_.time << " waited: " << wait_time);
                    vel_pub_.publish(geometry_msgs::Twist());
                    continue;
                }
            }
            rs = robot_state_;
            last_odom_time = robot_state_.time;
        }

        // check if a new trajectory is available
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        if (current_trajectory_ && rs.localised)
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
        const Eigen::Isometry2d robot_pose = rs.map_to_odom * rs.robot_state.pose;
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
            const bool success = layered_map_->update(roi);

            if (!success)
            {
                ROS_ERROR("ControllerThread: Failed to update layered map");
                vel_pub_.publish(geometry_msgs::Twist());
                continue;
            }
        }

        // Update feedback to correspond to our current position
        const Eigen::Isometry2d global_robot_pose = rs.map_to_odom * rs.robot_state.pose;
        const Eigen::Quaterniond qt = Eigen::Quaterniond(
            Eigen::AngleAxisd(Eigen::Rotation2Dd(global_robot_pose.linear()).angle(), Eigen::Vector3d::UnitZ()));
        autonomy::DriveFeedback feedback;
        feedback.base_position.header.frame_id = global_frame_;
        feedback.base_position.pose.position.x = global_robot_pose.translation().x();
        feedback.base_position.pose.position.y = global_robot_pose.translation().y();
        feedback.base_position.pose.orientation.w = qt.w();
        feedback.base_position.pose.orientation.x = qt.x();
        feedback.base_position.pose.orientation.y = qt.y();
        feedback.base_position.pose.orientation.z = qt.z();
        ROS_ASSERT(goal_);
        goal_->publishFeedback(feedback);

        // control
        const auto result = controller_->control(rs.time, rs.robot_state, rs.map_to_odom);

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
        if (result.outcome == navigation_interface::Controller::Outcome::COMPLETE &&
            current_trajectory_->goal_trajectory)
        {
            ROS_INFO("Final trajectory complete");
            break;
        }
    }

    geometry_msgs::Twist cmd_vel;
    vel_pub_.publish(cmd_vel);

    controller_done_ = true;
}

void Autonomy::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    RobotState robot_state;
    robot_state.time = ros::SteadyTime::now();

    robot_state.robot_state.pose = convert(msg->pose.pose);
    robot_state.robot_state.velocity =
        Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.angular.z);

    try
    {
        const geometry_msgs::TransformStamped tr = tf_buffer_->lookupTransform(global_frame_, "odom", ros::Time(0));
        robot_state.map_to_odom = convert(tr.transform);
        robot_state.localised = true;

        if (tr.header.stamp > ros::Time(0) &&
            std::abs((msg->header.stamp - tr.header.stamp).toSec()) > localisation_timeout_)
        {
            ROS_WARN_STREAM_THROTTLE(1, "Robot is not localised: transform ("
                                            << global_frame_ << "->odom) is stale: "
                                            << (msg->header.stamp - tr.header.stamp).toSec() << "s");
            robot_state.localised = false;
        }
    }
    catch (const tf2::TransformException&)
    {
        ROS_WARN_STREAM_THROTTLE(1, "Robot is not localised: transform (" << global_frame_ << "->odom) is missing");
        robot_state.localised = false;
    }

    {
        std::lock_guard<std::mutex> lock(robot_state_mutex_);
        robot_state_ = robot_state;
    }
    robot_state_conditional_.notify_all();
}
}  // namespace Autonomy
