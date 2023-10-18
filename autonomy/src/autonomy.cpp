#include <autonomy/autonomy.h>
#include <autonomy/math.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <boost/tokenizer.hpp>
#include <navigation_interface/params.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
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
std::shared_ptr<PluginType>
    load(const YAML::Node& parameters, const std::string& planner_name, pluginlib::ClassLoader<PluginType>& loader,
         const std::shared_ptr<const gridmap::MapData>& costmap, const rclcpp::Node::SharedPtr node)
{
    try
    {
        const std::string class_name = parameters[planner_name].as<std::string>();
        std::shared_ptr<PluginType> sp(loader.createUnmanagedInstance(class_name));

        const YAML::Node plugin_params = parameters[loader.getName(class_name)];

        try
        {
            RCLCPP_INFO_STREAM(node->get_logger(), "Loading plugin: " << planner_name << " type: " << class_name);
            sp->initialize(plugin_params, costmap, node);
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
                                                           std::shared_ptr<gridmap::URDFTree> urdf_tree,
                                                           const rclcpp::Node::SharedPtr node)
{
    std::vector<std::shared_ptr<gridmap::Layer>> plugin_ptrs;
    if (parameters["layers"])
    {
        const YAML::Node layers_config = parameters["layers"];
        for (YAML::const_iterator it = layers_config.begin(); it != layers_config.end(); ++it)
        {
            rcpputils::assert_true(it->IsMap());

            std::string pname = (*it)["name"].as<std::string>();
            std::string type = (*it)["type"].as<std::string>();

            try
            {
                RCLCPP_INFO_STREAM(node->get_logger(), "Loading plugin: " << pname << " type: " << type);
                auto plugin_ptr = std::shared_ptr<gridmap::Layer>(loader.createUnmanagedInstance(type));
                plugin_ptr->initialize(pname, parameters[pname], robot_footprint, robot_tracker, urdf_tree, node);
                plugin_ptrs.push_back(plugin_ptr);
            }
            catch (const pluginlib::CreateClassException& e)
            {
                RCLCPP_ERROR_STREAM(node->get_logger(), "CreateClassException while loading plugin '" + pname +
                                                            "': " + std::string(e.what()));
                throw std::runtime_error("Exception while loading plugin '" + pname + "': " + std::string(e.what()));
            }
            catch (const pluginlib::LibraryLoadException& e)
            {
                RCLCPP_ERROR_STREAM(node->get_logger(), "LibraryLoadException while loading plugin '" + pname +
                                                            "': " + std::string(e.what()));
                throw std::runtime_error("Exception while loading plugin '" + pname + "': " + std::string(e.what()));
            }
            catch (const pluginlib::PluginlibException& e)
            {
                RCLCPP_ERROR_STREAM(node->get_logger(), "PluginlibException while loading plugin '" + pname +
                                                            "': " + std::string(e.what()));
                throw std::runtime_error("Exception while loading plugin '" + pname + "': " + std::string(e.what()));
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR_STREAM(node->get_logger(),
                                    "General Exception while loading plugin '" + pname + "': " + std::string(e.what()));
                throw std::runtime_error("Exception while loading plugin '" + pname + "': " + std::string(e.what()));
            }
        }
    }

    return plugin_ptrs;
}

}  // namespace

Autonomy::Autonomy(const std::string& node_name, const rclcpp::NodeOptions& options)
    : rclcpp::Node(node_name, options), layer_loader_("gridmap", "gridmap::Layer"),
      pp_loader_("navigation_interface", "navigation_interface::PathPlanner"),
      tp_loader_("navigation_interface", "navigation_interface::TrajectoryPlanner"),
      c_loader_("navigation_interface", "navigation_interface::Controller"), running_(false),
      execution_thread_running_(false), controller_done_(false), map_updated_(false), current_path_(nullptr),
      current_trajectory_(nullptr), goal_handle_ptr_(nullptr)
{
}

void Autonomy::init()
{
    // Create cb groups for different subscriptions
    callback_group_action_srv_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    umbrella_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    costmap_timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Each of these callback groups is basically a thread
    // Everything assigned to one of them gets bundled into the same thread

    auto umbrella_sub_opt = rclcpp::SubscriptionOptions();
    umbrella_sub_opt.callback_group = umbrella_callback_group_;

    // using namespace std::placeholders;
    const rcl_action_server_options_t& options = rcl_action_server_get_default_options();

    this->action_server_ = rclcpp_action::create_server<autonomy_interface::action::Drive>(
        shared_from_this(), "autonomy",
        std::bind(&Autonomy::goalCallback, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Autonomy::cancelCallback, this, std::placeholders::_1),
        std::bind(&Autonomy::acceptedCallback, this, std::placeholders::_1), options, callback_group_action_srv_);

    RCLCPP_INFO(this->get_logger(), "Starting");

    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    std::string navigation_config = "";
    if (!this->get_parameter("navigation_config", navigation_config))
    {
        throw std::invalid_argument("Could not get navigation_config param");
    }

    YAML::Node root_config;
    try
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Reading navigation config from: " << navigation_config);
        root_config = YAML::LoadFile(navigation_config);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Navigation config is not valid: " << e.what());
        throw;
    }

    max_planning_distance_ = root_config["max_planning_distance"].as<double>(0.0);
    clear_radius_ = root_config["clear_radius"].as<double>(2.0);
    path_planner_frequency_ = root_config["path_planner_frequency"].as<double>(0.5);
    trajectory_planner_frequency_ = root_config["trajectory_planner_frequency"].as<double>(8.0);
    controller_frequency_ = root_config["controller_frequency"].as<double>(10.0);
    path_swap_fraction_ = root_config["path_swap_fraction"].as<double>(0.40);

    const std::vector<Eigen::Vector2d> robot_footprint = navigation_interface::get_point_list(
        root_config, "footprint",
        {{+0.490, +0.000}, {+0.408, -0.408}, {-0.408, -0.408}, {-0.490, +0.000}, {-0.408, +0.408}, {+0.408, +0.408}});

    RCLCPP_INFO_STREAM(this->get_logger(), "max_planning_distance: " << max_planning_distance_);
    RCLCPP_INFO_STREAM(this->get_logger(), "clear_radius: " << clear_radius_);
    RCLCPP_INFO_STREAM(this->get_logger(), "path_planner_frequency: " << path_planner_frequency_);
    RCLCPP_INFO_STREAM(this->get_logger(), "trajectory_planner_frequency: " << trajectory_planner_frequency_);
    RCLCPP_INFO_STREAM(this->get_logger(), "controller_frequency: " << controller_frequency_);
    RCLCPP_INFO_STREAM(this->get_logger(), "path_swap_fraction: " << path_swap_fraction_);

    robot_tracker_.reset(new gridmap::RobotTracker());

    // Get the robot_description from robot_state_publisher node
    auto parameters_client =
        std::make_shared<rclcpp::SyncParametersClient>(shared_from_this(), "robot_state_publisher");
    RCLCPP_INFO(this->get_logger(), "Waiting for robot_state_publisher parameter service...");
    if (!parameters_client->wait_for_service(std::chrono::seconds(30)))
    {
        throw std::runtime_error("Timed out while waiting for robot_description parameter service. Exiting.");
    }
    auto robot_description_param = parameters_client->get_parameters({"robot_description"});
    RCLCPP_INFO(this->get_logger(), "Getting robot_description DONE");

    // Save the URDF with gridmap format
    urdf::Model urdf;
    urdf.initString(robot_description_param[0].value_to_string());
    urdf_tree_.reset(new gridmap::URDFTree(urdf));

    const YAML::Node costmap_config = root_config["costmap"];

    auto layers =
        loadMapLayers(costmap_config, layer_loader_, robot_footprint, robot_tracker_, urdf_tree_, shared_from_this());
    auto base_map_layer = std::make_shared<gridmap::BaseMapLayer>();
    base_map_layer->initialize("base_map", costmap_config["base_map"], robot_footprint, robot_tracker_, urdf_tree_,
                               nullptr);
    layered_map_ = std::make_shared<gridmap::LayeredMap>(base_map_layer, layers);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        std::bind(&Autonomy::odomCallback, this, std::placeholders::_1), umbrella_sub_opt);

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("~/cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)));
    current_goal_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("~/current_goal", rclcpp::QoS(1).transient_local());
    path_goal_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("~/path_goal", rclcpp::QoS(1).transient_local());
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS(0).transient_local());
    trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/trajectory", rclcpp::QoS(0).transient_local());

    planner_map_update_pub_ =
        this->create_publisher<std_msgs::msg::Float64>("~/planner_map_update_time", rclcpp::QoS(0).transient_local());
    trajectory_map_update_pub_ = this->create_publisher<std_msgs::msg::Float64>("~/trajectory_map_update_time",
                                                                                rclcpp::QoS(0).transient_local());
    control_map_update_pub_ =
        this->create_publisher<std_msgs::msg::Float64>("~/control_map_update_time", rclcpp::QoS(0).transient_local());

    // Create the planners
    path_planner_ =
        load<navigation_interface::PathPlanner>(root_config, "path_planner", pp_loader_, nullptr, shared_from_this());
    trajectory_planner_ = load<navigation_interface::TrajectoryPlanner>(root_config, "trajectory_planner", tp_loader_,
                                                                        nullptr, shared_from_this());
    controller_ =
        load<navigation_interface::Controller>(root_config, "controller", c_loader_, nullptr, shared_from_this());

    active_map_sub_ = this->create_subscription<map_manager::msg::MapInfo>(
        "/map_manager/active_map", rclcpp::QoS(1).transient_local(),
        std::bind(&Autonomy::activeMapCallback, this, std::placeholders::_1), umbrella_sub_opt);

    mapper_status_sub_ = this->create_subscription<cartographer_ros_msgs::msg::SystemState>(
        "/mapper/state", rclcpp::QoS(1).transient_local(),
        std::bind(&Autonomy::mapperCallback, this, std::placeholders::_1), umbrella_sub_opt);

    costmap_publisher_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/costmap", rclcpp::QoS(1).transient_local());
    costmap_updates_publisher_ = this->create_publisher<map_msgs::msg::OccupancyGridUpdate>(
        "~/costmap_updates", rclcpp::QoS(1).transient_local());

    get_map_info_client_ = this->create_client<map_manager::srv::GetMapInfo>(
        "/map_manager/get_map_info", rclcpp::ServicesQoS().get_rmw_qos_profile(), srv_callback_group_);

    get_occupancy_grid_client_ = this->create_client<map_manager::srv::GetOccupancyGrid>(
        "/map_manager/get_occupancy_grid", rclcpp::ServicesQoS().get_rmw_qos_profile(), srv_callback_group_);

    get_zones_client_ = this->create_client<map_manager::srv::GetZones>(
        "/map_manager/get_zones", rclcpp::ServicesQoS().get_rmw_qos_profile(), srv_callback_group_);

    costmap_timer_ =
        rclcpp::create_timer(shared_from_this(), this->get_clock(), rclcpp::Duration::from_seconds(1.),
                             std::bind(&Autonomy::updateCostmapCallback, this), costmap_timer_callback_group_);

    RCLCPP_INFO(this->get_logger(), "Successfully started");
}

Autonomy::~Autonomy()
{
    running_ = false;
}

void Autonomy::activeMapCallback(const map_manager::msg::MapInfo& map)
{
    // Block future execution tasks from executing
    {
        std::lock_guard<std::mutex> lock(map_updated_mutex_);
        map_updated_ = false;
    }

    // Cancel current execution and wait for async task to complete
    running_ = false;  // preempt goal execution

    // Wait for async execution to complete
    if (execution_future_.valid())
    {
        execution_future_.wait();
    }

    if (!map.name.empty())
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Received map (" << map.name << ")");

        // Call each service client synchronously
        const std::chrono::seconds timeout = std::chrono::seconds(30);

        auto map_req = std::make_shared<map_manager::srv::GetMapInfo::Request>();
        map_req->map_name = map.name;
        auto map_res_future = get_map_info_client_->async_send_request(map_req);

        auto og_req = std::make_shared<map_manager::srv::GetOccupancyGrid::Request>();
        og_req->map_name = map.name;
        auto og_res_future = get_occupancy_grid_client_->async_send_request(og_req);

        auto zones_req = std::make_shared<map_manager::srv::GetZones::Request>();
        zones_req->map_name = map.name;
        auto zones_res_future = get_zones_client_->async_send_request(zones_req);

        // Requires node executor to be spinning to receive the service response
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Waiting for service: " + std::string(get_map_info_client_->get_service_name()));
        if (map_res_future.wait_for(timeout) != std::future_status::ready)
        {
            throw std::runtime_error("Failed to call: " + std::string(get_map_info_client_->get_service_name()));
        }

        // Requires node executor to be spinning to receive the service response
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Waiting for service: " + std::string(get_occupancy_grid_client_->get_service_name()));
        if (og_res_future.wait_for(timeout) != std::future_status::ready)
        {
            throw std::runtime_error("Failed to call: " + std::string(get_occupancy_grid_client_->get_service_name()));
        }

        // Requires node executor to be spinning to receive the service response
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Waiting for service: " + std::string(get_zones_client_->get_service_name()));
        if (zones_res_future.wait_for(timeout) != std::future_status::ready)
        {
            throw std::runtime_error("Failed to call: " + std::string(get_zones_client_->get_service_name()));
        }

        layered_map_->setMap(map_res_future.get()->map_info, og_res_future.get()->grid, zones_res_future.get()->zones);

        RCLCPP_INFO_STREAM(this->get_logger(), "Set layered map");
    }
    else
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Generating an empty map");

        // generate an empty map
        map_manager::msg::MapInfo map_info;
        map_info.name = "";
        map_info.meta_data.width = 2000;
        map_info.meta_data.height = 2000;
        map_info.meta_data.resolution = 0.02f;
        map_info.meta_data.origin.position.x =
            -static_cast<double>(map_info.meta_data.width) * map_info.meta_data.resolution / 2.0;
        map_info.meta_data.origin.position.y =
            -static_cast<double>(map_info.meta_data.height) * map_info.meta_data.resolution / 2.0;
        nav_msgs::msg::OccupancyGrid map_data;
        map_data.info = map_info.meta_data;
        map_data.data = std::vector<int8_t>(map_info.meta_data.width * map_info.meta_data.height, 0);
        const std::vector<graph_map::msg::Zone> zones;
        layered_map_->setMap(map_info, map_data, zones);
    }

    // Create a separate instance of MapData (which contains the occupancy grid) for each planner
    // This way, updating the map in one planner thread will not slow down the others
    path_planner_map_data_ = std::make_shared<gridmap::MapData>(
        layered_map_->map()->map_info, layered_map_->map()->grid.dimensions(), layered_map_->map()->zones);
    path_planner_->setMapData(path_planner_map_data_);
    layered_map_->update(path_planner_map_data_->grid);

    trajectory_planner_map_data_ = std::make_shared<gridmap::MapData>(
        layered_map_->map()->map_info, layered_map_->map()->grid.dimensions(), layered_map_->map()->zones);
    trajectory_planner_->setMapData(trajectory_planner_map_data_);
    layered_map_->update(trajectory_planner_map_data_->grid);

    controller_map_data_ = std::make_shared<gridmap::MapData>(
        layered_map_->map()->map_info, layered_map_->map()->grid.dimensions(), layered_map_->map()->zones);
    controller_->setMapData(controller_map_data_);
    layered_map_->update(controller_map_data_->grid);

    {
        nav_msgs::msg::OccupancyGrid grid = layered_map_->map()->grid.toMsg();
        grid.header.frame_id = "map";
        grid.header.stamp = this->get_clock()->now();
        costmap_publisher_->publish(grid);
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Finished updating map");

    // Permit any waiting execution task to execute
    {
        std::lock_guard<std::mutex> lock(map_updated_mutex_);
        map_updated_ = true;
    }
    map_updated_conditional_.notify_all();
}

void Autonomy::updateCostmapCallback()
{
    // Skip if execution task is running
    if (execution_future_.valid() && execution_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    {
        return;
    }

    // Update costmap
    if (layered_map_->map())
    {
        const gridmap::RobotState robot_state = robot_tracker_->robotState(this->get_clock());

        if (robot_state.localised)
        {
            layered_map_->update();

            {
                nav_msgs::msg::OccupancyGrid grid = layered_map_->map()->grid.toMsg();
                grid.header.frame_id = "map";
                grid.header.stamp = this->get_clock()->now();  // ros::Time::now();
                costmap_publisher_->publish(grid);
            }
        }
        else
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Robot not localised");
        }
    }
}

void Autonomy::executeGoal()
{
    if (!map_updated_)
    {
        // Wait for map to finish being updated
        RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for map to be updated...");
        std::unique_lock<std::mutex> lock(map_updated_mutex_);
        map_updated_conditional_.wait(lock, [this] { return this->map_updated_; });
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Executing goal: " + rclcpp_action::to_string(goalHandle()->get_goal_id()));

    current_goal_pub_->publish(transformedGoalPose());
    // TODO: add mutex

    // make sure no threads are running
    rcpputils::assert_true(!running_);
    rcpputils::assert_true(!path_planner_thread_);
    rcpputils::assert_true(!trajectory_planner_thread_);
    rcpputils::assert_true(!controller_thread_);
    rcpputils::assert_true(path_planner_map_data_ != nullptr);
    rcpputils::assert_true(trajectory_planner_map_data_ != nullptr);
    rcpputils::assert_true(controller_map_data_ != nullptr);

    controller_done_ = false;
    running_ = true;

    // start the threads
    path_planner_thread_.reset(new std::thread(&Autonomy::pathPlannerThread, this));
    trajectory_planner_thread_.reset(new std::thread(&Autonomy::trajectoryPlannerThread, this));
    controller_thread_.reset(new std::thread(&Autonomy::controllerThread, this));
    // TODO: change to async tasks

    // wait till all threads are done
    std::shared_ptr<GoalHandleDrive> goal_handle = nullptr;

    while (running_)
    {
        goal_handle = goalHandle();  // update goal handle

        if (!goalHandle()->is_active())
        {
            break;  // stop execution
        }

        // Send successful goal result to client if complete
        if (controller_done_)
        {
            auto action_result = std::make_shared<autonomy_interface::action::Drive::Result>();
            action_result->success = true;
            goal_handle->succeed(action_result);  //, "Goal reached");
            break;                                // stop execution
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Send aborted goal result to client if not complete
    if (!controller_done_ && goal_handle)
    {
        auto action_result_aborted = std::make_shared<autonomy_interface::action::Drive::Result>();
        action_result_aborted->success = false;
        goal_handle->abort(action_result_aborted);
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

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Goal " << rclcpp_action::to_string(goalHandle()->get_goal_id()) << " execution complete");

    setGoalHandle(nullptr);
}

rclcpp_action::GoalResponse Autonomy::goalCallback(const rclcpp_action::GoalUUID& uuid,
                                                   std::shared_ptr<const Drive::Goal> goal)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received goal: " + rclcpp_action::to_string(uuid) + " ("
                                               << goal->target_pose.pose.position.x << ", "
                                               << goal->target_pose.pose.position.y << ")");

    RCLCPP_INFO_STREAM(this->get_logger(), "Goal standard deviations: X: " << goal->std_x << ", Y: " << goal->std_y
                                                                           << ", W: " << goal->std_w
                                                                           << ", Max Samples: " << goal->max_samples);

    // Fetch current robot state
    const gridmap::RobotState robot_state = robot_tracker_->robotState(this->get_clock());

    // Check robot is localised in map
    if (!robot_state.localised)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Rejected new goal: " << rclcpp_action::to_string(uuid) << " - Robot not localised");
        return rclcpp_action::GoalResponse::REJECT;
    }

    // Check Euclidean distance to goal does not exceed max_planning_distance_
    if (max_planning_distance_ > 0.0)
    {
        const Eigen::Isometry2d robot_pose = robot_state.map_to_odom * robot_state.odom.pose;
        const Eigen::Vector2d goal_position(goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);
        const double squared_dist_to_goal = (robot_pose.translation() - goal_position).squaredNorm();
        if (squared_dist_to_goal > (max_planning_distance_ * max_planning_distance_))
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Rejected new goal: " << rclcpp_action::to_string(uuid)
                                                                         << " - Goal beyond max planning distance of "
                                                                         << max_planning_distance_ << "m");
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    // Check sampling settings
    if (goal->std_x < 0.0 || goal->std_y < 0.0 || goal->std_w < 0.0)
    {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Rejected new goal: " << rclcpp_action::to_string(uuid) << " -  Bad sample settings");
        return rclcpp_action::GoalResponse::REJECT;
    }

    // Check goal can be transformed to global_frame_, store as transformed_goal_pose_
    if (goal->target_pose.header.frame_id != global_frame_)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Goal is in frame: " << goal->target_pose.header.frame_id << ", not: "
                                                                    << global_frame_ << ", transforming...");

        // Transform into global_frame_
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tfBuffer_->lookupTransform(global_frame_, goal->target_pose.header.frame_id,
                                                   rclcpp::Time(0));  // ros::Time(0)
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(), ex.what());
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Can't modify pose in-place because it's a const, instead store as transformed_goal_pose_
        geometry_msgs::msg::PoseStamped transformed_goal_pose;
        tf2::doTransform(goal->target_pose, transformed_goal_pose, transform);
        setTransformedGoalPose(transformed_goal_pose);
    }
    else
    {
        setTransformedGoalPose(goal->target_pose);
    }

    // Check active map has been set
    if (!(layered_map_->map() && path_planner_map_data_ && trajectory_planner_map_data_ && controller_map_data_))
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Rejected new goal: " << rclcpp_action::to_string(uuid) << " - No Map!");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Accepted new goal: " << rclcpp_action::to_string(uuid));
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Autonomy::cancelCallback(const std::shared_ptr<GoalHandleDrive> goal_handle)
{
    auto current_goal_handle = goalHandle();

    // Check if goal ids match
    if (current_goal_handle && goal_handle->get_goal_id() == current_goal_handle->get_goal_id())
    {
        // Check executing
        if (execution_future_.valid() && running_)
        {
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "Cancelling goal: " << rclcpp_action::to_string(current_goal_handle->get_goal_id()));

            auto feedback = std::make_shared<autonomy_interface::action::Drive::Feedback>();
            feedback->state = autonomy_interface::action::Drive::Feedback::NO_PLAN;
            current_goal_handle->publish_feedback(feedback);

            running_ = false;  // stop execution

            return rclcpp_action::CancelResponse::ACCEPT;
        }
    }
    return rclcpp_action::CancelResponse::REJECT;
}

void Autonomy::acceptedCallback(const std::shared_ptr<GoalHandleDrive> goal_handle)
{
    // Check execution status
    if (execution_future_.valid() && running_)
    {
        // A goal is currently executing...
        auto current_goal_handle = goalHandle();
        if (current_goal_handle->is_active())
        {
            // Set active goal to aborted and update goal_handle_ for autonomy threads
            // Note, this does not actually stop the execution thread. It only updates the goal_handle_, so the
            // planners should seamlessly pick up the new goal
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "Updating goal with: " << rclcpp_action::to_string(goal_handle->get_goal_id()));
            auto action_result_aborted = std::make_shared<autonomy_interface::action::Drive::Result>();
            action_result_aborted->success = false;
            current_goal_handle->abort(action_result_aborted);
            setGoalHandle(goal_handle);  // update goal for threads
            return;
        }
        else
        {
            // A goal is currently finishing up... wait for it to gracefully finish
            execution_future_.wait();  // TODO: consider implications of blocking callback group
        }
    }

    // Update goal_handle_ and execute goal asynchronously
    setGoalHandle(goal_handle);  // update goal for threads
    execution_future_ = std::async(&Autonomy::executeGoal, this);
}

void Autonomy::pathPlannerThread()
{
    rclcpp::WallRate rate(path_planner_frequency_);
    const double path_planner_period_s = 1.0 / path_planner_frequency_;
    auto feedback = std::make_shared<autonomy_interface::action::Drive::Feedback>();
    feedback->state = autonomy_interface::action::Drive::Feedback::NO_PLAN;

    std::string last_goal_id = rclcpp_action::to_string(
        goalHandle()->get_goal_id());  // Keeps track of the previous goal ID to see if has been updated

    rcpputils::assert_true(layered_map_ != nullptr);
    rcpputils::assert_true(path_planner_map_data_ != nullptr);
    // Path planning window. Add a 2m border.
    const int size_x = max_planning_distance_ > 0.0
                           ? static_cast<int>(((2 * max_planning_distance_) + 4.0) /
                                              layered_map_->map()->grid.dimensions().resolution())
                           : layered_map_->map()->grid.dimensions().size().x() * 2.0;
    const int size_y = max_planning_distance_ > 0.0
                           ? static_cast<int>(((2 * max_planning_distance_) + 4.0) /
                                              layered_map_->map()->grid.dimensions().resolution())
                           : layered_map_->map()->grid.dimensions().size().y() * 2.0;

    while (running_)
    {
        auto goal_handle = goalHandle();  // update goal handle

        const gridmap::RobotState robot_state = robot_tracker_->robotState(this->get_clock());

        if (!robot_state.localised)
        {
            current_path_.reset();

            // publish
            nav_msgs::msg::Path gui_path;
            gui_path.header.frame_id = global_frame_;
            path_pub_->publish(gui_path);

            feedback->state = autonomy_interface::action::Drive::Feedback::NO_PLAN;

            {
                std::lock_guard<std::mutex> lock(path_mutex_);
                rcpputils::assert_true(goal_handle->get_goal() != nullptr);
                goal_handle->publish_feedback(feedback);
            }

            RCLCPP_INFO_STREAM(this->get_logger(), "Robot not localised. Unable to plan!");

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

        rcpputils::assert_true((top_left_x + actual_size_x) <= layered_map_->map()->grid.dimensions().size().x());
        rcpputils::assert_true((top_left_y + actual_size_y) <= layered_map_->map()->grid.dimensions().size().y());

        const gridmap::AABB roi{{top_left_x, top_left_y}, {actual_size_x, actual_size_y}};
        {
            bool success = false;
            const auto t0 = this->get_clock()->now();
            if (max_planning_distance_ > 0.0)
            {
                success = layered_map_->update(path_planner_map_data_->grid, roi);
            }
            else
            {
                success = layered_map_->update(path_planner_map_data_->grid);
            }

            const double map_update_duration_s = rclcpp::Duration(this->get_clock()->now() - t0).seconds();
            std_msgs::msg::Float64 map_update_duration_msg;
            map_update_duration_msg.data = map_update_duration_s;
            planner_map_update_pub_->publish(map_update_duration_msg);

            // if (map_update_duration > rate.expectedCycleTime().toSec())
            if (map_update_duration_s > path_planner_period_s)
                RCLCPP_WARN_STREAM(this->get_logger(), "Path Planning MAP UPDATE took too long: "
                                                           << map_update_duration_s
                                                           << "s. Expected: " << path_planner_period_s << "s.");

            if (!success)
            {
                RCLCPP_ERROR(this->get_logger(), "Path Planning map update failed");

                {
                    std::lock_guard<std::mutex> lock(path_mutex_);
                    current_path_.reset();
                }

                // publish
                nav_msgs::msg::Path gui_path;
                gui_path.header.frame_id = global_frame_;
                path_pub_->publish(gui_path);

                feedback->state = autonomy_interface::action::Drive::Feedback::NO_PLAN;
                {
                    rcpputils::assert_true(goal_handle->get_goal() != nullptr);
                    goal_handle->publish_feedback(feedback);
                }

                rate.sleep();

                continue;
            }

            {
                nav_msgs::msg::OccupancyGrid grid = path_planner_map_data_->grid.toMsg();
                grid.header.frame_id = "map";
                grid.header.stamp = this->get_clock()->now();
                costmap_publisher_->publish(grid);
            }
        }
        bool goal_updated = false;
        Eigen::Isometry2d goal_pose;
        navigation_interface::PathPlanner::GoalSampleSettings goal_sample_settings;
        double avoid_distance;
        double backwards_mult;
        double strafe_mult;
        double rotation_mult;
        {
            auto goal = goal_handle->get_goal();
            rcpputils::assert_true(goal != nullptr);

            goal_pose = convert(transformedGoalPose().pose);

            goal_sample_settings.std_x = goal->std_x;
            goal_sample_settings.std_y = goal->std_y;
            goal_sample_settings.std_w = goal->std_w;
            goal_sample_settings.max_samples = goal->max_samples;

            avoid_distance = goal->avoid_distance;

            backwards_mult = goal->backwards_mult;
            strafe_mult = goal->strafe_mult;
            rotation_mult = goal->rotation_mult;

            std::string new_goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
            if (new_goal_id != last_goal_id)
            {
                goal_updated = true;
                RCLCPP_INFO_STREAM(this->get_logger(), "Goal updated from: " << last_goal_id << " to:" << new_goal_id);
            }
            last_goal_id = new_goal_id;
        }

        // Plan
        navigation_interface::PathPlanner::Result result;
        const auto now = rclcpp::Clock(RCL_STEADY_TIME).now();  // ros::SteadyTime::now();
        {

            const auto t0 = this->get_clock()->now();
            result = path_planner_->plan(roi, robot_pose, goal_pose, goal_sample_settings, avoid_distance,
                                         backwards_mult, strafe_mult, rotation_mult);

            const double plan_duration_s = rclcpp::Duration(this->get_clock()->now() - t0).seconds();
            if (plan_duration_s > path_planner_period_s)  // rate.expectedCycleTime().toSec())
                RCLCPP_WARN_STREAM(this->get_logger(), "Path Planning PLAN call took too long: "
                                                           << plan_duration_s
                                                           << "s. Expected: " << path_planner_period_s << "s.");

            RCLCPP_DEBUG_STREAM(this->get_logger(), "Path Planning took: " << plan_duration_s << "s.");
        }

        if (!running_)
            break;

        // Use some heuristics to decide if the new plan should be used or we stick to the old one
        if (result.outcome == navigation_interface::PathPlanner::Outcome::SUCCESSFUL)
        {
            bool update = false;
            rcpputils::assert_true(!result.path.nodes.empty());

            std::lock_guard<std::mutex> lock(path_mutex_);
            if (current_path_ && !goal_updated)
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
                const double cost = path_planner_->cost(path, avoid_distance);
                if (cost < std::numeric_limits<double>::max())
                {
                    current_path_->last_successful_time = now;
                    current_path_->last_successful_cost = cost;
                }

                const double time_since_successful_recalc = (now - current_path_->last_successful_time).seconds();

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

                RCLCPP_INFO_STREAM(this->get_logger(),
                                   "swap: " << update << " old_cost: " << cost << " (" << old_path_possible << ")"
                                            << " new_cost: " << result.cost << " (" << new_path_possible << ")"
                                            << " new_is_better: " << new_path_much_better << " time_invalid: "
                                            << time_since_successful_recalc << "s (" << old_path_expired << ")"
                                            << " dist_to_old: " << distance_to_old_path << "m (" << robot_near_old_path
                                            << ")");
            }
            else if (!current_path_)
            {
                RCLCPP_INFO_STREAM(this->get_logger(), "First path found");
                update = true;
            }
            else
            {
                update = true;
            }

            if (update)
            {
                result.path.id = uuid();
                current_path_.reset(new TrackingPath{goal_pose, now, result.cost, now, result.cost, result.path});

                // publish
                nav_msgs::msg::Path gui_path;
                gui_path.header.frame_id = global_frame_;
                for (const auto& node : result.path.nodes)
                {
                    const Eigen::Quaterniond qt(
                        Eigen::AngleAxisd(Eigen::Rotation2Dd(node.linear()).smallestAngle(), Eigen::Vector3d::UnitZ()));

                    geometry_msgs::msg::PoseStamped p;
                    p.header.frame_id = global_frame_;
                    p.pose.position.x = node.translation().x();
                    p.pose.position.y = node.translation().y();
                    p.pose.orientation.w = qt.w();
                    p.pose.orientation.x = qt.x();
                    p.pose.orientation.y = qt.y();
                    p.pose.orientation.z = qt.z();

                    gui_path.poses.push_back(p);
                }
                path_goal_pub_->publish(gui_path.poses.back());
                path_pub_->publish(gui_path);
            }

            feedback->state = autonomy_interface::action::Drive::Feedback::GOOD_PLAN;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to find a path");

            std::lock_guard<std::mutex> lock(path_mutex_);
            const double time_since_successful_recalc = current_path_
                                                            ? (now - current_path_->last_successful_time).seconds()
                                                            : std::numeric_limits<double>::max();

            // In the situation of persistent
            if (time_since_successful_recalc > 4.0)
            {
                feedback->state = autonomy_interface::action::Drive::Feedback::STUCK;

                current_path_.reset();

                std::lock_guard<std::mutex> t_lock(trajectory_mutex_);
                current_trajectory_.reset();

                RCLCPP_INFO_STREAM(this->get_logger(), "Clearing radius of " << clear_radius_ << "m around robot");
                layered_map_->clearRadius(path_planner_map_data_->grid, robot_pose.translation(), clear_radius_);

                RCLCPP_INFO(this->get_logger(), "Waiting for sensors to stabilise...");
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            }
        }

        // Update feedback to correspond to our current position
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            rcpputils::assert_true(goal_handle->get_goal() != nullptr);
            goal_handle->publish_feedback(feedback);
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
    rclcpp::WallRate rate(trajectory_planner_frequency_);
    const double trajectory_planner_period_s = 1.0 / trajectory_planner_frequency_;

    rcpputils::assert_true(layered_map_ != nullptr);
    rcpputils::assert_true(trajectory_planner_map_data_ != nullptr);
    const int size_x = static_cast<int>(8.0 / layered_map_->map()->grid.dimensions().resolution());
    const int size_y = static_cast<int>(8.0 / layered_map_->map()->grid.dimensions().resolution());

    while (running_)
    {
        auto goal_handle = goalHandle();  // update goal handle

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
            nav_msgs::msg::Path gui_path;
            gui_path.header.frame_id = "odom";
            trajectory_pub_->publish(gui_path);

            rate.sleep();
            continue;
        }

        const gridmap::RobotState robot_state = robot_tracker_->robotState(this->get_clock());

        if (!robot_state.localised)
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            current_trajectory_.reset();

            // publish
            nav_msgs::msg::Path gui_path;
            gui_path.header.frame_id = "odom";
            trajectory_pub_->publish(gui_path);

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

        rcpputils::assert_true((top_left_x + actual_size_x) <= layered_map_->map()->grid.dimensions().size().x());
        rcpputils::assert_true((top_left_y + actual_size_y) <= layered_map_->map()->grid.dimensions().size().y());

        const gridmap::AABB roi{{top_left_x, top_left_y}, {actual_size_x, actual_size_y}};

        {
            const auto t0 = this->get_clock()->now();
            const bool success = layered_map_->update(trajectory_planner_map_data_->grid, roi);

            const double map_update_duration_s = rclcpp::Duration(this->get_clock()->now() - t0).seconds();

            std_msgs::msg::Float64 map_update_duration_msg;
            map_update_duration_msg.data = map_update_duration_s;
            trajectory_map_update_pub_->publish(map_update_duration_msg);

            if (map_update_duration_s > trajectory_planner_period_s)  // rate.expectedCycleTime().toSec())
                RCLCPP_WARN_STREAM(this->get_logger(), "Trajectory Planning MAP UPDATE took too long: "
                                                           << map_update_duration_s
                                                           << "s. Expected: " << trajectory_planner_period_s << "s.");

            if (!success)
            {
                RCLCPP_ERROR(this->get_logger(), "Trajectory Planning map update failed");

                std::lock_guard<std::mutex> lock(trajectory_mutex_);
                current_trajectory_.reset();

                // publish
                nav_msgs::msg::Path gui_path;
                gui_path.header.frame_id = "odom";
                trajectory_pub_->publish(gui_path);

                rate.sleep();

                continue;
            }
        }

        double avoid_distance;
        {
            auto goal = goal_handle->get_goal();
            rcpputils::assert_true(goal != nullptr);

            avoid_distance = goal->avoid_distance;
        }

        navigation_interface::TrajectoryPlanner::Result result;
        {
            const auto t0 = this->get_clock()->now();

            // optimise path
            const auto ks = navigation_interface::KinodynamicState{robot_state.odom.pose, robot_state.odom.velocity, 0};
            result = trajectory_planner_->plan(roi, ks, robot_state.map_to_odom, avoid_distance);

            const double plan_duration_s = rclcpp::Duration(this->get_clock()->now() - t0).seconds();
            if (plan_duration_s > trajectory_planner_period_s)  // rate.expectedCycleTime().toSec())
                RCLCPP_WARN_STREAM(this->get_logger(), "Trajectory Planning PLAN duration took too long: "
                                                           << plan_duration_s
                                                           << "s. Expected: " << trajectory_planner_period_s << "s.");
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
            nav_msgs::msg::Path gui_path;
            gui_path.header.frame_id = "odom";
            for (const auto& state : result.trajectory.states)
            {
                const Eigen::Quaterniond qt(
                    Eigen::AngleAxisd(Eigen::Rotation2Dd(state.pose.linear()).angle(), Eigen::Vector3d::UnitZ()));

                geometry_msgs::msg::PoseStamped p;
                p.header.frame_id = "odom";
                p.pose.position.x = state.pose.translation().x();
                p.pose.position.y = state.pose.translation().y();
                p.pose.orientation.w = qt.w();
                p.pose.orientation.x = qt.x();
                p.pose.orientation.y = qt.y();
                p.pose.orientation.z = qt.z();

                gui_path.poses.push_back(p);
            }
            trajectory_pub_->publish(gui_path);
        }
        // clear the trajectory from the controller
        else
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            current_trajectory_.reset();

            // publish
            nav_msgs::msg::Path gui_path;
            gui_path.header.frame_id = "odom";
            trajectory_pub_->publish(gui_path);
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
    const double controller_period_s = 1.0 / controller_frequency_;
    rclcpp::Time last_odom_time(0);

    rcpputils::assert_true(layered_map_ != nullptr);
    rcpputils::assert_true(controller_map_data_ != nullptr);
    const int size_x = static_cast<int>(5.0 / layered_map_->map()->grid.dimensions().resolution());
    const int size_y = static_cast<int>(5.0 / layered_map_->map()->grid.dimensions().resolution());

    while (running_)
    {
        auto goal_handle = goalHandle();  // update goal handle

        // wait for a new odom (or timeout)
        gridmap::RobotState robot_state = robot_tracker_->robotState(this->get_clock());
        try
        {
            if (robot_state.odom.time.seconds() == last_odom_time.seconds())
                robot_state = robot_tracker_->waitForRobotState(2.0 * controller_period_s * 1000.0, this->get_clock());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), e.what());
            vel_pub_->publish(geometry_msgs::msg::Twist());
        }
        last_odom_time = robot_state.odom.time;

        // check if a new trajectory is available
        {
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
                vel_pub_->publish(geometry_msgs::msg::Twist());
                continue;
            }
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

        rcpputils::assert_true((top_left_x + actual_size_x) <= layered_map_->map()->grid.dimensions().size().x());
        rcpputils::assert_true((top_left_y + actual_size_y) <= layered_map_->map()->grid.dimensions().size().y());

        const gridmap::AABB roi{{top_left_x, top_left_y}, {actual_size_x, actual_size_y}};

        {
            const auto t0 = this->get_clock()->now();

            const bool success = layered_map_->update(controller_map_data_->grid, roi);

            if (!success)
            {
                RCLCPP_ERROR(this->get_logger(), "Control Planning map update failed");
                vel_pub_->publish(geometry_msgs::msg::Twist());
                continue;
            }

            const double map_update_duration_s = rclcpp::Duration(this->get_clock()->now() - t0).seconds();

            std_msgs::msg::Float64 map_update_duration_msg;
            map_update_duration_msg.data = map_update_duration_s;
            control_map_update_pub_->publish(map_update_duration_msg);

            if (map_update_duration_s > controller_period_s)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Control Planning MAP UPDATE took too long: "
                                                            << map_update_duration_s << "s, Expecting update rate of: "
                                                            << controller_period_s << "s.");
                vel_pub_->publish(geometry_msgs::msg::Twist());
                continue;
            }
        }

        Eigen::Vector3d max_velocity;
        double xy_goal_tolerance;
        double yaw_goal_tolerance;
        {
            auto goal = goal_handle->get_goal();
            rcpputils::assert_true(goal != nullptr);

            max_velocity[0] = goal->max_velocity_x;
            max_velocity[1] = goal->max_velocity_y;
            max_velocity[2] = goal->max_velocity_w;

            xy_goal_tolerance = goal->xy_goal_tolerance;
            yaw_goal_tolerance = goal->yaw_goal_tolerance;
        }

        navigation_interface::Controller::Result result;
        {
            const auto t0 = this->get_clock()->now();

            // control
            const auto ks = navigation_interface::KinodynamicState{robot_state.odom.pose, robot_state.odom.velocity, 0};
            const auto t = rclcpp::Clock(RCL_STEADY_TIME).now();  // ros::SteadyTime::now();
            result = controller_->control(t, roi, ks, robot_state.map_to_odom, max_velocity, xy_goal_tolerance,
                                          yaw_goal_tolerance);

            const double control_duration_s = rclcpp::Duration(this->get_clock()->now() - t0).seconds();
            if (control_duration_s > controller_period_s)
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Control Planning CONTROL duration took too long: "
                                                            << control_duration_s << "s, Expecting update rate of: "
                                                            << controller_period_s << "s.");
                vel_pub_->publish(geometry_msgs::msg::Twist());
                continue;
            }
        }

        // send cmd_vel
        geometry_msgs::msg::Twist cmd_vel;
        if (result.outcome == navigation_interface::Controller::Outcome::SUCCESSFUL)
        {
            rcpputils::assert_true(result.command.allFinite());
            cmd_vel.linear.x = result.command.x();
            cmd_vel.linear.y = result.command.y();
            cmd_vel.angular.z = result.command.z();
        }
        vel_pub_->publish(cmd_vel);

        // finish if end of path
        if (result.outcome == navigation_interface::Controller::Outcome::COMPLETE)
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            if (current_trajectory_->goal_trajectory)
            {
                RCLCPP_INFO(this->get_logger(), "Final trajectory complete");
                break;
            }
        }
    }

    geometry_msgs::msg::Twist cmd_vel;
    vel_pub_->publish(cmd_vel);

    controller_done_ = true;
}

void Autonomy::odomCallback(const nav_msgs::msg::Odometry& msg) const
{
    robot_tracker_->addOdometryData(msg);
}

void Autonomy::mapperCallback(const cartographer_ros_msgs::msg::SystemState& msg) const
{
    const bool was_localised = robot_tracker_->localised();
    robot_tracker_->addLocalisationData(msg);
    const bool is_localised = robot_tracker_->localised();

    // reset the layers when localisation switches
    if (was_localised != is_localised)
    {
        if (is_localised)
            RCLCPP_INFO_STREAM(this->get_logger(), "Robot localised!");
        else
            RCLCPP_INFO_STREAM(this->get_logger(), "Robot not localised!");

        if (layered_map_->map())
            layered_map_->clear();
    }
}

}  // namespace autonomy
