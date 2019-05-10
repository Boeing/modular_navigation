#include <algorithm>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/layered_costmap.h>

#include <cstdio>
#include <string>
#include <vector>
#include <chrono>

#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace costmap_2d
{

Costmap2DROS::Costmap2DROS(const std::string& name, tf2_ros::Buffer& tf)
    : layered_costmap_(nullptr), name_(name), tf_(tf), transform_tolerance_(0.3), map_update_thread_shutdown_(false),
      stop_updates_(false), initialized_(true), stopped_(false), robot_stopped_(false), map_update_thread_(nullptr),
      last_publish_(0), plugin_loader_("costmap_2d", "costmap_2d::Layer"), publisher_(nullptr)
{
    // Initialize old pose with something
    tf2::toMsg(tf2::Transform::getIdentity(), old_pose_.pose);

    ros::NodeHandle private_nh("~/" + name);

    // get two frames
    private_nh.param("global_frame", global_frame_, std::string("map"));
    private_nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));

    ros::Time last_error = ros::Time::now();
    std::string tf_error;

    // we need to make sure that the transform between the robot base frame and the global frame is available
    while (ros::ok() && !tf_.canTransform(global_frame_, robot_base_frame_, ros::Time(), ros::Duration(0.1), &tf_error))
    {
        ros::spinOnce();
        if (last_error + ros::Duration(5.0) < ros::Time::now())
        {
            ROS_WARN("Timed out waiting for transform from %s to %s to become available before running costmap, tf "
                     "error: %s",
                     robot_base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());
            last_error = ros::Time::now();
        }
        // The error string will accumulate and errors will typically be the same, so the last
        // will do for the warning above. Reset the string here to avoid accumulation.
        tf_error.clear();
    }

    // check if we want a rolling window version of the costmap
    bool rolling_window;
    bool track_unknown_space;
    bool always_send_full_costmap;
    private_nh.param("rolling_window", rolling_window, false);
    private_nh.param("track_unknown_space", track_unknown_space, false);
    private_nh.param("always_send_full_costmap", always_send_full_costmap, false);

    const double width = private_nh.param("width", 20.0);
    const double height = private_nh.param("height", 20.0);
    const double resolution = private_nh.param("resolution", 0.02);
    const double origin_x = private_nh.param("origin_x", -10.0);
    const double origin_y = private_nh.param("origin_y", -10.0);

    layered_costmap_ = std::make_shared<LayeredCostmap>(global_frame_, rolling_window, track_unknown_space);

    layered_costmap_->resizeMap((unsigned int)(width / resolution),
                                (unsigned int)(height / resolution), resolution, origin_x, origin_y);

    if (private_nh.hasParam("plugins"))
    {
        XmlRpc::XmlRpcValue my_list;
        private_nh.getParam("plugins", my_list);
        for (int32_t i = 0; i < my_list.size(); ++i)
        {
            std::string pname = static_cast<std::string>(my_list[i]["name"]);
            std::string type = static_cast<std::string>(my_list[i]["type"]);
            ROS_INFO("Using plugin \"%s\"", pname.c_str());

            boost::shared_ptr<Layer> plugin = plugin_loader_.createInstance(type);
            layered_costmap_->addPlugin(plugin);
            plugin->initialize(layered_costmap_.get(), name + "/" + pname, &tf_);
        }
    }

    publisher_ = std::make_shared<Costmap2DPublisher>(&private_nh, *layered_costmap_->getCostmap(), global_frame_,
                                                      "costmap", always_send_full_costmap);

    stop_updates_ = false;
    initialized_ = true;
    stopped_ = false;

//    robot_stopped_ = false;
//    timer_ = private_nh.createTimer(ros::Duration(.1), &Costmap2DROS::movementCB, this);

    const double map_update_frequency = private_nh.param("update_frequency", 10.0);
    const double map_publish_frequency = private_nh.param("publish_frequency", 1.0);

    if (map_publish_frequency > 0)
        publish_cycle = ros::Duration(1 / map_publish_frequency);
    else
        publish_cycle = ros::Duration(-1);

    map_update_thread_ = std::make_shared<std::thread>(boost::bind(&Costmap2DROS::mapUpdateLoop, this, map_update_frequency));
}

Costmap2DROS::~Costmap2DROS()
{
    map_update_thread_shutdown_ = true;
    if (map_update_thread_)
    {
        map_update_thread_->join();
        map_update_thread_.reset();
    }
}

/*
void Costmap2DROS::reconfigureCB(costmap_2d::Costmap2DConfig& config, uint32_t)
{
    transform_tolerance_ = config.transform_tolerance;
    if (map_update_thread_ != nullptr)
    {
        map_update_thread_shutdown_ = true;
        map_update_thread_->join();
        map_update_thread_.reset();
    }
    map_update_thread_shutdown_ = false;
    double map_update_frequency = config.update_frequency;

    double map_publish_frequency = config.publish_frequency;
    if (map_publish_frequency > 0)
        publish_cycle = ros::Duration(1 / map_publish_frequency);
    else
        publish_cycle = ros::Duration(-1);

    // find size parameters
    double map_width_meters = config.width, map_height_meters = config.height, resolution = config.resolution,
           origin_x = config.origin_x, origin_y = config.origin_y;

    if (!layered_costmap_->isSizeLocked())
    {
        layered_costmap_->resizeMap((unsigned int)(map_width_meters / resolution),
                                    (unsigned int)(map_height_meters / resolution), resolution, origin_x, origin_y);
    }

    old_config_ = config;

    map_update_thread_ =
        std::make_shared<std::thread>(boost::bind(&Costmap2DROS::mapUpdateLoop, this, map_update_frequency));
}

*/

/*
void Costmap2DROS::movementCB(const ros::TimerEvent&)
{
    // don't allow configuration to happen while this check occurs
    // boost::recursive_mutex::scoped_lock mcl(configuration_mutex_);

    geometry_msgs::PoseStamped new_pose;

    if (!getRobotPose(new_pose))
    {
        ROS_WARN_THROTTLE(1.0, "Could not get robot pose, cancelling reconfiguration");
        robot_stopped_ = false;
    }
    // make sure that the robot is not moving
    else
    {
        old_pose_ = new_pose;

        robot_stopped_ = (tf2::Vector3(old_pose_.pose.position.x, old_pose_.pose.position.y, old_pose_.pose.position.z)
                              .distance(tf2::Vector3(new_pose.pose.position.x, new_pose.pose.position.y,
                                                     new_pose.pose.position.z)) < 1e-3) &&
                         (tf2::Quaternion(old_pose_.pose.orientation.x, old_pose_.pose.orientation.y,
                                          old_pose_.pose.orientation.z, old_pose_.pose.orientation.w)
                              .angle(tf2::Quaternion(new_pose.pose.orientation.x, new_pose.pose.orientation.y,
                                                     new_pose.pose.orientation.z, new_pose.pose.orientation.w)) < 1e-3);
    }
}

*/

void Costmap2DROS::mapUpdateLoop(double frequency)
{
    // the user might not want to run the loop every cycle
    if (frequency == 0.0)
        return;

    ros::NodeHandle nh;
    ros::Rate r(frequency);
    while (nh.ok() && !map_update_thread_shutdown_)
    {
        updateMap();

        if (publish_cycle.toSec() > 0 && layered_costmap_->isInitialized())
        {
            unsigned int x0, y0, xn, yn;
            layered_costmap_->getBounds(&x0, &xn, &y0, &yn);
            publisher_->updateBounds(x0, xn, y0, yn);

            ros::Time now = ros::Time::now();
            if (last_publish_ + publish_cycle < now)
            {
                publisher_->publishCostmap();
                last_publish_ = now;
            }
        }

        r.sleep();

        // make sure to sleep for the remainder of our cycle time
        if (r.cycleTime() > ros::Duration(1 / frequency))
            ROS_WARN("Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds",
                     frequency, r.cycleTime().toSec());
    }
}

void Costmap2DROS::updateMap()
{
    if (!stop_updates_)
    {
        // get global pose
        geometry_msgs::PoseStamped pose;
        if (!getRobotPose(pose))
        {
            ROS_WARN_THROTTLE(1.0, "Could not get robot pose, cancelling map update");
        }
        else
        {
            const double x = pose.pose.position.x;
            const double y = pose.pose.position.y;
            const double yaw = tf2::getYaw(pose.pose.orientation);

            layered_costmap_->updateMap(x, y, yaw);

            initialized_ = true;
        }
    }
}

void Costmap2DROS::start()
{
    std::vector<boost::shared_ptr<Layer>>* plugins = layered_costmap_->getPlugins();
    // check if we're stopped or just paused
    if (stopped_)
    {
        // if we're stopped we need to re-subscribe to topics
        for (std::vector<boost::shared_ptr<Layer>>::iterator plugin = plugins->begin(); plugin != plugins->end(); ++plugin)
        {
            (*plugin)->activate();
        }
        stopped_ = false;
    }
    stop_updates_ = false;

    // block until the costmap is re-initialized.. meaning one update cycle has run
    ros::Rate r(100.0);
    while (ros::ok() && !initialized_)
        r.sleep();
}

// cppcheck-suppress unusedFunction
void Costmap2DROS::stop()
{
    stop_updates_ = true;
    std::vector<boost::shared_ptr<Layer>>* plugins = layered_costmap_->getPlugins();
    // unsubscribe from topics
    for (std::vector<boost::shared_ptr<Layer>>::iterator plugin = plugins->begin(); plugin != plugins->end(); ++plugin)
    {
        (*plugin)->deactivate();
    }
    initialized_ = false;
    stopped_ = true;
}

// cppcheck-suppress unusedFunction
void Costmap2DROS::pause()
{
    stop_updates_ = true;
    initialized_ = false;
}

// cppcheck-suppress unusedFunction
void Costmap2DROS::resume()
{
    stop_updates_ = false;

    // block until the costmap is re-initialized.. meaning one update cycle has run
    ros::Rate r(100.0);
    while (!initialized_)
        r.sleep();
}

// cppcheck-suppress unusedFunction
void Costmap2DROS::resetLayers()
{
    const auto top = layered_costmap_->getCostmap();
    top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());
    std::vector<boost::shared_ptr<Layer>>* plugins = layered_costmap_->getPlugins();
    for (std::vector<boost::shared_ptr<Layer>>::iterator plugin = plugins->begin(); plugin != plugins->end(); ++plugin)
    {
        (*plugin)->reset();
    }
}

bool Costmap2DROS::getRobotPose(geometry_msgs::PoseStamped& global_pose) const
{
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = ros::Time();
    ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

    // get the global pose of the robot
    try
    {
        tf_.transform(robot_pose, global_pose, global_frame_);
    }
    catch (tf2::LookupException& ex)
    {
        ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
        ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
        ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    // check global_pose timeout
    if (current_time.toSec() - global_pose.header.stamp.toSec() > transform_tolerance_)
    {
        ROS_WARN_THROTTLE(
            1.0, "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
            current_time.toSec(), global_pose.header.stamp.toSec(), transform_tolerance_);
        return false;
    }

    return true;
}
}
