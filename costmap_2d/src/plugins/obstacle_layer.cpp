#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/plugins/obstacle_layer.h>

#include <tf2_ros/message_filter.h>

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
PLUGINLIB_EXPORT_CLASS(costmap_2d::ObstacleLayer, costmap_2d::Layer)

using costmap_2d::Observation;
using costmap_2d::ObservationBuffer;

namespace costmap_2d
{

ObstacleLayer::ObstacleLayer()
    : footprint_clearing_enabled_(false), max_obstacle_height_(0), marking_(true), clearing_(false),
      rolling_window_(false), dsrv_(nullptr), combination_method_(0), sub_sample_(3)
{
    Costmap2D::costmap_ = nullptr;  // this is the unsigned char* member of parent class Costmap2D.
}

void ObstacleLayer::onInitialize()
{
    ros::NodeHandle nh("~/" + name_), g_nh;
    rolling_window_ = layered_costmap_->isRolling();

    bool track_unknown_space;
    nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
    if (track_unknown_space)
        default_value_ = NO_INFORMATION;
    else
        default_value_ = FREE_SPACE;

    ObstacleLayer::matchSize();
    current_ = true;

    global_frame_ = layered_costmap_->getGlobalFrameID();
    double transform_tolerance;
    nh.param("transform_tolerance", transform_tolerance, 0.2);

    std::string topics_string;
    // get the topics that we'll subscribe to from the parameter server
    nh.param("observation_sources", topics_string, std::string(""));
    ROS_INFO("    Subscribed to Topics: %s", topics_string.c_str());

    // now we need to split the topics based on whitespace which we can use a stringstream for
    std::stringstream ss(topics_string);

    std::string source;
    while (ss >> source)
    {
        ros::NodeHandle source_node(nh, source);

        // get the parameters for the specific topic
        double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
        std::string topic, sensor_frame, data_type;
        bool inf_is_valid, clearing, marking;
        int sub_sample_in;

        source_node.param("topic", topic, source);
        source_node.param("sensor_frame", sensor_frame, std::string(""));
        source_node.param("observation_persistence", observation_keep_time, 0.0);
        source_node.param("expected_update_rate", expected_update_rate, 0.0);
        source_node.param("data_type", data_type, std::string("PointCloud"));
        source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
        source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
        source_node.param("inf_is_valid", inf_is_valid, false);
        source_node.param("clearing", clearing, false);
        source_node.param("marking", marking, true);
        source_node.param("sub_sample", sub_sample_in, 2);

        assert(sub_sample_in > 0);
        sub_sample_ = (unsigned int)sub_sample_in;

        subsample_counter_map_[topic] = 0;

        if (!(data_type == "PointCloud2" || data_type == "PointCloud" || data_type == "LaserScan"))
        {
            ROS_FATAL("Only topics that use point clouds or laser scans are currently supported");
            throw std::runtime_error("Only topics that use point clouds or laser scans are currently supported");
        }

        std::string raytrace_range_param_name, obstacle_range_param_name;

        // get the obstacle range for the sensor
        double obstacle_range = 2.5;
        if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
        {
            source_node.getParam(obstacle_range_param_name, obstacle_range);
        }

        // get the raytrace range for the sensor
        double raytrace_range = 3.0;
        if (source_node.searchParam("raytrace_range", raytrace_range_param_name))
        {
            source_node.getParam(raytrace_range_param_name, raytrace_range);
        }

        clearing_ = clearing;
        marking_ = marking;

        ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
                  sensor_frame.c_str());

        // create an observation buffer
        observation_buffers_.push_back(boost::shared_ptr<ObservationBuffer>(new ObservationBuffer(
            topic, observation_keep_time, expected_update_rate * sub_sample_, min_obstacle_height, max_obstacle_height,
            obstacle_range, raytrace_range, *tf_, global_frame_, sensor_frame, transform_tolerance)));

        ROS_DEBUG("Created an observation buffer for source %s, topic %s, global frame: %s, "
                  "expected update rate: %.2f, observation persistence: %.2f",
                  source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);

        // create a callback for the topic
        if (data_type == "LaserScan")
        {
            boost::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> sub(
                new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));

            boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> filter(
                new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, global_frame_, 50, g_nh));

            if (inf_is_valid)
            {
                filter->registerCallback(
                    boost::bind(&ObstacleLayer::laserScanValidInfCallback, this, _1, observation_buffers_.back()));
            }
            else
            {
                filter->registerCallback(
                    boost::bind(&ObstacleLayer::laserScanCallback, this, _1, observation_buffers_.back()));
            }

            observation_subscribers_.push_back(sub);
            observation_notifiers_.push_back(filter);

            observation_notifiers_.back()->setTolerance(ros::Duration(0.05));
        }
        else if (data_type == "PointCloud")
        {
            boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud>> sub(
                new message_filters::Subscriber<sensor_msgs::PointCloud>(g_nh, topic, 50));

            if (inf_is_valid)
            {
                ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
            }

            boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud>> filter(
                new tf2_ros::MessageFilter<sensor_msgs::PointCloud>(*sub, *tf_, global_frame_, 50, g_nh));
            filter->registerCallback(
                boost::bind(&ObstacleLayer::pointCloudCallback, this, _1, observation_buffers_.back()));

            observation_subscribers_.push_back(sub);
            observation_notifiers_.push_back(filter);
        }
        else
        {
            boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub(
                new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));

            if (inf_is_valid)
            {
                ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
            }

            boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> filter(
                new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_, global_frame_, 50, g_nh));
            filter->registerCallback(
                boost::bind(&ObstacleLayer::pointCloud2Callback, this, _1, observation_buffers_.back()));

            observation_subscribers_.push_back(sub);
            observation_notifiers_.push_back(filter);
        }

        if (sensor_frame != "")
        {
            std::vector<std::string> target_frames;
            target_frames.push_back(global_frame_);
            target_frames.push_back(sensor_frame);
            observation_notifiers_.back()->setTargetFrames(target_frames);
        }
    }

    dsrv_ = NULL;
    setupDynamicReconfigure(nh);
}

void ObstacleLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{
    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>::CallbackType cb =
        boost::bind(&ObstacleLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

ObstacleLayer::~ObstacleLayer()
{
    if (dsrv_)
        delete dsrv_;
}
void ObstacleLayer::reconfigureCB(costmap_2d::ObstaclePluginConfig& config, uint32_t)
{
    enabled_ = config.enabled;
    footprint_clearing_enabled_ = config.footprint_clearing_enabled;
    max_obstacle_height_ = config.max_obstacle_height;
    combination_method_ = config.combination_method;
}

void ObstacleLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                                      const boost::shared_ptr<ObservationBuffer>& buffer)
{
    auto topic = message->header.frame_id;
    if (subsample_counter_map_[topic] > sub_sample_)
    {
        // project the laser into a point cloud
        sensor_msgs::PointCloud2 cloud;
        cloud.header = message->header;

        // project the scan into a point cloud
        try
        {
            projector_.transformLaserScanToPointCloud(message->header.frame_id, *message, cloud, *tf_);
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s",
                     global_frame_.c_str(), ex.what());
            projector_.projectLaser(*message, cloud);
        }

        // buffer the point cloud
        buffer->lock();
        buffer->bufferCloud(cloud);
        buffer->unlock();

        subsample_counter_map_[topic] = 0;
    }
    else
        ++subsample_counter_map_[topic];
}

void ObstacleLayer::laserScanValidInfCallback(const sensor_msgs::LaserScanConstPtr& raw_message,
                                              const boost::shared_ptr<ObservationBuffer>& buffer)
{
    // Filter positive infinities ("Inf"s) to max_range.
    sensor_msgs::LaserScan message = *raw_message;

    auto topic = message.header.frame_id;
    if (subsample_counter_map_[topic] > sub_sample_)
    {
        for (size_t i = 0; i < message.ranges.size(); i++)
        {
            float range = message.ranges[i];
            if (!std::isfinite(range) && range > 0)
            {
                float epsilon = 0.0001;  // a tenth of a millimeter
                message.ranges[i] = message.range_max - epsilon;
            }
        }

        // project the laser into a point cloud
        sensor_msgs::PointCloud2 cloud;
        cloud.header = message.header;

        // project the scan into a point cloud
        try
        {
            projector_.transformLaserScanToPointCloud(message.header.frame_id, message, cloud, *tf_);
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s",
                     global_frame_.c_str(), ex.what());
            projector_.projectLaser(message, cloud);
        }

        // buffer the point cloud
        buffer->lock();
        buffer->bufferCloud(cloud);
        buffer->unlock();
        subsample_counter_map_[topic] = 0;
    }
    else
        ++subsample_counter_map_[topic];
}

void ObstacleLayer::pointCloudCallback(const sensor_msgs::PointCloudConstPtr& message,
                                       const boost::shared_ptr<ObservationBuffer>& buffer)
{
    auto topic = message->header.frame_id;
    if (subsample_counter_map_[topic] > sub_sample_)
    {
        sensor_msgs::PointCloud2 cloud2;

        if (!sensor_msgs::convertPointCloudToPointCloud2(*message, cloud2))
        {
            ROS_ERROR("Failed to convert a PointCloud to a PointCloud2, dropping message");
            return;
        }

        // buffer the point cloud
        buffer->lock();
        buffer->bufferCloud(cloud2);
        buffer->unlock();
        subsample_counter_map_[topic] = 0;
    }
    else
        ++subsample_counter_map_[topic];
}

void ObstacleLayer::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                                        const boost::shared_ptr<ObservationBuffer>& buffer)
{
    auto topic = message->header.frame_id;
    if (subsample_counter_map_[topic] > sub_sample_)
    {
        // buffer the point cloud
        buffer->lock();
        buffer->bufferCloud(*message);
        buffer->unlock();
        subsample_counter_map_[topic] = 0;
    }
    else
        ++subsample_counter_map_[topic];
}

void ObstacleLayer::updateBounds(const double robot_x, const double robot_y, const double robot_yaw, double* min_x,
                                 double* min_y, double* max_x, double* max_y)
{
    if (rolling_window_)
        updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);

    if (!enabled_)
        return;

    useExtraBounds(min_x, min_y, max_x, max_y);

    std::vector<Observation> observations;

    // update the global current status
    current_ = getObservations(observations);

    if (clearing_)
        // raytrace freespace
        for (unsigned int i = 0; i < observations.size(); ++i)
            raytraceFreespace(observations[i], min_x, min_y, max_x, max_y);

    if (marking_)
    {
        // place the new obstacles into a priority queue... each with a priority of zero to begin with
        for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
        {
            const Observation& obs = *it;

            const sensor_msgs::PointCloud2& cloud = *(obs.cloud_);

            const double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

            sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
            {
                const double px = *iter_x;
                const double py = *iter_y;
                const double pz = *iter_z;

                // if the obstacle is too high or too far away from the robot we won't add it
                if (pz > max_obstacle_height_)
                {
                    continue;
                }

                // compute the squared distance from the hitpoint to the pointcloud's origin
                const double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) +
                                       (py - obs.origin_.y) * (py - obs.origin_.y) +
                                       (pz - obs.origin_.z) * (pz - obs.origin_.z);

                // if the point is far enough away... we won't consider it
                if (sq_dist >= sq_obstacle_range)
                {
                    continue;
                }

                // now we need to compute the map coordinates for the observation
                unsigned int mx;
                unsigned int my;
                if (!worldToMap(px, py, mx, my))
                {
                    ROS_DEBUG("Computing map coords failed");
                    continue;
                }

                const unsigned int index = getIndex(mx, my);
                costmap_[index] = LETHAL_OBSTACLE;
                touch(px, py, min_x, min_y, max_x, max_y);
            }
        }
    }

    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void ObstacleLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                    double* max_x, double* max_y)
{
    if (!footprint_clearing_enabled_)
        return;

    transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

    for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
    {
        touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
    }
}

void ObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, unsigned int min_i, unsigned int min_j,
                                unsigned int max_i, unsigned int max_j)
{
    if (!enabled_)
        return;

    if (footprint_clearing_enabled_)
    {
        setConvexPolygonCost(transformed_footprint_, costmap_2d::FREE_SPACE);
    }

    switch (combination_method_)
    {
        case 0:  // Overwrite
            updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
            break;
        case 1:  // Maximum
            updateWithMax(master_grid, min_i, min_j, max_i, max_j);
            break;
        default:  // Nothing
            break;
    }
}

void ObstacleLayer::addStaticObservation(costmap_2d::Observation& obs, bool marking, bool clearing)
{
    if (marking)
        static_marking_observations_.push_back(obs);
    if (clearing)
        static_clearing_observations_.push_back(obs);
}

// cppcheck-suppress unusedFunction
void ObstacleLayer::clearStaticObservations(bool marking, bool clearing)
{
    if (marking)
        static_marking_observations_.clear();
    if (clearing)
        static_clearing_observations_.clear();
}

bool ObstacleLayer::getObservations(std::vector<Observation>& observations) const
{
    bool current = true;
    // get the marking observations
    for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
    {
        observation_buffers_[i]->lock();
        observation_buffers_[i]->getObservations(observations);
        current = observation_buffers_[i]->isCurrent() && current;
        observation_buffers_[i]->unlock();
    }

    // Static observations are used for tests only, maybe this can be removed?
    if (!static_marking_observations_.empty())
        observations.insert(observations.end(), static_marking_observations_.begin(),
                            static_marking_observations_.end());
    if (!static_clearing_observations_.empty())
        observations.insert(observations.end(), static_clearing_observations_.begin(),
                            static_clearing_observations_.end());

    return current;
}

void ObstacleLayer::raytraceFreespace(const Observation& clearing_observation, double* min_x, double* min_y,
                                      double* max_x, double* max_y)
{
    const double ox = clearing_observation.origin_.x;
    const double oy = clearing_observation.origin_.y;
    const sensor_msgs::PointCloud2& cloud = *(clearing_observation.cloud_);

    // get the map coordinates of the origin of the sensor
    unsigned int x0, y0;
    if (!worldToMap(ox, oy, x0, y0))
    {
        ROS_WARN_THROTTLE(
            1.0,
            "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.",
            ox, oy);
    }
    else
    {
        // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
        const double origin_x = origin_x_;
        const double origin_y = origin_y_;
        const double map_end_x = origin_x + size_x_ * resolution_;
        const double map_end_y = origin_y + size_y_ * resolution_;

        touch(ox, oy, min_x, min_y, max_x, max_y);

        // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
        {
            double wx = *iter_x;
            double wy = *iter_y;

            // now we also need to make sure that the enpoint we're raytracing
            // to isn't off the costmap and scale if necessary
            const double a = wx - ox;
            const double b = wy - oy;

            // the minimum value to raytrace from is the origin
            if (wx < origin_x)
            {
                const double t = (origin_x - ox) / a;
                wx = origin_x;
                wy = oy + b * t;
            }
            if (wy < origin_y)
            {
                const double t = (origin_y - oy) / b;
                wx = ox + a * t;
                wy = origin_y;
            }

            // the maximum value to raytrace to is the end of the map
            if (wx > map_end_x)
            {
                const double t = (map_end_x - ox) / a;
                wx = map_end_x - .001;
                wy = oy + b * t;
            }
            if (wy > map_end_y)
            {
                const double t = (map_end_y - oy) / b;
                wx = ox + a * t;
                wy = map_end_y - .001;
            }

            // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
            unsigned int x1, y1;

            // check for legality just in case
            if (!worldToMap(wx, wy, x1, y1))
                continue;

            const unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);
            MarkCell marker(costmap_, FREE_SPACE);

            // and finally... we can execute our trace to clear obstacles along that line
            raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);

            updateRaytraceBounds(ox, oy, wx, wy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);
        }  // End for loop tracing each point and clearing obstacles between origin and point in cloud
    }
}

void ObstacleLayer::activate()
{
    // if we're stopped we need to re-subscribe to topics
    for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
    {
        if (observation_subscribers_[i] != nullptr)
            observation_subscribers_[i]->subscribe();
    }

    for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
    {
        if (observation_buffers_[i])
            observation_buffers_[i]->resetLastUpdated();
    }
}
void ObstacleLayer::deactivate()
{
    for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
    {
        if (observation_subscribers_[i] != nullptr)
            observation_subscribers_[i]->unsubscribe();
    }
}

void ObstacleLayer::updateRaytraceBounds(const double ox, const double oy, const double wx, const double wy,
                                         const double range, double* min_x, double* min_y, double* max_x, double* max_y)
{
    const double dx = wx - ox;
    const double dy = wy - oy;
    const double full_distance = hypot(dx, dy);
    const double scale = std::min(1.0, range / full_distance);
    const double ex = ox + dx * scale;
    const double ey = oy + dy * scale;
    touch(ex, ey, min_x, min_y, max_x, max_y);
}

void ObstacleLayer::reset()
{
    deactivate();
    resetMaps();
    current_ = true;
    activate();
}

}  // namespace costmap_2d
