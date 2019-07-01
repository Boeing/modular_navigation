/*
#include <gridmap/plugins/obstacle_data.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/message_filter.h>

#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/point_cloud2_iterator.h>

#include <opencv2/highgui.hpp>

#include <chrono>

PLUGINLIB_EXPORT_CLASS(gridmap::ObstacleData, gridmap::DataSource)

namespace gridmap
{

ObstacleData::ObstacleData()
{
}

void ObstacleData::onInitialize()
{
    ros::NodeHandle nh("~/" + name_);
    ros::NodeHandle g_nh;

    const std::string topics_string = nh.param("observation_sources", std::string(""));

    std::stringstream ss(topics_string);
    std::string source;
    while (ss >> source)
    {
        ros::NodeHandle source_node(nh, source);

        const std::string topic = source_node.param("topic", source);
        const std::string data_type = source_node.param("data_type", std::string("PointCloud"));

        const double observation_persistence = source_node.param("observation_persistence", 0.0);
        const double expected_update_rate = source_node.param("expected_update_rate", 0.0);
        const double min_obstacle_height = source_node.param("min_obstacle_height", 0.0);
        const double max_obstacle_height = source_node.param("max_obstacle_height", 2.0);
        const double obstacle_range = source_node.param("obstacle_range", 2.5);
        const double raytrace_range = source_node.param("raytrace_range", 3.0);
        const int sub_sample = source_node.param("sub_sample", 0);
        const bool inf_is_valid = source_node.param("inf_is_valid", false);

        if (data_type == "LaserScan")
        {
            boost::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> sub(
                new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));

            boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> filter(
                new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_buffer_, global_frame_, 50,
                                                                   g_nh));

            boost::shared_ptr<message_filters::SubscriberBase> subscriber(sub);
            boost::shared_ptr<tf2_ros::MessageFilterBase> message_filter(filter);

            auto source = std::make_shared<Sensor>(Sensor{
                subscriber, message_filter, observation_persistence, expected_update_rate, min_obstacle_height,
                max_obstacle_height, obstacle_range, raytrace_range, sub_sample, 0, inf_is_valid});

            filter->registerCallback(boost::bind(&ObstacleData::laserScanCallback, this, _1, source));

            sensors_.push_back(source);
        }
        else if (data_type == "PointCloud2")
        {
            boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub(
                new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));

            boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> filter(
                new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_buffer_, global_frame_,
                                                                     50, g_nh));

            boost::shared_ptr<message_filters::SubscriberBase> subscriber(sub);
            boost::shared_ptr<tf2_ros::MessageFilterBase> message_filter(filter);

            auto source = std::make_shared<Sensor>(Sensor{
                subscriber, message_filter, observation_persistence, expected_update_rate, min_obstacle_height,
                max_obstacle_height, obstacle_range, raytrace_range, sub_sample, 0, inf_is_valid});

            filter->registerCallback(boost::bind(&ObstacleData::pointCloud2Callback, this, _1, source));

            sensors_.push_back(source);
        }
        else
        {
            const std::string msg = "Only topics that use point clouds or laser scans are currently supported";
            ROS_FATAL_STREAM(msg);
            throw std::runtime_error(msg);
        }
    }
}

ObstacleData::~ObstacleData()
{
}

void ObstacleData::laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                                     const std::shared_ptr<Sensor>& sensor)
{
    if (sensor->sub_sample == 0 ||
        (sensor->sub_sample > 0 && sensor->sub_sample_count > sensor->sub_sample))
    {
        ROS_INFO("Processing laser");

        sensor_msgs::PointCloud2 local_cloud;

        if (sensor->inf_is_valid)
        {
            sensor_msgs::LaserScan valid_scan = *message;
            for (size_t i = 0; i < message->ranges.size(); i++)
            {
                const float range = message->ranges[i];
                if (!std::isfinite(range) && range > 0)
                {
                    const float epsilon = 0.0001f;
                    valid_scan.ranges[i] = message->range_max - epsilon;
                }
            }
            projector_.projectLaser(valid_scan, local_cloud);
        }
        else
        {
            projector_.projectLaser(*message, local_cloud);
        }

        const auto tr =
            tf_buffer_->lookupTransform(global_frame_, message->header.frame_id, message->header.stamp);

        // transform the cloud to the global frame
        sensor_msgs::PointCloud2 cloud;
        tf2::doTransform(local_cloud, cloud, tr);

        auto lock = map_data_->getLock();
        raytrace(tr.transform.translation.x, tr.transform.translation.y, cloud,
                         sensor->min_obstacle_height, sensor->max_obstacle_height,
                         sensor->raytrace_range, sensor->obstacle_range);

        sensor->sub_sample_count = 0;
    }
    else
        ++sensor->sub_sample_count;
}

void ObstacleData::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                                       const std::shared_ptr<Sensor>& sensor)
{
    if (sensor->sub_sample == 0 ||
        (sensor->sub_sample > 0 && sensor->sub_sample_count > sensor->sub_sample))
    {
        const auto tr =
            tf_buffer_->lookupTransform(global_frame_, message->header.frame_id, message->header.stamp);

        // transform the cloud to the global frame
        sensor_msgs::PointCloud2 cloud;
        tf2::doTransform(*message, cloud, tr);

        auto lock = map_data_->getLock();
        raytrace(tr.transform.translation.x, tr.transform.translation.y, cloud,
                         sensor->min_obstacle_height, sensor->max_obstacle_height,
                         sensor->raytrace_range, sensor->obstacle_range);

        sensor->sub_sample_count = 0;
    }
    else
        ++sensor->sub_sample_count;
}

void ObstacleData::matchSize()
{
}

void ObstacleData::raytrace(const double sensor_x, const double sensor_y, const sensor_msgs::PointCloud2& cloud,
                            const double min_obstacle_height, const double max_obstacle_height,
                            const double raytrace_range, const double obstacle_range)
{
    // get the map coordinates of the origin of the sensor
    unsigned int x0, y0;
    if (!map_data_->worldToMap(sensor_x, sensor_y, x0, y0))
    {
        ROS_WARN_THROTTLE(1.0,
                          "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap "
                          "cannot raytrace for it.",
                          sensor_x, sensor_y);
    }
    else
    {
        const double map_end_x = map_data_->originX() + map_data_->sizeX() * map_data_->resolution();
        const double map_end_y = map_data_->originY() + map_data_->sizeY() * map_data_->resolution();

        // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

//        MarkClearing marker(marking_.data(), -0.4f);
//        const unsigned int cell_raytrace_range = cellDistance(10.0);

        //        const double sq_obstacle_range = obstacle_range * obstacle_range;

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            float px = *iter_x;
            float py = *iter_y;
            const float pz = *iter_z;

            if (pz < min_obstacle_height || pz > max_obstacle_height)
            {
                continue;
            }

            //            const double sq_dist = (px - sensor_x) * (px - sensor_x) + (py - sensor_y) * (py - sensor_y);
            //            if (sq_dist >= sq_obstacle_range)
            //            {
            //                continue;
            //            }

            // now we also need to make sure that the enpoint we're raytracing
            // to isn't off the costmap and scale if necessary
            const double a = px - sensor_x;
            const double b = py - sensor_y;

            // the minimum value to raytrace from is the origin
            if (px < map_data_->originX())
            {
                const double t = (map_data_->originX() - sensor_x) / a;
                px = map_data_->originX();
                py = sensor_y + b * t;
            }
            if (py < map_data_->originY())
            {
                const double t = (map_data_->originY() - sensor_y) / b;
                px = sensor_x + a * t;
                py = map_data_->originY();
            }

            // the maximum value to raytrace to is the end of the map
            if (px > map_end_x)
            {
                const double t = (map_end_x - sensor_x) / a;
                px = map_end_x - .001;
                py = sensor_y + b * t;
            }
            if (py > map_end_y)
            {
                const double t = (map_end_y - sensor_y) / b;
                px = sensor_x + a * t;
                py = map_end_y - .001;
            }

            unsigned int x1, y1;
            if (!map_data_->worldToMap(px, py, x1, y1))
                continue;

//            raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);

//            const auto it = getIndex(x1, y1);
//            marking_[it] = std::min(3.5f, marking_[it] + 0.7f);
        }
    }
}
}
*/
