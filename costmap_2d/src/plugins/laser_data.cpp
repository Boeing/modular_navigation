#include <costmap_2d/plugins/laser_data.h>
#include <costmap_2d/raytrace.h>
#include <costmap_2d/params.h>

#include <tf2_ros/message_filter.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/point_cloud2_iterator.h>

#include <opencv2/highgui.hpp>

#include <chrono>

PLUGINLIB_EXPORT_CLASS(costmap_2d::LaserData, costmap_2d::DataSource)

namespace costmap_2d
{

LaserData::LaserData()
{
}

void LaserData::onInitialize(const XmlRpc::XmlRpcValue& parameters)
{
    ros::NodeHandle nh("~/" + name_);
    ros::NodeHandle g_nh;

    const std::string topic = get_config_with_default_warn<std::string>(parameters, "topic", "/laser/scan", XmlRpc::XmlRpcValue::TypeString);
    hit_probability_log_ = logodds(get_config_with_default_warn<double>(parameters, "hit_probability", 0.7, XmlRpc::XmlRpcValue::TypeDouble));
    miss_probability_log_ = logodds(get_config_with_default_warn<double>(parameters, "miss_probability", 0.4, XmlRpc::XmlRpcValue::TypeDouble));
    min_obstacle_height_ = get_config_with_default_warn<double>(parameters, "min_obstacle_height", 0.0, XmlRpc::XmlRpcValue::TypeDouble);
    min_obstacle_height_ = get_config_with_default_warn<double>(parameters, "min_obstacle_height", 0.0, XmlRpc::XmlRpcValue::TypeDouble);
    max_obstacle_height_ = get_config_with_default_warn<double>(parameters, "max_obstacle_height", 2.0, XmlRpc::XmlRpcValue::TypeDouble);
    obstacle_range_ = get_config_with_default_warn<double>(parameters, "obstacle_range", 2.5, XmlRpc::XmlRpcValue::TypeDouble);
    raytrace_range_ = get_config_with_default_warn<double>(parameters, "raytrace_range", 3.0, XmlRpc::XmlRpcValue::TypeDouble);
    sub_sample_ = get_config_with_default_warn<int>(parameters, "sub_sample", 10, XmlRpc::XmlRpcValue::TypeInt);
    inf_is_valid_ = get_config_with_default_warn<bool>(parameters, "inf_is_valid", false, XmlRpc::XmlRpcValue::TypeBoolean);

    ROS_INFO_STREAM("Subscribing to laser: " << topic);

    subscriber_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(g_nh, topic, 50));
    message_filter_.reset(new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*subscriber_, *tf_buffer_, global_frame_, 50, g_nh));
    message_filter_->registerCallback(boost::bind(&LaserData::laserScanCallback, this, _1));
}

LaserData::~LaserData()
{
}

void LaserData::laserScanCallback(const sensor_msgs::LaserScanConstPtr& message)
{
    if (sub_sample_ == 0 ||
        (sub_sample_ > 0 && sub_sample_count_ > sub_sample_))
    {
        ROS_INFO("Processing laser");

        sensor_msgs::PointCloud2 local_cloud;

        if (inf_is_valid_)
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
        raytrace(tr.transform.translation.x, tr.transform.translation.y, cloud);

        sub_sample_count_ = 0;
    }
    else
        ++sub_sample_count_;
}

void LaserData::matchSize()
{
}

void LaserData::raytrace(const double sensor_x, const double sensor_y, const sensor_msgs::PointCloud2& cloud)
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

//        ROS_INFO_STREAM("miss_probability_log_: " << miss_probability_log_);

//        ROS_INFO_STREAM("map_data_: " << map_data_->sizeX() << " " << map_data_->sizeY());

        AddLogCost marker(map_data_->data(), miss_probability_log_, map_data_->clampingThresMin(), map_data_->clampingThresMax());

        const unsigned int cell_raytrace_range = map_data_->distance(100.0);

        //        const double sq_obstacle_range = obstacle_range * obstacle_range;

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            float px = *iter_x;
            float py = *iter_y;
            const float pz = *iter_z;

            if (pz < min_obstacle_height_ || pz > max_obstacle_height_)
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

//            ROS_INFO_STREAM("World to map 0: " << x0 << " " << y0);
//            ROS_INFO_STREAM("World to map 1: " << x1 << " " << y1);

            raytraceLine(marker, x0, y0, x1, y1, map_data_->sizeX(), cell_raytrace_range);

            map_data_->update(x1, y1, hit_probability_log_);
        }
    }
}
}
