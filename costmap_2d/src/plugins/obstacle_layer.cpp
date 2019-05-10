#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/plugins/obstacle_layer.h>

#include <tf2_ros/message_filter.h>

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/highgui.hpp>

#include <chrono>

PLUGINLIB_EXPORT_CLASS(costmap_2d::ObstacleLayer, costmap_2d::Layer)

namespace costmap_2d
{

namespace
{

class MarkClearing
{
  public:
    MarkClearing(float* marking, float value)
        : marking_(marking), value_(value)
    {
    }
    inline void operator()(unsigned int offset)
    {
        marking_[offset] = std::max(-2.0f, std::min(3.5f, marking_[offset] + value_));
    }

  private:
    float* marking_;
    float value_;
};

}

ObstacleLayer::ObstacleLayer() : footprint_clearing_enabled_(false)
{
}

void ObstacleLayer::onInitialize()
{
    ros::NodeHandle nh("~/" + name_), g_nh;

    default_value_ = NO_INFORMATION;

    ObstacleLayer::matchSize();
    current_ = true;

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
                new tf2_ros::MessageFilter<sensor_msgs::LaserScan>(*sub, *tf_, layered_costmap_->getGlobalFrameID(), 50, g_nh));

            boost::shared_ptr<message_filters::SubscriberBase> subscriber(sub);
            boost::shared_ptr<tf2_ros::MessageFilterBase> message_filter(filter);

            auto source = std::make_shared<DataSource>(DataSource{subscriber, message_filter, observation_persistence, expected_update_rate,
                    min_obstacle_height, max_obstacle_height, obstacle_range, raytrace_range, sub_sample,
                    0, inf_is_valid});

            filter->registerCallback(boost::bind(&ObstacleLayer::laserScanCallback, this, _1, source));

            data_sources_.push_back(source);
        }
        else if (data_type == "PointCloud2")
        {
            boost::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub(
                new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));

            boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> filter(
                new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_, layered_costmap_->getGlobalFrameID(), 50, g_nh));

            boost::shared_ptr<message_filters::SubscriberBase> subscriber(sub);
            boost::shared_ptr<tf2_ros::MessageFilterBase> message_filter(filter);

            auto source = std::make_shared<DataSource>(DataSource{subscriber, message_filter, observation_persistence, expected_update_rate,
                    min_obstacle_height, max_obstacle_height, obstacle_range, raytrace_range, sub_sample,
                    0, inf_is_valid});

            filter->registerCallback(boost::bind(&ObstacleLayer::pointCloud2Callback, this, _1, source));

            data_sources_.push_back(source);
        }
        else
        {
            const std::string msg = "Only topics that use point clouds or laser scans are currently supported";
            ROS_FATAL_STREAM(msg);
            throw std::runtime_error(msg);
        }
    }

    enabled_ = true;
    footprint_clearing_enabled_ = true;
}

ObstacleLayer::~ObstacleLayer()
{
}

void ObstacleLayer::laserScanCallback(const sensor_msgs::LaserScanConstPtr& message,
                                      const std::shared_ptr<DataSource>& data_source)
{
    if (data_source->sub_sample == 0 || (data_source->sub_sample > 0 && data_source->sub_sample_count > data_source->sub_sample))
    {
        ROS_INFO("Processing laser");

        sensor_msgs::PointCloud2 local_cloud;

        if (data_source->inf_is_valid)
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

        const auto tr = tf_->lookupTransform(layered_costmap_->getGlobalFrameID(), message->header.frame_id, message->header.stamp);

        // transform the cloud to the global frame
        sensor_msgs::PointCloud2 cloud;
        tf2::doTransform(local_cloud, cloud, tr);

        {
            boost::unique_lock<Costmap2D::mutex_t> lock(*getMutex());
            raytraceClearing(tr.transform.translation.x, tr.transform.translation.y, cloud, data_source->min_obstacle_height, data_source->max_obstacle_height, data_source->raytrace_range, data_source->obstacle_range);
//            markPoints(tr.transform.translation.x, tr.transform.translation.y, cloud, data_source->min_obstacle_height, data_source->max_obstacle_height, data_source->obstacle_range);
        }

        data_source->sub_sample_count = 0;
    }
    else
        ++data_source->sub_sample_count;
}

void ObstacleLayer::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                                        const std::shared_ptr<DataSource>& data_source)
{
    if (data_source->sub_sample == 0 || (data_source->sub_sample > 0 && data_source->sub_sample_count > data_source->sub_sample))
    {
        const auto tr = tf_->lookupTransform(layered_costmap_->getGlobalFrameID(), message->header.frame_id, message->header.stamp);

        // transform the cloud to the global frame
        sensor_msgs::PointCloud2 cloud;
        tf2::doTransform(*message, cloud, tr);

        {
            boost::unique_lock<Costmap2D::mutex_t> lock(*getMutex());
            raytraceClearing(tr.transform.translation.x, tr.transform.translation.y, cloud, data_source->min_obstacle_height, data_source->max_obstacle_height, data_source->raytrace_range, data_source->obstacle_range);
//            markPoints(tr.transform.translation.x, tr.transform.translation.y, cloud, data_source->min_obstacle_height, data_source->max_obstacle_height, data_source->obstacle_range);
        }

        data_source->sub_sample_count = 0;
    }
    else
        ++data_source->sub_sample_count;
}

void ObstacleLayer::updateBounds(const double robot_x, const double robot_y, const double robot_yaw, double* min_x,
                                 double* min_y, double* max_x, double* max_y)
{
    if (!enabled_)
        return;

    unsigned int robot_mx;
    unsigned int robot_my;
    layered_costmap_->getCostmap()->worldToMap(robot_x, robot_y, robot_mx, robot_my);

    *min_x = robot_x - 2.0;
    *max_x = robot_x + 2.0;
    *min_y = robot_y - 2.0;
    *max_y = robot_y + 2.0;

    // clear robot_radius
    if (footprint_clearing_enabled_)
    {

    }
}

void ObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, unsigned int min_i, unsigned int min_j,
                                unsigned int max_i, unsigned int max_j)
{
    if (!enabled_)
        return;

    boost::unique_lock<Costmap2D::mutex_t> lock(*getMutex());

    unsigned char* master = master_grid.getCharMap();
    const unsigned int span = master_grid.getSizeInCellsX();

    unsigned int it;
    unsigned int i;
    for (unsigned int j = min_j; j < max_j; j++)
    {
        it = span * j + min_i;
        for (i = min_i; i < max_i; i++)
        {
            if (marking_[it] >= 3.5f)
            {
                master[it] = costmap_2d::LETHAL_OBSTACLE;
            }
            else
            {
                master[it] = costmap_2d::FREE_SPACE;
            }

            it++;
        }
    }

//    cv::Mat cv_im = cv::Mat(size_y_, size_x_, CV_32F, reinterpret_cast<void*>(marking_timestamp_.data()));
//    cv::Mat ts_u8;
//    cv_im.convertTo(ts_u8, CV_8U, 10.0, 0);
//    cv::namedWindow("win", cv::WINDOW_NORMAL);
//    cv::imshow("win", ts_u8);
//    cv::waitKey(1);
}

void ObstacleLayer::activate()
{
    for (const auto& ds : data_sources_)
    {
        ds->subscriber->subscribe();
    }
}

void ObstacleLayer::deactivate()
{
    for (const auto& ds : data_sources_)
    {
        ds->subscriber->unsubscribe();
    }
}

void ObstacleLayer::reset()
{
    deactivate();
    resetMaps();
    current_ = true;
    activate();
}

void ObstacleLayer::matchSize()
{

    auto master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(),
              master->getOriginY());

    ROS_INFO_STREAM("matchSize " << master->getSizeInCellsX() << " " << master->getSizeInCellsY());


    if (master->getSizeInCellsX() > 0 && master->getSizeInCellsY())
    {
        ROS_INFO_STREAM("building " << master->getSizeInCellsX() << " " << master->getSizeInCellsY());

        const int size = master->getSizeInCellsY() * master->getSizeInCellsX();

        marking_ = std::vector<float>(size, 0.5f);
    }
}

void ObstacleLayer::raytraceClearing(const double sensor_x, const double sensor_y, const sensor_msgs::PointCloud2& cloud,
                                     const double min_obstacle_height, const double max_obstacle_height,
                                     const double raytrace_range, const double obstacle_range)
{
    // get the map coordinates of the origin of the sensor
    unsigned int x0, y0;
    if (!worldToMap(sensor_x, sensor_y, x0, y0))
    {
        ROS_WARN_THROTTLE(1.0,
                          "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap "
                          "cannot raytrace for it.",
                          sensor_x, sensor_y);
    }
    else
    {
        // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
        const double map_end_x = origin_x_ + size_x_ * resolution_;
        const double map_end_y = origin_y_ + size_y_ * resolution_;

        // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

//        cv::Mat cv_im = cv::Mat(size_y_, size_x_, CV_8UC1, reinterpret_cast<void*>(marking_.data()));

        MarkClearing marker(marking_.data(), -0.4f);
        const unsigned int cell_raytrace_range = cellDistance(10.0);

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
            if (px < origin_x_)
            {
                const double t = (origin_x_ - sensor_x) / a;
                px = origin_x_;
                py = sensor_y + b * t;
            }
            if (py < origin_y_)
            {
                const double t = (origin_y_ - sensor_y) / b;
                px = sensor_x + a * t;
                py = origin_y_;
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
            if (!worldToMap(px, py, x1, y1))
                continue;

            raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);

            const auto it = getIndex(x1, y1);
            marking_[it] = std::min(3.5f, marking_[it] + 0.7f);
        }

//        cv::namedWindow("win", cv::WINDOW_NORMAL);
//        cv::imshow("win", cv_im);
//        cv::waitKey(1);
    }
}

}
