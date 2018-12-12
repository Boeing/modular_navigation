#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/plugins/static_layer.h>

#include <pluginlib/class_list_macros.h>

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::StaticLayer, costmap_2d::Layer)

using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace costmap_2d
{

StaticLayer::StaticLayer()
{
}

StaticLayer::~StaticLayer()
{
}

void StaticLayer::onInitialize()
{
    ros::NodeHandle nh("~/" + name_);
    ros::NodeHandle g_nh;

    current_ = true;
    enabled_ = true;

    global_frame_ = layered_costmap_->getGlobalFrameID();

    std::string map_topic = nh.param("map_topic", std::string("map"));

    track_unknown_space_ = nh.param("track_unknown_space", true);
    use_maximum_ = nh.param("use_maximum", false);
    trinary_costmap_ = nh.param("trinary_costmap", true);

    const int temp_lethal_threshold = nh.param("lethal_cost_threshold", 100);
    const int temp_unknown_cost_value = nh.param("unknown_cost_value", -1);

    lethal_threshold_ = static_cast<unsigned char>(std::max(std::min(temp_lethal_threshold, 100), 0));
    unknown_cost_value_ = static_cast<unsigned char>(temp_unknown_cost_value);

    map_sub_ = g_nh.subscribe(map_topic, 1, &StaticLayer::incomingMap, this);
    map_received_ = false;
    has_updated_data_ = false;
}

void StaticLayer::matchSize()
{
    // If we are using rolling costmap, the static map size is unrelated to the size of the layered costmap
    if (!layered_costmap_->isRolling())
    {
        Costmap2D* master = layered_costmap_->getCostmap();
        resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(),
                  master->getOriginY());
    }
}

unsigned char StaticLayer::interpretValue(unsigned char value) const
{
    // check if the static value is above the unknown or lethal thresholds
    if (track_unknown_space_ && value == unknown_cost_value_)
        return 225;  // NO_INFORMATION;
    else if (!track_unknown_space_ && value == unknown_cost_value_)
        return 0;  // FREE_SPACE;
    else if (value >= lethal_threshold_)
        return 254;  // LETHAL_OBSTACLE;
    else if (trinary_costmap_)
        return 0;  // FREE_SPACE;

    const double scale = static_cast<double>(value) / lethal_threshold_;
    return static_cast<unsigned char>(scale * 254);  // LETHAL_OBSTACLE);
}

void StaticLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
    ROS_INFO_STREAM("Received occupancy grid of size=" << new_map->info.width << "x" << new_map->info.height
                                                       << " resolution=" << new_map->info.resolution);

    // We want to keep the resolution provided in configuration
    // But we need to increase the size of the costmap data to fit the map

    const double size_x_m = static_cast<double>(new_map->info.width * new_map->info.resolution);
    const double size_y_m = static_cast<double>(new_map->info.height * new_map->info.resolution);

    const double resolution =
        layered_costmap_->isRolling() ? layered_costmap_->getCostmap()->getResolution() : resolution_;

    const unsigned int size_x = static_cast<unsigned int>(size_x_m / resolution);
    const unsigned int size_y = static_cast<unsigned int>(size_y_m / resolution);

    ROS_INFO_STREAM("Resizing costmap to size=" << size_x << "x" << size_y << " resolution=" << resolution);

    //
    // If rolling then don't resize the master grid
    //
    if (layered_costmap_->isRolling())
    {
        resizeMap(size_x, size_y, resolution, new_map->info.origin.position.x, new_map->info.origin.position.y);
    }
    //
    // If not rolling then resize the master grid
    // This also causes all layers to match size
    //
    else
    {
        layered_costmap_->resizeMap(size_x, size_y, resolution, new_map->info.origin.position.x,
                                    new_map->info.origin.position.y, true);
    }

    //
    // Copy the occupancy grid into the costmap
    //
    boost::unique_lock<Costmap2D::mutex_t> lock(*getMutex());
    unsigned int index = 0;
    for (unsigned int i = 0; i < size_y; ++i)
    {
        for (unsigned int j = 0; j < size_x; ++j)
        {
            const float x_min_m = static_cast<float>(j * resolution_);
            const float x_max_m = static_cast<float>((j + 1) * resolution_);

            const float y_min_m = static_cast<float>(i * resolution_);
            const float y_max_m = static_cast<float>((i + 1) * resolution_);

            const unsigned int _x_min = static_cast<unsigned int>(x_min_m / new_map->info.resolution);
            const unsigned int _x_max =
                std::min(static_cast<unsigned int>(x_max_m / new_map->info.resolution), new_map->info.width - 1);

            const unsigned int _y_min = static_cast<unsigned int>(y_min_m / new_map->info.resolution);
            const unsigned int _y_max =
                std::min(static_cast<unsigned int>(y_max_m / new_map->info.resolution), new_map->info.height - 1);

            unsigned char value = 0;
            for (unsigned int _i = _y_min; _i <= _y_max; ++_i)
            {
                for (unsigned int _j = _x_min; _j <= _x_max; ++_j)
                {
                    const unsigned int _index = (_i * new_map->info.width) + _j;
                    value = std::max(interpretValue(static_cast<unsigned char>(new_map->data[_index])), value);
                }
            }

            costmap_[index] = value;
            ++index;
        }
    }

    map_frame_ = new_map->header.frame_id;

    map_received_ = true;
    has_updated_data_ = true;
}

void StaticLayer::activate()
{
    onInitialize();
}

void StaticLayer::deactivate()
{
    map_sub_.shutdown();
}

void StaticLayer::reset()
{
    onInitialize();
}

void StaticLayer::updateBounds(double, double, double, double* min_x, double* min_y, double* max_x, double* max_y)
{
    if (!layered_costmap_->isRolling())
    {
        if (!map_received_ || !(has_updated_data_ || has_extra_bounds_))
            return;
    }

    useExtraBounds(min_x, min_y, max_x, max_y);

    double wx, wy;

    // See where the edge of the map is in the world
    mapToWorld(0, 0, wx, wy);

    *min_x = std::min(wx, *min_x);
    *min_y = std::min(wy, *min_y);

    // See where the edge of the map is in the world
    mapToWorld(size_x_, size_y_, wx, wy);

    *max_x = std::max(wx, *max_x);
    *max_y = std::max(wy, *max_y);

    has_updated_data_ = false;
}

void StaticLayer::updateCosts(costmap_2d::Costmap2D& master_grid, unsigned int min_i, unsigned int min_j,
                              unsigned int max_i, unsigned int max_j)
{
    if (!map_received_)
        return;

    boost::unique_lock<Costmap2D::mutex_t> lock(*getMutex());

    if (!layered_costmap_->isRolling())
    {
        if (!use_maximum_)
            updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
        else
            updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    }
    else
    {
        // If rolling window, the master_grid is unlikely to have same coordinates as this layer
        unsigned int mx, my;
        double wx, wy;

        // Might even be in a different frame
        geometry_msgs::TransformStamped transform;
        try
        {
            transform = tf_->lookupTransform(map_frame_, global_frame_, ros::Time(0));
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }
        // Copy map data given proper transformations
        tf2::Transform tf2_transform;
        tf2::convert(transform.transform, tf2_transform);
        for (unsigned int i = min_i; i < max_i; ++i)
        {
            for (unsigned int j = min_j; j < max_j; ++j)
            {
                // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
                layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);

                // Transform from global_frame_ to map_frame_
                tf2::Vector3 p(wx, wy, 0);
                p = tf2_transform * p;

                // Set master_grid with cell from map
                if (worldToMap(p.x(), p.y(), mx, my))
                {
                    if (!use_maximum_)
                        master_grid.setCost(i, j, getCost(mx, my));
                    else
                        master_grid.setCost(i, j, std::max(getCost(mx, my), master_grid.getCost(i, j)));
                }
            }
        }
    }
}

}  // namespace costmap_2d
