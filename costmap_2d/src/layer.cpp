#include "costmap_2d/layer.h"

namespace costmap_2d
{

Layer::Layer() : layered_costmap_(nullptr), current_(false), enabled_(false), name_(), tf_(nullptr)
{
}

void Layer::initialize(LayeredCostmap* parent, std::string name, tf2_ros::Buffer* tf)
{
    layered_costmap_ = parent;
    name_ = name;
    tf_ = tf;
    onInitialize();
}

const std::vector<geometry_msgs::Point>& Layer::getFootprint() const
{
    return layered_costmap_->getFootprint();
}

}  // end namespace costmap_2d
