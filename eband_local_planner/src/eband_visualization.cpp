#include <eband_local_planner/eband_visualization.h>

#include <string>
#include <vector>

namespace eband_local_planner
{

EBandVisualization::EBandVisualization(ros::NodeHandle& pn,
                                       const std::shared_ptr<costmap_2d::Costmap2DROS>& local_costmap)
    : local_costmap_(local_costmap)
{
    pn.param("marker_lifetime", marker_lifetime_, 0.2);
    one_bubble_pub_ = pn.advertise<visualization_msgs::Marker>("eband_visualization", 1);
    bubble_pub_ = pn.advertise<visualization_msgs::MarkerArray>("eband_visualization_array", 1);
}

EBandVisualization::~EBandVisualization()
{
}

void EBandVisualization::publishBand(const std::vector<Bubble>& band, const std::string& marker_name_space)
{
    if (band.empty())
        return;

    visualization_msgs::MarkerArray eband_msg;
    eband_msg.markers.resize(band.size());

    visualization_msgs::MarkerArray eband_heading_msg;
    eband_heading_msg.markers.resize(band.size());
    std::string marker_heading_name_space = marker_name_space;
    marker_heading_name_space.append("_heading");

    for (std::size_t i = 0; i < band.size(); i++)
    {
        eband_msg.markers[i] = makeMarker(band[i], marker_name_space, static_cast<int>(i), Color::green);
    }

    bubble_pub_.publish(eband_msg);
}

void EBandVisualization::publishBubble(const Bubble& bubble, const int marker_id, const Color marker_color, const std::string& marker_name_space)
{
    visualization_msgs::Marker bubble_msg = makeMarker(bubble, marker_name_space, marker_id, marker_color);
    one_bubble_pub_.publish(bubble_msg);
}

visualization_msgs::Marker EBandVisualization::makeMarker(const Bubble& bubble, const std::string& marker_name_space, const int marker_id, const Color& marker_color) const
{
    visualization_msgs::Marker marker;

    // header
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = local_costmap_->getGlobalFrameID();

    // identifier and cmds
    marker.ns = marker_name_space;
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    // body
    marker.pose = bubble.center;

    // scale ~ diameter --> is 2x expansion ~ radius
    marker.scale.x = 2.0 * bubble.expansion;
    marker.scale.y = 2.0 * bubble.expansion;
    marker.scale.z = 2.0 * bubble.expansion;

    // color (rgb)
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    switch (marker_color)
    {
        case Color::red:
        {
            marker.color.r = 1.0f;
            break;
        }
        case Color::green:
        {
            marker.color.g = 1.0f;
            break;
        }
        case Color::blue:
        {
            marker.color.b = 1.0f;
            break;
        }
    }

    // transparency (alpha value < 1 : displays marker transparent)
    marker.color.a = 0.75;

    // lifetime of this marker
    marker.lifetime = ros::Duration(marker_lifetime_);

    return marker;
}

}
