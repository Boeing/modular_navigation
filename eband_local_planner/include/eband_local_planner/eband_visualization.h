#ifndef EBAND_LOCAL_PLANNER_EBAND_VISUALIZATION_H
#define EBAND_LOCAL_PLANNER_EBAND_VISUALIZATION_H

#include <ros/ros.h>

#include <string>
#include <vector>

#include <eband_local_planner/conversions_and_types.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <costmap_2d/costmap_2d_ros.h>

namespace eband_local_planner
{

class EBandVisualization
{
  public:
    enum class Color
    {
        blue,
        red,
        green
    };

    EBandVisualization(ros::NodeHandle& pn, const std::shared_ptr<costmap_2d::Costmap2DROS>& local_costmap);
    ~EBandVisualization();

    void publishBand(const std::vector<Bubble>& band, const std::string& marker_name_space="bubbles");
    void publishBubble(const Bubble& bubble, const int marker_id, const Color marker_color=Color::red, const std::string& marker_name_space="ctrl_target");

  private:
    const std::shared_ptr<costmap_2d::Costmap2DROS> local_costmap_;

    ros::Publisher bubble_pub_;
    ros::Publisher one_bubble_pub_;

    double marker_lifetime_;

    visualization_msgs::Marker makeMarker(const Bubble& bubble, const std::string& marker_name_space, const int marker_id, const Color& marker_color) const;
};

}

#endif
