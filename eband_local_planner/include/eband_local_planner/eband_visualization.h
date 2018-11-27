#ifndef EBAND_LOCAL_PLANNER_EBAND_VISUALIZATION_H
#define EBAND_LOCAL_PLANNER_EBAND_VISUALIZATION_H

#include <ros/ros.h>

#include <string>
#include <vector>

#include <eband_local_planner/conversions_and_types.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace eband_local_planner
{

/**
 * @class ConversionsAndTypes
 * @brief Implements type-convrsions and types used by elastic-band optimizer and according ros-wrapper class
 */
class EBandVisualization
{
  public:
    // typedefs
    enum Color
    {
        blue,
        red,
        green
    };

    EBandVisualization(ros::NodeHandle& pn, costmap_2d::Costmap2DROS* costmap_ros);
    ~EBandVisualization();

    /**
     * @brief publishes the bubbles (Position and Expansion) in a band as Marker-Array
     * @param the name space under which the markers shall be bublished
     * @param the shape of the markers
     * @param the band which shall be published
     */
    void publishBand(std::string marker_name_space, std::vector<Bubble> band);

    /**
     * @brief publishes a single bubble as a Marker
     * @param the name space under which the markers shall be bublished
     * @param the shape of the markers
     * @param the bubble which shall be published
     */
    void publishBubble(std::string marker_name_space, int marker_id, Bubble bubble);

    /**
     * @brief publishes a single bubble as a Marker
     * @param the name space under which the markers shall be bublished
     * @param the shape of the markers
     * @param the bubble which shall be published
     * @param color in which the bubble shall be displayed
     */
    void publishBubble(std::string marker_name_space, int marker_id, Color marker_color, Bubble bubble);

    /**
     * @brief publishes the list of forces along the band as Marker-Array
     * @param the name space under which the markers shall be bublished
     * @param the shape of the markers
     * @param the list of forces which shall be published
     * @param the list of bubbles on which the forces act (needed to get origin of force-vector)
     */
    void publishForceList(std::string marker_name_space, std::vector<geometry_msgs::WrenchStamped> forces,
                          std::vector<Bubble> band);

    /**
     * @brief publishes a single force as a Marker
     * @param the name space under which the markers shall be bublished
     * @param the shape of the markers
     * @param the force which shall be published
     */
    void publishForce(std::string marker_name_space, int id, Color marker_color, geometry_msgs::WrenchStamped force,
                      Bubble bubble);

  private:
    costmap_2d::Costmap2DROS* costmap_ros_;

    ros::Publisher bubble_pub_;
    ros::Publisher one_bubble_pub_;

    double marker_lifetime_;

    /**
     * @brief converts a bubble into a Marker msg - this is visualization-specific
     * @param the bubble to convert
     * @param reference to hand back the marker
     * @param name space under which the marker shall be published
     * @param object id of the marker in its name space
     */
    void bubbleToMarker(Bubble bubble, visualization_msgs::Marker& marker, std::string marker_name_space, int marker_id,
                        Color marker_color);

    /**
     * @brief converts the haeding of a bubble into a Arrow-Marker msg - this is visualization-specific
     * @param the bubble to convert
     * @param reference to hand back the marker
     * @param name space under which the marker shall be published
     * @param object id of the marker in its name space
     */
    void bubbleHeadingToMarker(Bubble bubble, visualization_msgs::Marker& marker, std::string marker_name_space,
                               int marker_id, Color marker_color);

    /**
     * @brief converts a wrench into a Marker msg - this is visualization-specific
     * @param the wrench to convert
     * @param origin of force or wrench
     * @param reference to hand back the marker
     * @param name space under which the marker shall be published
     * @param object id of the marker in its name space
     * @param color in which the marker shall be displayed
     */
    void forceToMarker(geometry_msgs::WrenchStamped wrench, geometry_msgs::Pose wrench_origin,
                       visualization_msgs::Marker& marker, std::string marker_name_space, int marker_id,
                       Color marker_color);
};

}  // namespace eband_local_planner

#endif  // EBAND_LOCAL_PLANNER_EBAND_VISUALIZATION_H
