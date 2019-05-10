#ifndef COSTMAP_2D_COSTMAP_2D_PUBLISHER_H_
#define COSTMAP_2D_COSTMAP_2D_PUBLISHER_H_

#include <costmap_2d/costmap_2d.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

namespace costmap_2d
{

class Costmap2DPublisher
{
  public:
    Costmap2DPublisher(ros::NodeHandle* ros_node, Costmap2D& costmap, std::string global_frame, std::string topic_name,
                       bool always_send_full_costmap = false);

    ~Costmap2DPublisher();
    void updateBounds(unsigned int x0, unsigned int xn, unsigned int y0, unsigned int yn)
    {
        x0_ = std::min(x0, x0_);
        xn_ = std::max(xn, xn_);
        y0_ = std::min(y0, y0_);
        yn_ = std::max(yn, yn_);
    }

    void publishCostmap();

    bool active()
    {
        return active_;
    }

  private:
    void prepareGrid();

    void onNewSubscription(const ros::SingleSubscriberPublisher& pub);

    ros::NodeHandle* node;
    Costmap2D& costmap_;
    std::string global_frame_;

    unsigned int x0_;
    unsigned int xn_;
    unsigned int y0_;
    unsigned int yn_;

    double saved_origin_x_;
    double saved_origin_y_;

    bool active_;
    bool always_send_full_costmap_;

    ros::Publisher costmap_pub_;
    ros::Publisher costmap_update_pub_;
    nav_msgs::OccupancyGrid grid_;

    static char* cost_translation_table_;
};
}

#endif
