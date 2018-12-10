#ifndef COSTMAP_2D_STATIC_LAYER_H_
#define COSTMAP_2D_STATIC_LAYER_H_

#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

namespace costmap_2d
{

class StaticLayer : public CostmapLayer
{
  public:
    StaticLayer();
    virtual ~StaticLayer();
    virtual void onInitialize();
    virtual void activate();
    virtual void deactivate();
    virtual void reset();

    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                              double* max_x, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, unsigned int min_i, unsigned int min_j, unsigned int max_i, unsigned int max_j);

    virtual void matchSize();

  protected:
    virtual void onFootprintChanged(){}

  private:
    void incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map);

    unsigned char interpretValue(unsigned char value) const;

    std::string global_frame_;
    std::string map_frame_;

    bool map_received_;
    bool has_updated_data_;
    bool track_unknown_space_;
    bool use_maximum_;
    bool trinary_costmap_;

    ros::Subscriber map_sub_;

    unsigned char lethal_threshold_;
    unsigned char unknown_cost_value_;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_STATIC_LAYER_H_
