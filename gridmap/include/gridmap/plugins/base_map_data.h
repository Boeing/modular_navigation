#ifndef GRIDMAP_BASE_MAP_DATA_H
#define GRIDMAP_BASE_MAP_DATA_H

#include <gridmap/data_source.h>

#include <nav_msgs/OccupancyGrid.h>

#include <ros/ros.h>

namespace gridmap
{

class BaseMapData : public DataSource
{
  public:
    BaseMapData();
    virtual ~BaseMapData() override;

    virtual void onInitialize(const XmlRpc::XmlRpcValue& parameters) override;

    virtual void matchSize() override;

  private:
    double interpretValue(unsigned char value) const;
    void incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map);

    int lethal_threshold_;

    ros::Subscriber map_sub_;
};
}

#endif
