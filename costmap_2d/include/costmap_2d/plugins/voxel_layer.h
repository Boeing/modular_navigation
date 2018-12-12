#ifndef COSTMAP_2D_VOXEL_LAYER_H_
#define COSTMAP_2D_VOXEL_LAYER_H_

#include <costmap_2d/VoxelGrid.h>
#include <costmap_2d/VoxelPluginConfig.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>
#include <costmap_2d/plugins/obstacle_layer.h>

#include <dynamic_reconfigure/server.h>
#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <voxel_grid/voxel_grid.h>

namespace costmap_2d
{

class VoxelLayer : public ObstacleLayer
{
  public:
    VoxelLayer() : voxel_grid_(0, 0, 0)
    {
        costmap_ = nullptr;  // this is the unsigned char* member of parent class's parent class Costmap2D.
    }

    virtual ~VoxelLayer() override;

    virtual void onInitialize() override;

    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                              double* max_x, double* max_y) override;

    void updateOrigin(double new_origin_x, double new_origin_y) override;

    virtual void matchSize() override;

    virtual void reset() override;

  protected:
    virtual void onFootprintChanged() override
    {
    }

    virtual void setupDynamicReconfigure(ros::NodeHandle& nh) override;

    virtual void resetMaps() override;

  private:
    void reconfigureCB(costmap_2d::VoxelPluginConfig& config, uint32_t level);
    void clearNonLethal(double wx, double wy, double w_size_x, double w_size_y, bool clear_no_info);
    virtual void raytraceFreespace(const costmap_2d::Observation& clearing_observation, double* min_x, double* min_y,
                                   double* max_x, double* max_y) override;

    dynamic_reconfigure::Server<costmap_2d::VoxelPluginConfig>* voxel_dsrv_;

    bool publish_voxel_;
    ros::Publisher voxel_pub_;
    voxel_grid::VoxelGrid voxel_grid_;
    double z_resolution_, origin_z_;
    unsigned int unknown_threshold_, mark_threshold_, size_z_;
    ros::Publisher clearing_endpoints_pub_;
    sensor_msgs::PointCloud clearing_endpoints_;

    inline bool worldToMap3DFloat(double wx, double wy, double wz, double& mx, double& my, double& mz)
    {
        if (wx < origin_x_ || wy < origin_y_ || wz < origin_z_)
            return false;
        mx = ((wx - origin_x_) / resolution_);
        my = ((wy - origin_y_) / resolution_);
        mz = ((wz - origin_z_) / z_resolution_);
        if (mx < size_x_ && my < size_y_ && mz < size_z_)
            return true;

        return false;
    }

    inline bool worldToMap3D(double wx, double wy, double wz, unsigned int& mx, unsigned int& my, unsigned int& mz)
    {
        if (wx < origin_x_ || wy < origin_y_ || wz < origin_z_)
            return false;

        mx = (int)((wx - origin_x_) / resolution_);
        my = (int)((wy - origin_y_) / resolution_);
        mz = (int)((wz - origin_z_) / z_resolution_);

        if (mx < size_x_ && my < size_y_ && mz < size_z_)
            return true;

        return false;
    }

    inline void mapToWorld3D(unsigned int mx, unsigned int my, unsigned int mz, double& wx, double& wy, double& wz)
    {
        // returns the center point of the cell
        wx = origin_x_ + (mx + 0.5) * resolution_;
        wy = origin_y_ + (my + 0.5) * resolution_;
        wz = origin_z_ + (mz + 0.5) * z_resolution_;
    }

    inline double dist(double x0, double y0, double z0, double x1, double y1, double z1)
    {
        return sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0));
    }
};
}

#endif
