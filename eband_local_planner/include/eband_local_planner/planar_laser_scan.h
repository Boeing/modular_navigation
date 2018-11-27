#ifndef TRAJECTORY_ROLLOUT_PLANAR_LASER_SCAN_H_
#define TRAJECTORY_ROLLOUT_PLANAR_LASER_SCAN_H_

#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>

namespace eband_local_planner
{
/**
 * @class PlanarLaserScan
 * @brief Stores a scan from a planar laser that can be used to clear freespace
 */
class PlanarLaserScan
{
  public:
    PlanarLaserScan()
    {
    }
    geometry_msgs::Point32 origin;
    sensor_msgs::PointCloud cloud;
    double angle_min, angle_max, angle_increment;
};
};

#endif
