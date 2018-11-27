#ifndef TRAJECTORY_ROLLOUT_WORLD_MODEL_H_
#define TRAJECTORY_ROLLOUT_WORLD_MODEL_H_

#include <eband_local_planner/planar_laser_scan.h>

#include <costmap_2d/footprint.h>
#include <costmap_2d/observation.h>

#include <geometry_msgs/Point.h>

#include <vector>

namespace eband_local_planner
{
/**
 * @class WorldModel
 * @brief An interface the trajectory controller uses to interact with the
 * world regardless of the underlying world model.
 */
class WorldModel
{
  public:
    /**
     * @brief  Subclass will implement this method to check a footprint at a given position and orientation for legality
     * in the world
     * @param  position The position of the robot in world coordinates
     * @param  footprint The specification of the footprint of the robot in world coordinates
     * @param  inscribed_radius The radius of the inscribed circle of the robot
     * @param  circumscribed_radius The radius of the circumscribed circle of the robot
     * @return Positive if all the points lie outside the footprint, negative otherwise
     */
    virtual double footprintCost(const geometry_msgs::Point& position,
                                 const std::vector<geometry_msgs::Point>& footprint, double inscribed_radius,
                                 double circumscribed_radius) = 0;

    double footprintCost(double x, double y, double theta, const std::vector<geometry_msgs::Point>& footprint_spec,
                         double inscribed_radius = 0.0, double circumscribed_radius = 0.0)
    {

        double cos_th = cos(theta);
        double sin_th = sin(theta);
        std::vector<geometry_msgs::Point> oriented_footprint;
        for (unsigned int i = 0; i < footprint_spec.size(); ++i)
        {
            geometry_msgs::Point new_pt;
            new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
            new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
            oriented_footprint.push_back(new_pt);
        }

        geometry_msgs::Point robot_position;
        robot_position.x = x;
        robot_position.y = y;

        if (inscribed_radius == 0.0)
        {
            costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius, circumscribed_radius);
        }

        return footprintCost(robot_position, oriented_footprint, inscribed_radius, circumscribed_radius);
    }

    /**
     * @brief  Checks if any obstacles in the costmap lie inside a convex footprint that is rasterized into the grid
     * @param  position The position of the robot in world coordinates
     * @param  footprint The specification of the footprint of the robot in world coordinates
     * @param  inscribed_radius The radius of the inscribed circle of the robot
     * @param  circumscribed_radius The radius of the circumscribed circle of the robot
     * @return Positive if all the points lie outside the footprint, negative otherwise
     */
    double footprintCost(const geometry_msgs::Point& position, const std::vector<geometry_msgs::Point>& footprint,
                         double inscribed_radius, double circumscribed_radius, double extra)
    {
        return footprintCost(position, footprint, inscribed_radius, circumscribed_radius);
    }

    /**
     * @brief  Subclass will implement a destructor
     */
    virtual ~WorldModel()
    {
    }

  protected:
    WorldModel()
    {
    }
};
};
#endif
