#ifndef TRAJECTORY_ROLLOUT_COSTMAP_MODEL_
#define TRAJECTORY_ROLLOUT_COSTMAP_MODEL_

#include <eband_local_planner/world_model.h>

#include <costmap_2d/costmap_2d.h>

namespace eband_local_planner
{
/**
 * @class CostmapModel
 * @brief A class that implements the WorldModel interface to provide grid
 * based collision checks for the trajectory controller using the costmap.
 */
class CostmapModel : public WorldModel
{
  public:
    /**
     * @brief  Constructor for the CostmapModel
     * @param costmap The costmap that should be used
     * @return
     */
    CostmapModel(const costmap_2d::Costmap2D& costmap);

    /**
     * @brief  Destructor for the world model
     */
    virtual ~CostmapModel()
    {
    }
    using WorldModel::footprintCost;

    /**
     * @brief  Checks if any obstacles in the costmap lie inside a convex footprint that is rasterized into the grid
     * @param  position The position of the robot in world coordinates
     * @param  footprint The specification of the footprint of the robot in world coordinates
     * @param  inscribed_radius The radius of the inscribed circle of the robot
     * @param  circumscribed_radius The radius of the circumscribed circle of the robot
     * @return Positive if all the points lie outside the footprint, negative otherwise
     */
    virtual double footprintCost(const geometry_msgs::Point& position,
                                 const std::vector<geometry_msgs::Point>& footprint, double inscribed_radius,
                                 double circumscribed_radius);

    /**
     * @brief  Rasterizes a line in the costmap grid and checks for collisions
     * @param x0 The x position of the first cell in grid coordinates
     * @param y0 The y position of the first cell in grid coordinates
     * @param x1 The x position of the second cell in grid coordinates
     * @param y1 The y position of the second cell in grid coordinates
     * @return A positive cost for a legal line... negative otherwise
     */
    double lineCost(int x0, int x1, int y0, int y1) const;

    /**
     * @brief  Checks the cost of a point in the costmap
     * @param x The x position of the point in cell coordinates
     * @param y The y position of the point in cell coordinates
     * @return A positive cost for a legal point... negative otherwise
     */
    double pointCost(int x, int y) const;

  private:
    const costmap_2d::Costmap2D& costmap_;  ///< @brief Allows access of costmap obstacle information
};
};
#endif
