#ifndef COSTMAP_2D_INFLATION_LAYER_H_
#define COSTMAP_2D_INFLATION_LAYER_H_

#include <boost/thread.hpp>
#include <costmap_2d/InflationPluginConfig.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

namespace costmap_2d
{
/**
 * @class CellData
 * @brief Storage for cell information used during obstacle inflation
 */
class CellData
{
  public:
    /**
     * @brief  Constructor for a CellData objects
     * @param  i The index of the cell in the cost map
     * @param  x The x coordinate of the cell in the cost map
     * @param  y The y coordinate of the cell in the cost map
     * @param  sx The x coordinate of the closest obstacle cell in the costmap
     * @param  sy The y coordinate of the closest obstacle cell in the costmap
     * @return
     */
    CellData(const double i, const unsigned int x, const unsigned int y, const unsigned int sx, const unsigned int sy)
        : index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
    {
    }
    unsigned int index_;
    unsigned int x_, y_;
    unsigned int src_x_, src_y_;
};

class InflationLayer : public Layer
{
  public:
    InflationLayer();

    virtual ~InflationLayer() override
    {
        deleteKernels();
        if (dsrv_)
            delete dsrv_;
    }

    virtual void onInitialize() override;

    virtual void activate() override
    {
    }

    virtual void deactivate() override
    {
    }

    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                              double* max_x, double* max_y) override;

    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, unsigned int min_i, unsigned int min_j,
                             unsigned int max_i, unsigned int max_j) override;

    virtual void matchSize() override;

    virtual void reset() override
    {
        onInitialize();
    }

    /** @brief  Given a distance, compute a cost.
     * @param  distance The distance from an obstacle in cells
     * @return A cost value for the distance */
    inline unsigned char computeCost(const double distance) const
    {
        unsigned char cost = 0;
        if (distance == 0)
            cost = LETHAL_OBSTACLE;
        else if (distance * resolution_ <= inscribed_radius_)
            cost = INSCRIBED_INFLATED_OBSTACLE;
        else
        {
            // make sure cost falls off by Euclidean distance
            double euclidean_distance = distance * resolution_;
            double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
            cost = (unsigned char)((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
        }
        return cost;
    }

    /**
     * @brief Change the values of the inflation radius parameters
     * @param inflation_radius The new inflation radius
     * @param cost_scaling_factor The new weight
     */
    void setInflationParameters(const double inflation_radius, const double cost_scaling_factor);

  protected:
    virtual void onFootprintChanged();
    boost::recursive_mutex* inflation_access_;

    double resolution_;
    double inflation_radius_;
    double inscribed_radius_;
    double weight_;
    bool inflate_unknown_;

  private:
    /**
     * @brief  Lookup pre-computed distances
     * @param mx The x coordinate of the current cell
     * @param my The y coordinate of the current cell
     * @param src_x The x coordinate of the source cell
     * @param src_y The y coordinate of the source cell
     * @return
     */
    inline double distanceLookup(const int mx, const int my, const int src_x, const int src_y)
    {
        unsigned int dx = abs(mx - src_x);
        unsigned int dy = abs(my - src_y);
        return cached_distances_[dx][dy];
    }

    /**
     * @brief  Lookup pre-computed costs
     * @param mx The x coordinate of the current cell
     * @param my The y coordinate of the current cell
     * @param src_x The x coordinate of the source cell
     * @param src_y The y coordinate of the source cell
     * @return
     */
    inline unsigned char costLookup(const int mx, const int my, const int src_x, const int src_y)
    {
        unsigned int dx = abs(mx - src_x);
        unsigned int dy = abs(my - src_y);
        return cached_costs_[dx][dy];
    }

    void computeCaches();
    void deleteKernels();
    void inflate_area(const int min_i, const int min_j, const int max_i, const int max_j, unsigned char* master_grid);

    unsigned int cellDistance(const double world_dist)
    {
        return layered_costmap_->getCostmap()->cellDistance(world_dist);
    }

    inline void enqueue(const unsigned int index, const unsigned int mx, const unsigned int my,
                        const unsigned int src_x, const unsigned int src_y);

    unsigned int cell_inflation_radius_;
    unsigned int cached_cell_inflation_radius_;
    std::map<double, std::vector<CellData>> inflation_cells_;

    dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>* dsrv_;

    bool* seen_;
    unsigned int seen_size_;

    unsigned char** cached_costs_;
    double** cached_distances_;
    double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

    void reconfigureCB(costmap_2d::InflationPluginConfig& config, uint32_t level);

    bool need_reinflation_;  ///< Indicates that the entire costmap should be reinflated next time around.
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_INFLATION_LAYER_H_
