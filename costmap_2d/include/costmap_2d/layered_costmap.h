#ifndef COSTMAP_2D_LAYERED_COSTMAP_H_
#define COSTMAP_2D_LAYERED_COSTMAP_H_

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/layer.h>
#include <string>
#include <vector>

namespace costmap_2d
{
class Layer;

/**
 * @class LayeredCostmap
 * @brief Instantiates different layer plugins and aggregates them into one score
 */
class LayeredCostmap
{
  public:
    /**
     * @brief  Constructor for a costmap
     */
    LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown);

    /**
     * @brief  Destructor
     */
    ~LayeredCostmap();

    /**
     * @brief  Update the underlying costmap with new data.
     * If you want to update the map outside of the update loop that runs, you can call this.
     */
    void updateMap(double robot_x, double robot_y, double robot_yaw);

    std::string getGlobalFrameID() const
    {
        return global_frame_;
    }

    void resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x, double origin_y,
                   bool size_locked = false);

    bool isCurrent();

    Costmap2D* getCostmap()
    {
        return &costmap_;
    }

    bool isRolling()
    {
        return rolling_window_;
    }

    bool isTrackingUnknown()
    {
        return costmap_.getDefaultValue() == costmap_2d::NO_INFORMATION;
    }

    std::vector<boost::shared_ptr<Layer>>* getPlugins()
    {
        return &plugins_;
    }

    void addPlugin(boost::shared_ptr<Layer> plugin)
    {
        plugins_.push_back(plugin);
    }

    bool isSizeLocked()
    {
        return size_locked_;
    }

    void getBounds(unsigned int* x0, unsigned int* xn, unsigned int* y0, unsigned int* yn)
    {
        *x0 = bx0_;
        *xn = bxn_;
        *y0 = by0_;
        *yn = byn_;
    }

    bool isInitialized()
    {
        return initialized_;
    }

    /** @brief Updates the stored footprint, updates the circumscribed
     * and inscribed radii, and calls onFootprintChanged() in all
     * layers. */
    void setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec);

    /** @brief Returns the latest footprint stored with setFootprint(). */
    const std::vector<geometry_msgs::Point>& getFootprint()
    {
        return footprint_;
    }

    /** @brief The radius of a circle centered at the origin of the
     * robot which just surrounds all points on the robot's
     * footprint.
     *
     * This is updated by setFootprint(). */
    double getCircumscribedRadius()
    {
        return circumscribed_radius_;
    }

    /** @brief The radius of a circle centered at the origin of the
     * robot which is just within all points and edges of the robot's
     * footprint.
     *
     * This is updated by setFootprint(). */
    double getInscribedRadius()
    {
        return inscribed_radius_;
    }

  private:
    Costmap2D costmap_;
    std::string global_frame_;

    bool rolling_window_;  /// < @brief Whether or not the costmap should roll with the robot

    bool current_;

    unsigned int bx0_;
    unsigned int bxn_;
    unsigned int by0_;
    unsigned int byn_;

    std::vector<boost::shared_ptr<Layer>> plugins_;

    bool initialized_;
    bool size_locked_;
    double circumscribed_radius_, inscribed_radius_;
    std::vector<geometry_msgs::Point> footprint_;
};
}

#endif
