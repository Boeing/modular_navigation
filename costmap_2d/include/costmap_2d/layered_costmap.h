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

class LayeredCostmap
{
  public:
    LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown);
    ~LayeredCostmap();

    void updateMap(const double robot_x, const double robot_y, const double robot_yaw);

    std::string getGlobalFrameID() const
    {
        return global_frame_;
    }

    void resizeMap(const unsigned int size_x, const unsigned int size_y, const double resolution, const double origin_x,
                   const double origin_y, const bool size_locked = false);

    bool isCurrent();

    std::shared_ptr<Costmap2D> getCostmap()
    {
        return costmap_;
    }

    const std::shared_ptr<const Costmap2D> getCostmap() const
    {
        return costmap_;
    }

    bool isRolling() const
    {
        return rolling_window_;
    }

    bool isTrackingUnknown() const
    {
        return costmap_->getDefaultValue() == costmap_2d::NO_INFORMATION;
    }

    std::vector<boost::shared_ptr<Layer>>* getPlugins()
    {
        return &plugins_;
    }

    void addPlugin(boost::shared_ptr<Layer> plugin)
    {
        plugins_.push_back(plugin);
    }

    void getBounds(unsigned int* x0, unsigned int* xn, unsigned int* y0, unsigned int* yn)
    {
        *x0 = bx0_;
        *xn = bxn_;
        *y0 = by0_;
        *yn = byn_;
    }

    bool isInitialized() const
    {
        return initialized_;
    }

  private:
    std::shared_ptr<Costmap2D> costmap_;
    std::string global_frame_;

    bool rolling_window_;

    bool current_;

    unsigned int update_count_;

    unsigned int bx0_;
    unsigned int bxn_;
    unsigned int by0_;
    unsigned int byn_;

    std::vector<boost::shared_ptr<Layer>> plugins_;

    bool initialized_;
};

}

#endif
