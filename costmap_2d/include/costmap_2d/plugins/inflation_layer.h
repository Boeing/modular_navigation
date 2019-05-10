#ifndef COSTMAP_2D_INFLATION_LAYER_H
#define COSTMAP_2D_INFLATION_LAYER_H

#include <costmap_2d/layer.h>

namespace costmap_2d
{

class InflationLayer : public Layer
{
  public:
    InflationLayer();

    virtual ~InflationLayer() override;

    virtual void onInitialize() override;

    virtual void activate() override;

    virtual void deactivate() override;

    virtual void reset() override;

    virtual void matchSize() override;

    virtual void updateBounds(const double, const double, const double, double* min_x, double* min_y, double* max_x,
                              double* max_y) override;

    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, unsigned int min_i, unsigned int min_j,
                             unsigned int max_i, unsigned int max_j) override;

  private:
    double inflation_radius_;
};
}

#endif
