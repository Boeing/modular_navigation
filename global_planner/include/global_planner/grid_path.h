#ifndef _GRID_PATH_H
#define _GRID_PATH_H

#include <global_planner/traceback.h>
#include <vector>

namespace global_planner
{

class GridPath : public Traceback
{
  public:
    GridPath(std::shared_ptr<PotentialCalculator> p_calc) : Traceback(p_calc)
    {
    }
    bool getPath(float* potential, double start_x, double start_y, double end_x, double end_y,
                 std::vector<std::pair<float, float>>& path);
};

}  // end namespace global_planner
#endif
