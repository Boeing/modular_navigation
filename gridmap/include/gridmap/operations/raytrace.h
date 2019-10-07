#ifndef GRIDMAP_RAYTRACE_H
#define GRIDMAP_RAYTRACE_H

#include <ros/ros.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace gridmap
{

class AddLogCost
{
  public:
    AddLogCost(double* map_data, const double log_cost, const double clamping_thres_min_log,
               const double clamping_thres_max_log)
        : map_data_(map_data), log_cost_(log_cost), clamping_thres_min_log_(clamping_thres_min_log),
          clamping_thres_max_log_(clamping_thres_max_log)
    {
    }
    inline void operator()(unsigned int offset, const int)
    {
        map_data_[offset] =
            std::max(clamping_thres_min_log_, std::min(clamping_thres_max_log_, map_data_[offset] + log_cost_));
    }

  private:
    double* map_data_;
    double log_cost_;
    double clamping_thres_min_log_;
    double clamping_thres_max_log_;
};

inline int sign(int x)
{
    return x > 0 ? 1.0 : -1.0;
}

template <class ActionType>
inline void raytraceLine(ActionType at, const unsigned int x0, const unsigned int y0, const unsigned int x1,
                         const unsigned int y1, const unsigned int size_x, const unsigned int max_length = UINT_MAX)
{
    int dx = x1 - x0;
    int dy = y1 - y0;

    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);

    int offset_dx = sign(dx);
    int offset_dy = sign(dy) * size_x;

    unsigned int offset = y0 * size_x + x0;

    // we need to chose how much to scale our dominant dimension, based on the maximum length of the line
    double dist = hypot(dx, dy);
    double scale = (dist == 0.0) ? 1.0 : std::min(1.0, max_length / dist);

    // if x is dominant
    if (abs_dx >= abs_dy)
    {
        int error_y = abs_dx / 2;
        bresenham2D(at, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset, (unsigned int)(scale * abs_dx));
        return;
    }

    // otherwise y is dominant
    int error_x = abs_dy / 2;
    bresenham2D(at, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int)(scale * abs_dy));
}

template <class ActionType>
inline void bresenham2D(ActionType at, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                        int offset_b, unsigned int offset, unsigned int max_length)
{
    unsigned int end = std::min(max_length, abs_da);
    for (unsigned int i = 0; i < end; ++i)
    {
        at(offset, i);
        offset += offset_a;
        error_b += abs_db;
        if ((unsigned int)error_b >= abs_da)
        {
            offset += offset_b;
            error_b -= abs_da;
        }
    }
    at(offset, end - 1);
}

std::vector<Eigen::Array2i> drawLine(int x, int y, int x2, int y2);
}  // namespace gridmap

#endif
