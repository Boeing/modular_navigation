#ifndef GRIDMAP_RAYTRACE_H
#define GRIDMAP_RAYTRACE_H

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

#include <ros/ros.h>

namespace gridmap
{

class AddLogCostLookup
{
  public:
    AddLogCostLookup(double* map_data, const double* log_cost_lookup, const double clamping_thres_min_log, const double clamping_thres_max_log)
        : map_data_(map_data), log_cost_lookup_(log_cost_lookup), clamping_thres_min_log_(clamping_thres_min_log),
          clamping_thres_max_log_(clamping_thres_max_log)
    {
    }
    inline void operator()(unsigned int offset, const int i)
    {
        map_data_[offset] = std::max(clamping_thres_min_log_, std::min(clamping_thres_max_log_, map_data_[offset] + log_cost_lookup_[i]));
    }

  private:
    double* map_data_;
    const double* log_cost_lookup_;
    double clamping_thres_min_log_;
    double clamping_thres_max_log_;
};

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

inline std::vector<Eigen::Array2i> drawLine(const Eigen::Array2i& start, const Eigen::Array2i& end)
{
    if ((start == end).all())
        return {end};

    double x1 = start.x();
    double x2 = end.x();

    if (x1 > x2)
    {
        return drawLine(end, start);
    }

    double y1 = start.y();
    double y2 = end.y();

    const bool steep = (std::abs(y2 - y1) > std::abs(x2 - x1));
    if (steep)
    {
        std::swap(x1, y1);
        std::swap(x2, y2);
    }

    const double dx = x2 - x1;
    const double dy = std::abs(y2 - y1);

    double error = dx / 2.0;
    const int ystep = (y1 < y2) ? 1 : -1;
    int y = static_cast<int>(y1);

    const int max_x = static_cast<int>(x2);

    std::vector<Eigen::Array2i> line;
    for (int x = static_cast<int>(x1); x < max_x; ++x)
    {
        if (steep)
        {
            line.push_back({y, x});
        }
        else
        {
            line.push_back({x, y});
        }

        error -= dy;
        if (error < 0)
        {
            y += ystep;
            error += dx;
        }
    }

    return line;
}

inline void clipRayEnd(const Eigen::Array2i& start, Eigen::Array2i& end, const Eigen::Array2i& size)
{
    const Eigen::Array2i dir = end - start;

    if (end.x() < 0)
    {
        const double t = static_cast<double>(start.x()) / dir.x();
        end.x() = 0;
        end.y() = start.y() + dir.y() * t;
    }

    if (end.y() < 0)
    {
        const double t = static_cast<double>(start.y()) / dir.y();
        end.x() = start.x() + dir.x() * t;
        end.y() = 0;
    }

    if (end.x() >= size.x())
    {
        const double t = static_cast<double>(size.x() - 1 - start.x()) / dir.x();
        end.x() = size.x() - 1;
        end.y() = start.y() + dir.y() * t;
    }

    if (end.y() >= size.y())
    {
        const double t = static_cast<double>(size.y() - 1 - start.y()) / dir.y();
        end.x() = start.x() + dir.x() * t;
        end.y() = size.y() - 1;
    }
}
}

#endif
