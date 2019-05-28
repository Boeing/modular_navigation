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
    AddLogCostLookup(double* map_data,
            const double* log_cost_lookup,
            const double clamping_thres_min_log,
            const double clamping_thres_max_log,
            const int size_x,
            const Eigen::Array2i origin)
        : map_data_(map_data), log_cost_lookup_(log_cost_lookup), clamping_thres_min_log_(clamping_thres_min_log),
          clamping_thres_max_log_(clamping_thres_max_log), size_x_(size_x), origin_(origin)
    {
    }
    inline void operator()(unsigned int offset, const int i)
    {
        const unsigned int my = offset / size_x_;
        const unsigned int mx = offset - (my * size_x_);

        const Eigen::Vector2i diff = (Eigen::Array2i(mx, my) - origin_);
        const int dist = diff.norm();
        map_data_[offset] = std::max(clamping_thres_min_log_, std::min(clamping_thres_max_log_, map_data_[offset] + log_cost_lookup_[dist]));
    }

  private:
    double* map_data_;
    const double* log_cost_lookup_;
    double clamping_thres_min_log_;
    double clamping_thres_max_log_;
    const unsigned int size_x_;
    const Eigen::Array2i origin_;
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

// THE EXTREMELY FAST LINE ALGORITHM Variation E (Addition Fixed Point PreCalc)
inline std::vector<Eigen::Array2i> drawLine(int x, int y, int x2, int y2)
{
    std::vector<Eigen::Array2i> line;
    bool yLonger=false;
    int shortLen=y2-y;
    int longLen=x2-x;
    if (abs(shortLen)>abs(longLen)) {
        int swap=shortLen;
        shortLen=longLen;
        longLen=swap;
        yLonger=true;
    }
    int decInc;
    if (longLen==0) decInc=0;
    else decInc = (shortLen << 16) / longLen;

    if (yLonger) {
        if (longLen>0) {
            longLen+=y;
            for (int j=0x8000+(x<<16);y<=longLen;++y) {
                line.push_back({j >> 16,y});
                j+=decInc;
            }
            return line;
        }
        longLen+=y;
        for (int j=0x8000+(x<<16);y>=longLen;--y) {
            line.push_back({j >> 16,y});
            j-=decInc;
        }
        return line;
    }

    if (longLen>0) {
        longLen+=x;
        for (int j=0x8000+(y<<16);x<=longLen;++x) {
            line.push_back({x,j >> 16});
            j+=decInc;
        }
        return line;
    }
    longLen+=x;
    for (int j=0x8000+(y<<16);x>=longLen;--x) {
        line.push_back({x,j >> 16});
        j-=decInc;
    }
    return line;
}

}

#endif
