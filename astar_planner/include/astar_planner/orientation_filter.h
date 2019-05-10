#ifndef ASTAR_PLANNER_ORIENTATION_FILTER_H
#define ASTAR_PLANNER_ORIENTATION_FILTER_H

#include <Eigen/Geometry>

#include <vector>

namespace astar_planner
{

enum OrientationMode
{
    NONE,
    FORWARD,
    INTERPOLATE,
    FORWARDTHENINTERPOLATE,
    BACKWARD,
    LEFTWARD,
    RIGHTWARD
};

class OrientationFilter
{
  public:
    OrientationFilter() : omode_(NONE), window_size_(1)
    {
    }

    void processPath(std::vector<Eigen::Isometry2d>& path) const;

    void setAngleBasedOnPositionDerivative(std::vector<Eigen::Isometry2d>& path, const std::size_t index) const;

    void interpolate(std::vector<Eigen::Isometry2d>& path, const std::size_t start_index, const std::size_t end_index) const;

    void setMode(OrientationMode new_mode)
    {
        omode_ = new_mode;
    }

    void setMode(int new_mode)
    {
        setMode((OrientationMode)new_mode);
    }

    void setWindowSize(const std::size_t window_size)
    {
        window_size_ = window_size;
    }

  protected:
    OrientationMode omode_;
    std::size_t window_size_;
};
}

#endif
