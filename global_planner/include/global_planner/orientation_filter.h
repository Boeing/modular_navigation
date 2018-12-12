#ifndef GLOBAL_PLANNER_ORIENTATION_FILTER_H
#define GLOBAL_PLANNER_ORIENTATION_FILTER_H

#include <nav_msgs/Path.h>

namespace global_planner
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
    OrientationFilter() : omode_(NONE)
    {
    }

    virtual void processPath(std::vector<geometry_msgs::PoseStamped>& path);

    void setAngleBasedOnPositionDerivative(std::vector<geometry_msgs::PoseStamped>& path, const std::size_t index);
    void interpolate(std::vector<geometry_msgs::PoseStamped>& path, const std::size_t start_index,
                     const std::size_t end_index);

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

}  // end namespace global_planner
#endif
