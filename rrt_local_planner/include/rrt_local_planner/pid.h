#ifndef RRT_LOCAL_PLANNER_PID_H
#define RRT_LOCAL_PLANNER_PID_H

#include <string>
#include <ros/time.h>

namespace rrt_local_planner
{

class PID
{
  public:
    struct Gains
    {
        double p_gain;  // Proportional gain
        double i_gain;  // Integral gain
        double d_gain;  // Derivative gain
        double i_min;  //Minimum allowable integral term
        double i_max;  // Maximum allowable integral term
        bool antiwindup;  //Antiwindup
    };

    PID(double p = 0.0, double i = 0.0, double d = 0.0, double i_min = 0.0, double i_max = 0.0, bool antiwindup = false);

    void reset();
    double compute(const double error, const double duration);

  private:
    Gains gains_;

    double p_error_last_;
    double p_error_;
    double i_error_;
    double d_error_;
    double cmd_;
};

}

#endif
