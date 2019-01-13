#include <boost/algorithm/clamp.hpp>
#include <eband_local_planner/pid.h>

namespace eband_local_planner
{

PID::PID(double p, double i, double d, double i_min, double i_max, bool antiwindup)
    : gains_({p, i, d, i_min, i_max, antiwindup}), p_error_last_(0.0), p_error_(0.0), i_error_(0.0), d_error_(0.0),
      cmd_(0.0)
{
}

void PID::reset()
{
    p_error_last_ = 0.0;
    p_error_ = 0.0;
    i_error_ = 0.0;
    d_error_ = 0.0;
    cmd_ = 0.0;
}

double PID::compute(const double error, const double duration)
{
    assert(duration > 0);

    double error_dot = d_error_;

    // Calculate the derivative error
    if (duration > 0.0)
    {
        error_dot = (error - p_error_last_) / duration;
        p_error_last_ = error;
    }

    double p_term, d_term, i_term;
    p_error_ = error;  // this is error = target - state
    d_error_ = error_dot;

    if (std::isnan(error_dot) || std::isinf(error_dot))
        return 0.0;

    // Calculate proportional contribution to command
    p_term = gains_.p_gain * p_error_;

    // Calculate the integral of the position error
    i_error_ += duration * p_error_;

    if (gains_.antiwindup)
    {
        // Prevent i_error_ from climbing higher than permitted by i_max_/i_min_
        i_error_ = boost::algorithm::clamp(i_error_, gains_.i_min / std::abs(gains_.i_gain),
                                           gains_.i_max / std::abs(gains_.i_gain));
    }

    // Calculate integral contribution to command
    i_term = gains_.i_gain * i_error_;

    if (!gains_.antiwindup)
    {
        // Limit i_term so that the limit is meaningful in the output
        i_term = boost::algorithm::clamp(i_term, gains_.i_min, gains_.i_max);
    }

    // Calculate derivative contribution to command
    d_term = gains_.d_gain * d_error_;

    // Compute the command
    cmd_ = p_term + i_term + d_term;

    return cmd_;
}
}
