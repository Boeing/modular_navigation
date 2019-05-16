#include <gridmap/params.h>
#include <gridmap/plugins/time_decay.h>

#include <pluginlib/class_list_macros.h>

#include <opencv2/highgui.hpp>

#include <ros/ros.h>
#include <chrono>

PLUGINLIB_EXPORT_CLASS(gridmap::TimeDecay, gridmap::DataSource)

namespace gridmap
{

TimeDecay::TimeDecay()
{
}

void TimeDecay::onInitialize(const XmlRpc::XmlRpcValue& parameters)
{
    frequency_ = get_config_with_default_warn<double>(parameters, "frequency", 1.0, XmlRpc::XmlRpcValue::TypeDouble);
    log_odds_decay_ = get_config_with_default_warn<double>(parameters, "log_odds_decay", 0.1, XmlRpc::XmlRpcValue::TypeDouble);

    time_decay_thread_ = std::thread(&TimeDecay::timeDecayThread, this, frequency_, log_odds_decay_);
}

TimeDecay::~TimeDecay()
{
    running_ = false;
    if (time_decay_thread_.joinable())
        time_decay_thread_.join();
}

void TimeDecay::matchSize()
{
}

void TimeDecay::timeDecayThread(const double frequency, const double log_odds_decay)
{
    while (running_ && ros::ok())
    {

    }
}

}
