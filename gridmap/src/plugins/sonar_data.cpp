#include <gridmap/params.h>
#include <gridmap/plugins/sonar_data.h>
#include <gridmap/raytrace.h>

#include <tf2_ros/message_filter.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/point_cloud2_iterator.h>

#include <opencv2/highgui.hpp>

#include <chrono>

PLUGINLIB_EXPORT_CLASS(gridmap::RangeData, gridmap::DataSource)

namespace gridmap
{

RangeData::RangeData()
{
}

void RangeData::onInitialize(const XmlRpc::XmlRpcValue& parameters)
{
    ros::NodeHandle nh("~/" + name_);
    ros::NodeHandle g_nh;

    const std::string topic =
        get_config_with_default_warn<std::string>(parameters, "topic", "/range", XmlRpc::XmlRpcValue::TypeString);
    hit_probability_log_ = logodds(
        get_config_with_default_warn<double>(parameters, "hit_probability", 0.7, XmlRpc::XmlRpcValue::TypeDouble));
    miss_probability_log_ = logodds(
        get_config_with_default_warn<double>(parameters, "miss_probability", 0.4, XmlRpc::XmlRpcValue::TypeDouble));
    min_obstacle_height_ =
        get_config_with_default_warn<double>(parameters, "min_obstacle_height", 0.0, XmlRpc::XmlRpcValue::TypeDouble);
    max_obstacle_height_ =
        get_config_with_default_warn<double>(parameters, "max_obstacle_height", 2.0, XmlRpc::XmlRpcValue::TypeDouble);
    obstacle_range_ =
        get_config_with_default_warn<double>(parameters, "obstacle_range", 2.5, XmlRpc::XmlRpcValue::TypeDouble);
    raytrace_range_ =
        get_config_with_default_warn<double>(parameters, "raytrace_range", 3.0, XmlRpc::XmlRpcValue::TypeDouble);
    sub_sample_ = get_config_with_default_warn<int>(parameters, "sub_sample", 10, XmlRpc::XmlRpcValue::TypeInt);

    ROS_INFO_STREAM("Subscribing to range sensor: " << topic);

    subscriber_.reset(new message_filters::Subscriber<sensor_msgs::Range>(g_nh, topic, 50));
    message_filter_.reset(
        new tf2_ros::MessageFilter<sensor_msgs::Range>(*subscriber_, *tf_buffer_, global_frame_, 50, g_nh));
    message_filter_->registerCallback(boost::bind(&RangeData::rangeCallback, this, _1));
}

RangeData::~RangeData()
{
}

void RangeData::rangeCallback(const sensor_msgs::RangeConstPtr& message)
{
    if (sub_sample_ == 0 || (sub_sample_ > 0 && sub_sample_count_ > sub_sample_))
    {

        sub_sample_count_ = 0;
    }
    else
        ++sub_sample_count_;
}

void RangeData::matchSize()
{
}

}
