#ifndef NAVIGATION_INTERFACE_PARAMS
#define NAVIGATION_INTERFACE_PARAMS

#include <ros/ros.h>

#include <Eigen/Geometry>
#include <string>

namespace navigation_interface
{

template <typename T> T get_param_with_default(const std::string& param_name, const T& default_val)
{
    if (ros::param::has(param_name))
    {
        T param_val;
        if (ros::param::get(param_name, param_val))
        {
            return param_val;
        }
    }
    return default_val;
}

template <typename T>
T get_param_with_default_warn(ros::NodeHandle nh, const std::string& param_name, const T& default_val)
{
    if (nh.hasParam(param_name))
    {
        T param_val;
        if (nh.getParam(param_name, param_val))
        {
            return param_val;
        }
    }
    ROS_WARN_STREAM("Using default value for '" << nh.getNamespace() << "/" << param_name << "': '" << default_val
                                                << "'");
    return default_val;
}

template <typename T>
T get_config_with_default_warn(XmlRpc::XmlRpcValue parameters, const std::string& param_name, const T& default_val,
                               const XmlRpc::XmlRpcValue::Type& xml_type)
{
    if (parameters.hasMember(param_name))
    {
        XmlRpc::XmlRpcValue& value = parameters[param_name];

        if (value.getType() != xml_type)
        {
            throw std::runtime_error(param_name + " has incorrect type");
        }

        return T(value);
    }
    else
    {
        ROS_WARN_STREAM("Using default value for " << param_name << ": " << default_val);
        return default_val;
    }
}

template <typename T, size_t size>
std::array<T, size> get_config_list_with_default(XmlRpc::XmlRpcValue parameters, const std::string& param_name,
                                                 const std::array<T, size>& default_val,
                                                 const XmlRpc::XmlRpcValue::Type& xml_type)
{
    if (parameters.hasMember(param_name))
    {
        XmlRpc::XmlRpcValue& value = parameters[param_name];
        if (value.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            throw std::runtime_error(param_name + " has incorrect type, expects a TypeArray");
        }
        if (value.size() != size)
        {
            throw std::runtime_error(param_name + " has incorrect size");
        }
        std::array<T, size> params_array;
        for (int32_t i = 0; i < value.size(); ++i)
        {
            if (value[i].getType() != xml_type)
            {
                throw std::runtime_error(param_name + " element has incorrect type");
            }
            else
            {
                params_array[i] = static_cast<T>(value[i]);
            }
        }
        return params_array;
    }
    else
    {
        return default_val;
    }
}

inline std::vector<Eigen::Vector2d> get_point_list(XmlRpc::XmlRpcValue parameters, const std::string& param_name)
{
    std::vector<Eigen::Vector2d> result;
    if (parameters.hasMember(param_name))
    {
        XmlRpc::XmlRpcValue& value = parameters[param_name];
        if (value.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            throw std::runtime_error("parameters have incorrect type, expects a TypeArray");
        }
        for (int32_t i = 0; i < value.size(); ++i)
        {
            if (value[i].getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
                throw std::runtime_error("element have incorrect type, expects a TypeArray");
            }
            else if (value[i].size() == 2)
            {
                throw std::runtime_error("element has incorrect size, expects a TypeArray");
            }
            else
            {
                result.push_back({static_cast<double>(value[i][0]), static_cast<double>(value[i][1])});
            }
        }
    }
    return result;
}
}  // namespace navigation_interface

#endif
