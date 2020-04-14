#ifndef NAVIGATION_INTERFACE_PARAMS
#define NAVIGATION_INTERFACE_PARAMS

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

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

inline std::vector<Eigen::Vector2d> get_point_list(const YAML::Node& parameters, const std::string& param_name,
                                                   const std::vector<Eigen::Vector2d>& default_value)
{
    std::vector<Eigen::Vector2d> result;
    if (parameters[param_name])
    {
        const YAML::Node& value = parameters[param_name];
        if (value.Type() != YAML::NodeType::Sequence)
        {
            throw std::runtime_error(param_name + " has incorrect type, expects a Sequence");
        }
        for (YAML::const_iterator it = value.begin(); it != value.end(); ++it)
        {
            ROS_ASSERT(it->IsSequence());
            if (it->Type() != YAML::NodeType::Sequence)
            {
                throw std::runtime_error(param_name + " element has incorrect type, expects a Sequence");
            }
            else if (it->size() == 2)
            {
                throw std::runtime_error(param_name + " element has incorrect size, expects a TypeArray");
            }
            else
            {
                result.push_back(
                    {static_cast<double>((*it)[0].as<double>()), static_cast<double>((*it)[1].as<double>())});
            }
        }
    }
    else
    {
        return default_value;
    }
    return result;
}
}  // namespace navigation_interface

#endif
