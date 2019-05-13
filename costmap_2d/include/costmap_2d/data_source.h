#ifndef COSTMAP_2D_LAYER_H_
#define COSTMAP_2D_LAYER_H_

#include <memory>
#include <string>

#include <costmap_2d/map_data.h>

#include <tf2_ros/buffer.h>

namespace costmap_2d
{

class DataSource
{
  public:
    DataSource() = default;
    virtual ~DataSource() = default;

    void initialize(const std::string& name,
                    const std::string& global_frame,
                    const XmlRpc::XmlRpcValue& parameters,
                    const std::shared_ptr<MapData>& map_data,
                    const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
    {
        name_ = name;
        global_frame_ = global_frame;
        map_data_ = map_data;
        tf_buffer_ = tf_buffer;
        onInitialize(parameters);
    }

    virtual void matchSize() = 0;

    std::string name() const
    {
        return name_;
    }

  protected:
    virtual void onInitialize(const XmlRpc::XmlRpcValue& parameters) = 0;

    std::string name_;
    std::string global_frame_;
    std::shared_ptr<MapData> map_data_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};
}

#endif
