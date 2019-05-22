#ifndef GRIDMAP_DATA_SOURCE_H
#define GRIDMAP_DATA_SOURCE_H

#include <mutex>
#include <memory>
#include <string>

#include <gridmap/grids/probability_grid.h>

#include <tf2_ros/buffer.h>

namespace gridmap
{

class DataSource
{
  public:
    DataSource() = default;
    virtual ~DataSource() = default;

    void initialize(const std::string& name,
                    const std::string& global_frame,
                    const XmlRpc::XmlRpcValue& parameters,
                    const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        name_ = name;
        global_frame_ = global_frame;
        map_data_ = nullptr;
        tf_buffer_ = tf_buffer;
        onInitialize(parameters);
    }

    void setMapData(const std::shared_ptr<ProbabilityGrid>& map_data)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        map_data_ = map_data;
        onMapDataChanged();
    }

    std::string name() const
    {
        return name_;
    }

  protected:
    virtual void onInitialize(const XmlRpc::XmlRpcValue& parameters) = 0;
    virtual void onMapDataChanged() = 0;

    std::mutex mutex_;

    std::string name_;
    std::string global_frame_;
    std::shared_ptr<ProbabilityGrid> map_data_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};
}

#endif
