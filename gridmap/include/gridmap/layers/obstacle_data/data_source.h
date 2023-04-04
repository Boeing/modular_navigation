#ifndef GRIDMAP_DATA_SOURCE_H
#define GRIDMAP_DATA_SOURCE_H

#include <gridmap/grids/probability_grid.h>
#include <gridmap/operations/rasterize.h>
#include <gridmap/robot_tracker.h>
#include <gridmap/urdf_tree.h>
//#include <ros/callback_queue.h>
#include "rclcpp/future_return_code.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
//#include <ros/subscription_queue.h> //This is handled by a QoS profile now
#include <yaml-cpp/yaml.h>

#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

namespace gridmap
{

inline uint64_t IndexToKey(const Eigen::Array2i& index)
{
    uint64_t k_0(static_cast<uint32_t>(index[0]));
    uint64_t k_1(static_cast<uint32_t>(index[1]));
    return (k_0 << 32) | k_1;
}

inline Eigen::Array2i KeyToIndex(const uint64_t& key)
{
    return Eigen::Array2i(static_cast<int32_t>((key >> 32) & 0xFFFFFFFF), static_cast<int32_t>(key & 0xFFFFFFFF));
}

inline Eigen::Isometry3d embed3d(const Eigen::Isometry2d& pose)
{
    return Eigen::Translation3d(pose.translation().x(), pose.translation().y(), 0.0) *
           Eigen::AngleAxisd(Eigen::Rotation2Dd(pose.rotation()).angle(), Eigen::Vector3d::UnitZ());
}

inline Eigen::Isometry2d convert(const geometry_msgs::msg::Transform& tr)
{
    const double yaw = std::atan2(2.0 * (tr.rotation.z * tr.rotation.w + tr.rotation.x * tr.rotation.y),
                                  -1.0 + 2.0 * (tr.rotation.w * tr.rotation.w + tr.rotation.x * tr.rotation.x));
    return Eigen::Translation2d(tr.translation.x, tr.translation.y) * Eigen::Rotation2Dd(yaw);
}

inline std::set<uint64_t> buildFootprintSet(const MapDimensions& dimensions, const Eigen::Isometry2d& robot_pose,
                                            const std::vector<Eigen::Vector2d>& footprint, const double scale = 1.0)
{
    int min_x = std::numeric_limits<int>::max();
    int max_x = 0;

    int min_y = std::numeric_limits<int>::max();
    int max_y = 0;

    std::vector<Eigen::Array2i> map_footprint;
    for (const Eigen::Vector2d& p : footprint)
    {
        const Eigen::Vector2d world_point = robot_pose.translation() + robot_pose.linear() * (scale * p);
        const auto map_point = dimensions.getCellIndex(world_point);
        map_footprint.push_back(map_point);
        min_x = std::min(map_point.x(), min_x);
        max_x = std::max(map_point.x(), max_x);
        min_y = std::min(map_point.y(), min_y);
        max_y = std::max(map_point.y(), max_y);
    }
    map_footprint.push_back(map_footprint.front());

    const std::vector<Eigen::Array2i> connected_poly = connectPolygon(map_footprint);

    std::set<uint64_t> footprint_set;
    auto insert_set = [&footprint_set](const int x, const int y) { footprint_set.insert(IndexToKey({x, y})); };

    gridmap::rasterPolygonFill(insert_set, connected_poly, min_x, max_x, min_y, max_y);

    // rasterPolygonFill is not properly including all edges
    for (const auto p : connected_poly)
        footprint_set.insert(IndexToKey({p.x(), p.y()}));

    return footprint_set;
}

class DataSource
{
  public:
    DataSource() = default;
    virtual ~DataSource() = default;

    virtual void initialize(const std::string& name, const YAML::Node& parameters,
                            const std::vector<Eigen::Vector2d>& robot_footprint,
                            const std::shared_ptr<RobotTracker>& robot_tracker,
                            const std::shared_ptr<URDFTree>& urdf_tree,
                            const rclcpp::Node::SharedPtr node) = 0;
    virtual void setMapData(const std::shared_ptr<ProbabilityGrid>& map_data) = 0;
    virtual std::string name() const = 0;
    virtual bool isDataOk() const = 0;
};

template <typename MsgType> class TopicDataSource : public DataSource
{
  public:
    TopicDataSource(const std::string& default_topic)
        : default_topic_(default_topic)
    {
    }
    virtual ~TopicDataSource()
    {
    }

    virtual void initialize(const std::string& name, const YAML::Node& parameters,
                            const std::vector<Eigen::Vector2d>& robot_footprint,
                            const std::shared_ptr<RobotTracker>& robot_tracker,
                            const std::shared_ptr<URDFTree>& urdf_tree,
                            const rclcpp::Node::SharedPtr node) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        name_ = name;
        map_data_ = nullptr;
        robot_footprint_ = robot_footprint;
        robot_tracker_ = robot_tracker;
        urdf_tree_ = urdf_tree;
        last_updated_ = node->get_clock()->now();
        node_ = node;

        maximum_sensor_delay_ = parameters["maximum_sensor_delay"].as<double>(1.0);
        sub_sample_ = parameters["sub_sample"].as<int>(0);

        onInitialize(parameters);

        last_updated_ = node->get_clock()->now();

        const std::string _topic = parameters["topic"].as<std::string>(name_ + "/" + default_topic_);
//        const std::string _topic = parameters["topic"].as<std::string>("/" + name_);

        // sub_opts_ = ros::SubscribeOptions::create<MsgType>(_topic, callback_queue_size_,
        //                                                    boost::bind(&TopicDataSource::callback, this, _1),
        //                                                    ros::VoidPtr(), &data_queue_);

        // create a mutually exclusive callback group and add it to a sub_options object
        auto cbg = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);  // V 0.1
        rclcpp::SubscriptionOptions sub_opts_;
        sub_opts_.callback_group = cbg;

        RCLCPP_INFO_STREAM(rclcpp::get_logger(""), "Subscribing to: " << _topic);
        // Create a callback group for the node
        // sub_opts_.callback_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // auto subscriber_ = node_->create_subscription<MsgType>(_topic, rclcpp::SensorDataQoS(),
        // boost::bind(&TopicDataSource::callback, node_, _1), sub_opts_);

        auto cb_func = [this](const typename MsgType::SharedPtr msg) { this->bufferIncomingMsg(msg); };
        subscriber_ = node_->create_subscription<MsgType>(_topic,
                                                                 rclcpp::SensorDataQoS(),  // callback_queue_size_,
                                                          std::bind(&TopicDataSource<MsgType>::bufferIncomingMsg, this, std::placeholders::_1),
                                                                 sub_opts_);

        // opts.transport_hints = ros::TransportHints();
        data_thread_ = std::thread(&TopicDataSource<MsgType>::dataThread, this);
    }

    virtual void setMapData(const std::shared_ptr<ProbabilityGrid>& map_data) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        map_data_ = map_data;
        onMapDataChanged();
    }

    virtual std::string name() const override
    {
        return name_;
    }

    virtual bool isDataOk() const override
    {
        std::lock_guard<std::mutex> lock(last_updated_mutex_);
        const double delay = (node_->get_clock()->now() - last_updated_).seconds();  // toSec()
        return delay < maximum_sensor_delay_;
    }

  protected:
    virtual void onInitialize(const YAML::Node& parameters) = 0;
    virtual void onMapDataChanged() = 0;
    virtual bool processData(const typename MsgType::SharedPtr msg, const Eigen::Isometry2d& robot_pose,
                             const Eigen::Isometry3d& sensor_transform) = 0;

    mutable std::mutex mutex_;

    std::string name_;
    std::string default_topic_;

    std::shared_ptr<ProbabilityGrid> map_data_;
    std::vector<Eigen::Vector2d> robot_footprint_;
    std::shared_ptr<RobotTracker> robot_tracker_;
    std::shared_ptr<URDFTree> urdf_tree_;

    mutable std::mutex last_updated_mutex_;
    rclcpp::Time last_updated_;

    double maximum_sensor_delay_ = 0;
    int sub_sample_ = 0;
    int sub_sample_count_ = 0;

    const size_t callback_queue_size_ = 100;

    std::thread data_thread_;

    // SizedCallbackQueue data_queue_;

    std::list<typename MsgType::SharedPtr> msg_buffer_;
    mutable std::mutex msg_buffer_mutex_;

    bool connected_ = false;
    mutable std::mutex connected_mutex_;

    std::unordered_map<std::string, Eigen::Isometry3d> transform_cache_;

    rclcpp::Node::SharedPtr node_;
    typename rclcpp::Subscription<MsgType>::SharedPtr subscriber_;

    std::promise<rclcpp::FutureReturnCode> promise_;

  private:
    Eigen::Isometry3d getSensorTransform(const std::string& frame_name)
    {
        auto it = transform_cache_.find(frame_name);
        Eigen::Isometry3d output;
        if (it == transform_cache_.end())
        {
            output = urdf_tree_->getTransform(frame_name, "base_link");
            bool success;
            std::tie(it, success) = transform_cache_.insert({frame_name, output});
        }
        else
        {
            output = it->second;
        }
        return output;
    }

    void bufferIncomingMsg(const typename MsgType::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock_c(connected_mutex_);
        if (connected_)
        {
            std::lock_guard<std::mutex> lock(msg_buffer_mutex_);
            msg_buffer_.push_back(msg);  // Add message to buffer
        }
    }

    void processMsg(const typename MsgType::SharedPtr msg)
    /*
    TODO:
        - Instead of using raw clock info, use futures.wait_for(seconds)
        - status == std::future_status::ready can be used to trigger warnings

    BE AWARE:
        Can this become a recursive spin call and reproduce this open issue?
        https://github.com/ros2/rclcpp/issues/773
    */
    {
        if (sub_sample_ == 0 || (sub_sample_ > 0 && sub_sample_count_ > sub_sample_))
        {
            sub_sample_count_ = 0;

            std::lock_guard<std::mutex> lock(mutex_);
            if (!map_data_)
                return;

            const double delay = (node_->get_clock()->now() - msg->header.stamp).seconds();
            if (delay > maximum_sensor_delay_)
            {
                RCLCPP_WARN_STREAM(rclcpp::get_logger(""),
                                   "DataSource '" << name_ << "' incoming data is " << delay << "s old!");
            }

            const Eigen::Isometry3d sensor_tr = getSensorTransform(msg->header.frame_id);
            const RobotState robot_state = robot_tracker_->robotState(msg->header.stamp);
            const Eigen::Isometry2d robot_pose = robot_state.map_to_odom * robot_state.odom.pose;
            const Eigen::Isometry3d tr = embed3d(robot_pose) * sensor_tr;

            const bool success = processData(msg, robot_pose, tr);

            if (!success)
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(""), "Failed to process data for '" << name_ << "'");
            }
            else
            {
                std::lock_guard<std::mutex> l(last_updated_mutex_);
                last_updated_ = msg->header.stamp;
            }
            RCLCPP_INFO_STREAM(rclcpp::get_logger(""), "SUCCEDED! to process data for '" << name_ << "'");//DEBUG
        }
        else
        {
            ++sub_sample_count_;
        }
    }

    void dataThread()
    {
        /*
        PREV:
            * threaded loop where:
                - if no connection then subscribe to topic
                - checks if connection is lost, if true execute above
                - last callback gets executed, waits for time if queue is empty
                - then some queue related error checking/logging happens
        */

        // ros::NodeHandle g_nh;

        // Node creation exposed to the whole class
        while (rclcpp::ok())
        {
            try
            {
                bool connected;
                {
                    std::lock_guard<std::mutex> lock_c(connected_mutex_);
                    connected = connected_;
                }

                // If not localised...
                if (!robot_tracker_->localised())
                {
                    if (connected)
                    {
                        RCLCPP_INFO_STREAM(rclcpp::get_logger(""), "Disconnecting data for: " << name_);
                        std::lock_guard<std::mutex> lock(connected_mutex_);
                        connected_ = false;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    continue;
                }

                // If localised, but not already connected...
                if (!connected)
                {
                    RCLCPP_INFO_STREAM(rclcpp::get_logger(""), "Connecting data for: " << name_);
                    // Clear stale data from message buffer
                    {
                        std::lock_guard<std::mutex> lock(msg_buffer_mutex_);
                        msg_buffer_.clear();
                    }
                    // Enable storing values to buffer
                    {
                        std::lock_guard<std::mutex> lock(connected_mutex_);
                        connected_ = true;
                    }
                    // Reset time buffer last updated
                    {
                        std::lock_guard<std::mutex> lock(last_updated_mutex_);
                        last_updated_ = node_->get_clock()->now();
                    }
                }

                // Create copy of message buffer to process
                bool is_buffer_empty;
                typename MsgType::SharedPtr queued_msg;
                {
                    std::lock_guard<std::mutex> lock(msg_buffer_mutex_);
                    is_buffer_empty = msg_buffer_.empty();
                    if (!is_buffer_empty)
                    {
                        queued_msg = msg_buffer_.front();
                        msg_buffer_.pop_front();
                    }
                }

                // Warn if sensor has not been published recently
                if (is_buffer_empty)
                {
                    std::lock_guard<std::mutex> l(last_updated_mutex_);
                    const double delay = (node_->get_clock()->now() - last_updated_).seconds();
                    if (delay > maximum_sensor_delay_)
                    {
                        RCLCPP_WARN_STREAM(rclcpp::get_logger(""),
                                           "DataSource '" << name_ << "' has not updated for " << delay << "s");
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    continue;
                }

                // Process queued message
                const auto t0 = std::chrono::steady_clock::now();
                processMsg(queued_msg);
                const double duration =
                    std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0)
                        .count();

                std::lock_guard<std::mutex> lock(mutex_);
                if (duration > maximum_sensor_delay_)
                {
                    RCLCPP_WARN_STREAM(rclcpp::get_logger(""),
                                       "DataSource '" << name_ << "' update took: " << duration
                                                      << "s. maximum_sensor_delay is: " << maximum_sensor_delay_
                                                      << "\nConsider compiling with optimisation flag -O2.");
                }
                // if (data_queue_.size() == callback_queue_size_)
                //{
                //     RCLCPP_WARN_STREAM(rclcpp::get_logger(""), "DataSource '" << name_ << "' callback queue is
                //     full!");
                // }
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger(""), "DataSource '" << name_ << "': " << e.what());
                // data_queue_.flushMessages();
                // data_queue_.clear();
            }
        }
    }
};

}  // namespace gridmap

#endif
