#ifndef GRIDMAP_DATA_SOURCE_H
#define GRIDMAP_DATA_SOURCE_H

#include <gridmap/grids/probability_grid.h>
#include <gridmap/operations/rasterize.h>
#include <gridmap/robot_tracker.h>
#include <gridmap/urdf_tree.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/subscription_queue.h>
#include <yaml-cpp/yaml.h>

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

inline Eigen::Isometry2d convert(const geometry_msgs::Transform& tr)
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
                            const std::shared_ptr<URDFTree>& urdf_tree) = 0;
    virtual void setMapData(const std::shared_ptr<ProbabilityGrid>& map_data) = 0;
    virtual std::string name() const = 0;
    virtual bool isDataOk() const = 0;
};

class SizedCallbackQueue : public ros::CallbackQueue
{
  public:
    SizedCallbackQueue() : ros::CallbackQueue()
    {
    }
    virtual ~SizedCallbackQueue() = default;

    size_t size() const
    {
        return callbacks_.size();
    }

    void flushMessages()
    {
        boost::mutex::scoped_lock id_lock(id_info_mutex_);
        boost::mutex::scoped_lock m_lock(mutex_);
        for (const ros::CallbackQueue::CallbackInfo& cb : callbacks_)
        {
            auto sq = boost::dynamic_pointer_cast<ros::SubscriptionQueue>(cb.callback);
            sq->clear();
        }
        callbacks_.clear();
    }
};

template <typename MsgType> class TopicDataSource : public DataSource
{
  public:
    TopicDataSource(const std::string& default_topic) : default_topic_(default_topic), last_updated_(ros::Time(0))
    {
    }
    virtual ~TopicDataSource()
    {
    }

    virtual void initialize(const std::string& name, const YAML::Node& parameters,
                            const std::vector<Eigen::Vector2d>& robot_footprint,
                            const std::shared_ptr<RobotTracker>& robot_tracker,
                            const std::shared_ptr<URDFTree>& urdf_tree) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        name_ = name;
        map_data_ = nullptr;
        robot_footprint_ = robot_footprint;
        robot_tracker_ = robot_tracker;
        urdf_tree_ = urdf_tree;
        last_updated_ = ros::Time::now();

        maximum_sensor_delay_ = parameters["maximum_sensor_delay"].as<double>(1.0);
        sub_sample_ = parameters["sub_sample"].as<int>(0);

        onInitialize(parameters);

        last_updated_ = ros::Time::now();

        const std::string _topic = parameters["topic"].as<std::string>(name_ + "/" + default_topic_);

        ROS_INFO_STREAM("Subscribing to: " << _topic);

        sub_opts_ = ros::SubscribeOptions::create<MsgType>(_topic, callback_queue_size_,
                                                           boost::bind(&TopicDataSource::callback, this, _1),
                                                           ros::VoidPtr(), &data_queue_);
        //        opts.transport_hints = ros::TransportHints();
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
        const double delay = (ros::Time::now() - last_updated_).toSec();
        return delay < maximum_sensor_delay_;
    }

  protected:
    virtual void onInitialize(const YAML::Node& parameters) = 0;
    virtual void onMapDataChanged() = 0;
    virtual bool processData(const typename MsgType::ConstPtr& msg, const Eigen::Isometry2d& robot_pose,
                             const Eigen::Isometry3d& sensor_transform) = 0;

    mutable std::mutex mutex_;

    std::string name_;
    std::string default_topic_;

    std::shared_ptr<ProbabilityGrid> map_data_;
    std::vector<Eigen::Vector2d> robot_footprint_;
    std::shared_ptr<RobotTracker> robot_tracker_;
    std::shared_ptr<URDFTree> urdf_tree_;

    mutable std::mutex last_updated_mutex_;
    ros::Time last_updated_;

    double maximum_sensor_delay_ = 0;
    int sub_sample_ = 0;
    int sub_sample_count_ = 0;

    const size_t callback_queue_size_ = 100;

    std::thread data_thread_;
    SizedCallbackQueue data_queue_;
    ros::SubscribeOptions sub_opts_;
    ros::Subscriber subscriber_;

    std::unordered_map<std::string, Eigen::Isometry3d> transform_cache_;

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

    void callback(const typename MsgType::ConstPtr& msg)
    {
        if (sub_sample_ == 0 || (sub_sample_ > 0 && sub_sample_count_ > sub_sample_))
        {
            sub_sample_count_ = 0;

            std::lock_guard<std::mutex> lock(mutex_);
            if (!map_data_)
                return;

            const double delay = (ros::Time::now() - msg->header.stamp).toSec();
            if (delay > maximum_sensor_delay_)
            {
                ROS_WARN_STREAM("DataSource '" << name_ << "' incoming data is " << delay << "s old!");
            }

            const Eigen::Isometry3d sensor_tr = getSensorTransform(msg->header.frame_id);
            const RobotState robot_state = robot_tracker_->robotState(msg->header.stamp);
            const Eigen::Isometry2d robot_pose = robot_state.map_to_odom * robot_state.odom.pose;
            const Eigen::Isometry3d tr = embed3d(robot_pose) * sensor_tr;

            const bool success = processData(msg, robot_pose, tr);
            if (!success)
            {
                ROS_ERROR_STREAM("Failed to process data for '" << name_ << "'");
            }
            else
            {
                std::lock_guard<std::mutex> l(last_updated_mutex_);
                last_updated_ = msg->header.stamp;
            }
        }
        else
        {
            ++sub_sample_count_;
        }
    }

    void dataThread()
    {
        ros::NodeHandle g_nh;

        bool connected = false;

        while (ros::ok())
        {
            try
            {
                if (!robot_tracker_->localised())
                {
                    if (connected)
                    {
                        ROS_INFO_STREAM("Disconnecting data for: " << name_);
                        subscriber_.shutdown();
                        data_queue_.flushMessages();
                        data_queue_.clear();
                        connected = false;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    continue;
                }

                if (!connected)
                {
                    ROS_INFO_STREAM("Connecting data for: " << name_);
                    subscriber_ = g_nh.subscribe(sub_opts_);
                    connected = true;
                }

                const auto t0 = std::chrono::steady_clock::now();
                const auto result = data_queue_.callOne(ros::WallDuration(maximum_sensor_delay_));
                const double duration =
                    std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0)
                        .count();

                std::lock_guard<std::mutex> lock(mutex_);
                if (result == ros::CallbackQueue::CallOneResult::Called)
                {
                    if (duration > maximum_sensor_delay_)
                    {
                        ROS_WARN_STREAM("DataSource '" << name_ << "' update took: " << duration
                                                       << "s. maximum_sensor_delay is: " << maximum_sensor_delay_
                                                       << "\nConsider compiling with optimisation flag -O2.");
                    }

                    if (data_queue_.size() == callback_queue_size_)
                    {
                        ROS_WARN_STREAM("DataSource '" << name_ << "' callback queue is full!");
                    }
                }
                else if (result == ros::CallbackQueue::CallOneResult::Empty)
                {
                    std::lock_guard<std::mutex> l(last_updated_mutex_);
                    const double delay = (ros::Time::now() - last_updated_).toSec();
                    if (delay > maximum_sensor_delay_)
                    {
                        ROS_WARN_STREAM("DataSource '" << name_ << "' has not updated for " << delay << "s");
                    }
                }
                else
                {
                    ROS_WARN_STREAM("DataSource '" << name_ << "' queue error");
                }
            }
            catch (const std::exception& e)
            {
                ROS_ERROR_STREAM("DataSource '" << name_ << "': " << e.what());
                data_queue_.flushMessages();
                data_queue_.clear();
            }
        }
    }
};

}  // namespace gridmap

#endif
