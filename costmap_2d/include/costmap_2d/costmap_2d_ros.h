#ifndef COSTMAP_2D_COSTMAP_2D_ROS_H_
#define COSTMAP_2D_COSTMAP_2D_ROS_H_

#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

#include <geometry_msgs/PoseStamped.h>

#include <atomic>
#include <memory>
#include <thread>

#include <pluginlib/class_loader.hpp>

#include <tf2/LinearMath/Transform.h>

namespace costmap_2d
{

class Costmap2DROS
{
  public:
    Costmap2DROS(const std::string& name, tf2_ros::Buffer& tf);
    Costmap2DROS(const Costmap2DROS&) = delete;
    ~Costmap2DROS();

    void start();
    void stop();

    void pause();
    void resume();

    void updateMap();

    void resetLayers();

    bool isCurrent()
    {
        return layered_costmap_->isCurrent();
    }

    bool getRobotPose(geometry_msgs::PoseStamped& global_pose) const;

    std::string getName() const
    {
        return name_;
    }

    double getTransformTolerance() const
    {
        return transform_tolerance_;
    }

    std::shared_ptr<Costmap2D> getCostmap()
    {
        return layered_costmap_->getCostmap();
    }

    const std::shared_ptr<const Costmap2D> getCostmap() const
    {
        return layered_costmap_->getCostmap();
    }

    std::string getGlobalFrameID() const
    {
        return global_frame_;
    }

    std::string getBaseFrameID() const
    {
        return robot_base_frame_;
    }

    LayeredCostmap* getLayeredCostmap()
    {
        return layered_costmap_.get();
    }

  protected:
    std::shared_ptr<LayeredCostmap> layered_costmap_;

    std::string name_;
    tf2_ros::Buffer& tf_;
    std::string global_frame_;
    std::string robot_base_frame_;
    double transform_tolerance_;

  private:
    void movementCB(const ros::TimerEvent& event);
    void mapUpdateLoop(double frequency);

    bool stop_updates_;
    bool initialized_;
    bool stopped_;
    bool robot_stopped_;

    std::shared_ptr<std::thread> map_update_thread_;
    std::atomic<bool> map_update_thread_shutdown_;

    ros::Timer timer_;
    ros::Time last_publish_;
    ros::Duration publish_cycle;

    pluginlib::ClassLoader<Layer> plugin_loader_;
    geometry_msgs::PoseStamped old_pose_;

    std::shared_ptr<Costmap2DPublisher> publisher_;
};
}

#endif
