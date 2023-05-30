#ifndef GRIDMAP_COMPRESSED_DEPTH_DATA_H
#define GRIDMAP_COMPRESSED_DEPTH_DATA_H

#include <gridmap/layers/obstacle_data/data_source.h>
#include <gridmap/operations/raytrace.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

namespace gridmap {

class CompressedDepthData
    : public TopicDataSource<sensor_msgs::msg::CompressedImage> {
public:
  CompressedDepthData();
  virtual ~CompressedDepthData() override;

  virtual void onInitialize(const YAML::Node &parameters) override;
  virtual void onMapDataChanged() override;
  virtual bool isDataOk() const override;

protected:
  virtual bool
  processData(const sensor_msgs::msg::CompressedImage::SharedPtr msg,
              const Eigen::Isometry2d &robot_pose,
              const Eigen::Isometry3d &sensor_transform) override;

private:
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  std::string camera_info_topic_;
  // rclcpp::Subscriber camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      camera_info_sub_;
  // camera_info_sub_ = nullptr;
  std::mutex camera_info_mutex_;
  std::atomic_bool got_camera_info_;

  double hit_probability_log_;
  double miss_probability_log_;

  double obstacle_height_;
  float min_range_;
  float max_range_;

  std::string image_mask_;
  std::unique_ptr<cv::Mat> cv_image_mask_;

  image_geometry::PinholeCameraModel camera_model_;
};

} // namespace gridmap

#endif
