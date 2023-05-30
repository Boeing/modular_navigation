#include <navigation_interface/params.h>
#include <pluginlib/class_list_macros.hpp>
#include <sim_band_planner/plugin.h>

PLUGINLIB_EXPORT_CLASS(sim_band_planner::SimBandPlanner,
                       navigation_interface::TrajectoryPlanner)

namespace sim_band_planner {

namespace {

std_msgs::msg::ColorRGBA distanceToColor(const double distance) {
  std_msgs::msg::ColorRGBA color;
  color.a = 0.25f;

  if (distance <= 0) {
    color.r = 1.0f;
  } else if (distance < 0.1) {
    color.r = 1.0f;
    color.g = 0.5f;
  } else if (distance < 0.2) {
    color.r = 1.0f;
    color.g = 1.0f;
  } else {
    color.g = 1.0f;
  }
  return color;
}

} // namespace

SimBandPlanner::SimBandPlanner() {}

SimBandPlanner::~SimBandPlanner() {}

// cppcheck-suppress unusedFunction
bool SimBandPlanner::setPath(const navigation_interface::Path &path) {
  if (path.nodes.empty())
    return false;

  moving_window_.reset(new MovingWindow(path, offsets_));

  return true;
}

// cppcheck-suppress unusedFunction
void SimBandPlanner::clearPath() { moving_window_.reset(); }

// cppcheck-suppress unusedFunction
boost::optional<std::string> SimBandPlanner::pathId() const {
  if (moving_window_)
    return boost::optional<std::string>(moving_window_->nominal_path.id);
  else
    return {};
}

boost::optional<navigation_interface::Path> SimBandPlanner::path() const {
  if (moving_window_)
    return boost::optional<navigation_interface::Path>(
        moving_window_->nominal_path);
  else
    return {};
}

navigation_interface::TrajectoryPlanner::Result
// cppcheck-suppress unusedFunction
SimBandPlanner::plan(const gridmap::AABB &local_region,
                     const navigation_interface::KinodynamicState &robot_state,
                     const Eigen::Isometry2d &map_to_odom,
                     const double avoid_distance) {
  navigation_interface::TrajectoryPlanner::Result result;

  const double inflated_robot_radius =
      robot_radius_ + std::max(0.0, avoid_distance);

  if (!moving_window_) {
    result.outcome = navigation_interface::TrajectoryPlanner::Outcome::FAILED;
    return result;
  }

  try {
    // Take a window of the planning scene at the robot pose
    const Eigen::Isometry2d robot_pose = map_to_odom * robot_state.pose;

    moving_window_->updateWindow(robot_pose, max_window_length_);

    auto lock = map_data_->grid.getReadLock();
    gridmap::Grid2D<uint8_t> local_grid(map_data_->grid, local_region);
    lock.unlock();

    // Construct and cv::Mat from the costmap data
    const cv::Mat cv_im = cv::Mat(
        local_grid.dimensions().size().y(), local_grid.dimensions().size().x(),
        CV_8U, reinterpret_cast<void *>(local_grid.cells().data()));

    DistanceField distance_field(cv_im, local_grid.dimensions().origin().x(),
                                 local_grid.dimensions().origin().y(),
                                 local_grid.dimensions().resolution(),
                                 inflated_robot_radius);

    Band sim_band(offsets_);

    // Add the robot position to the front of the band
    sim_band.nodes.push_back(Node(robot_pose, sim_band.radius_offsets));

    // The first moving window node is the previous segment (exclude from
    // optimization)
    rcpputils::assert_true(moving_window_->window.nodes.size() >= 1);
    if (moving_window_->window.nodes.size() > 1)
      sim_band.nodes.insert(sim_band.nodes.end(),
                            moving_window_->window.nodes.begin(),
                            moving_window_->window.nodes.end());
    else
      sim_band.nodes.push_back(moving_window_->window.nodes.front());

    simulate(sim_band, distance_field, num_iterations_, collision_distance_,
             nominal_force_gain_, internal_force_gain_, external_force_gain_,
             rotation_gain_, velocity_decay_, 1.0, alpha_decay_);

    // debug viz
    if (debug_viz_ &&
        marker_pub_->get_subscription_count() > 0) // getNumSubscribers() in ROS
    {
      visualization_msgs::msg::MarkerArray ma;

      visualization_msgs::msg::Marker delete_all;
      delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
      ma.markers.push_back(delete_all);

      // const auto now = ros::Time::now();
      const auto now = node_->get_clock()->now(); //.nanoseconds()

      auto make_cylider = [&ma, &now](const std_msgs::msg::ColorRGBA &color,
                                      const double radius, const double height,
                                      const Eigen::Isometry3d &pose) {
        visualization_msgs::msg::Marker marker;
        marker.ns = "cylinder";
        marker.id = static_cast<int>(ma.markers.size());
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.header.stamp = now;
        marker.header.frame_id = "map";
        marker.frame_locked = true;
        marker.scale.x = radius;
        marker.scale.y = radius;
        marker.scale.z = height;
        marker.color = color;
        const Eigen::Quaterniond qt(pose.linear());
        marker.pose.orientation.w = qt.w();
        marker.pose.orientation.x = qt.x();
        marker.pose.orientation.y = qt.y();
        marker.pose.orientation.z = qt.z();
        marker.pose.position.x = pose.translation().x();
        marker.pose.position.y = pose.translation().y();
        marker.pose.position.z = pose.translation().z();
        return marker;
      };

      std_msgs::msg::ColorRGBA red;
      red.r = 1.0;
      red.a = 1.0;

      std_msgs::msg::ColorRGBA green;
      green.g = 1.0;
      green.a = 1.0;

      std_msgs::msg::ColorRGBA blue;
      blue.b = 1.0;
      blue.a = 1.0;

      const double length = 0.1;
      const double radius = 0.02;

      for (const auto &node : sim_band.nodes) {
        const Eigen::Isometry3d pose =
            Eigen::Translation3d(node.pose.translation().x(),
                                 node.pose.translation().y(), 0.0) *
            Eigen::AngleAxisd(Eigen::Rotation2Dd(node.pose.rotation()).angle(),
                              Eigen::Vector3d::UnitZ());

        // X Axis
        const Eigen::Isometry3d x_pose =
            pose * (Eigen::Translation3d(length / 2.0, 0, 0) *
                    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()));
        ma.markers.push_back(make_cylider(red, radius, length, x_pose));

        // Y Axis
        const Eigen::Isometry3d y_pose =
            pose * (Eigen::Translation3d(0, length / 2.0, 0) *
                    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()));
        ma.markers.push_back(make_cylider(green, radius, length, y_pose));

        // Z Axis
        const Eigen::Isometry3d z_pose =
            pose * (Eigen::Translation3d(0, 0, length / 2.0) *
                    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));
        ma.markers.push_back(make_cylider(blue, radius, length, z_pose));

        {
          const sim_band_planner::ControlPoint &cp =
              node.control_points[node.closest_point];
          const Eigen::Vector2d cp_pose =
              node.pose.translation() + node.pose.linear() * cp.offset;
          const Eigen::Vector2d obj_pose =
              cp_pose + (cp.distance + inflated_robot_radius) *
                            cp.gradient.cast<double>();

          geometry_msgs::msg::Point start;
          start.x = node.pose.translation().x();
          start.y = node.pose.translation().y();

          geometry_msgs::msg::Point end;
          end.x = obj_pose.x();
          end.y = obj_pose.y();

          visualization_msgs::msg::Marker marker;
          marker.ns = "arrow";
          marker.id = static_cast<int>(ma.markers.size());
          marker.type = visualization_msgs::msg::Marker::ARROW;
          marker.action = visualization_msgs::msg::Marker::ADD;
          marker.header.stamp = now;
          marker.header.frame_id = "map";
          marker.frame_locked = true;
          marker.scale.x = 0.006;
          marker.scale.y = 0.006;
          marker.scale.z = 0.02;
          marker.color.r = 1.0;
          marker.color.a = 1.0;
          marker.pose.orientation.w = 1;
          marker.points.push_back(start);
          marker.points.push_back(end);
          ma.markers.push_back(marker);
        }
      }

      for (size_t c = 0; c < sim_band.nodes.front().control_points.size();
           c++) {
        const ControlPoint &cp = sim_band.nodes.front().control_points[c];
        const Eigen::Vector2d position =
            sim_band.nodes.front().pose.translation() +
            sim_band.nodes.front().pose.linear() * cp.offset;
        const Eigen::Isometry3d pose =
            Eigen::Translation3d(position.x(), position.y(), 0.0) *
            Eigen::Quaterniond::Identity();

        const std_msgs::msg::ColorRGBA color = distanceToColor(cp.distance);

        ma.markers.push_back(
            make_cylider(color, inflated_robot_radius * 2.0, 0.1, pose));

        {
          const Eigen::Vector2d obj_pose =
              position + (cp.distance + inflated_robot_radius) *
                             cp.gradient.cast<double>();

          geometry_msgs::msg::Point start;
          start.x = position.x();
          start.y = position.y();

          geometry_msgs::msg::Point end;
          end.x = obj_pose.x();
          end.y = obj_pose.y();

          visualization_msgs::msg::Marker marker;
          marker.ns = "arrow";
          marker.id = static_cast<int>(ma.markers.size());
          marker.type = visualization_msgs::msg::Marker::ARROW;
          marker.action = visualization_msgs::msg::Marker::ADD;
          marker.header.stamp = now;
          marker.header.frame_id = "map";
          marker.frame_locked = true;
          marker.scale.x = 0.006;
          marker.scale.y = 0.006;
          marker.scale.z = 0.02;
          marker.color = color;
          marker.pose.orientation.w = 1;
          marker.points.push_back(start);
          marker.points.push_back(end);
          ma.markers.push_back(marker);
        }
      }
      marker_pub_->publish(ma);
    }

    // copy the nodes back to the moving window
    rcpputils::assert_true(!sim_band.nodes.empty());
    moving_window_->window.nodes.clear();
    // don't copy the first node (its just the current robot pose)
    moving_window_->window.nodes.insert(moving_window_->window.nodes.end(),
                                        sim_band.nodes.begin() + 1,
                                        sim_band.nodes.end());

    // trim to valid
    result.outcome =
        navigation_interface::TrajectoryPlanner::Outcome::SUCCESSFUL;
    result.path_start_i = 0;
    result.path_end_i = moving_window_->end_i;
    std::size_t end_i;
    for (end_i = 0; end_i < sim_band.nodes.size(); ++end_i) {
      if (sim_band.nodes[end_i]
              .control_points[sim_band.nodes[end_i].closest_point]
              .distance < 0) {
        auto clock = node_->get_clock();
        RCLCPP_WARN_STREAM_THROTTLE(
            rclcpp::get_logger(""), *clock, 1000,
            "Point: " << end_i << " of trajectory is in collision");
        //                end_i = end_i > 1 ? end_i - 1 : 0;
        result.outcome =
            navigation_interface::TrajectoryPlanner::Outcome::PARTIAL;
        result.path_end_i =
            moving_window_->end_i - (sim_band.nodes.size() - 1 - end_i);
        break;
      }
    }

    if (end_i == 0)
      return result;

    // TODO collision check interpolation

    // Transform to odom frame
    result.trajectory.header.frame_id = "odom";
    result.cost = 0;
    const Eigen::Isometry2d odom_to_map = map_to_odom.inverse();

    for (std::size_t i = 0; i < end_i; ++i) {
      result.trajectory.states.push_back(
          {odom_to_map * sim_band.nodes[i].pose,
           {0, 0, 0},
           sim_band.nodes[i]
               .control_points[sim_band.nodes[i].closest_point]
               .distance});
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(""),
                        "Optimization failed: " << e.what());
    result.outcome = navigation_interface::TrajectoryPlanner::Outcome::FAILED;
    return result;
  }

  return result;
}

// cppcheck-suppress unusedFunction
bool SimBandPlanner::valid(const navigation_interface::Trajectory &,
                           const double) const {
  return true;
}

double SimBandPlanner::cost(const navigation_interface::Trajectory &,
                            const double) const {
  return 0.0;
}

// cppcheck-suppress unusedFunction
void SimBandPlanner::onInitialize(const YAML::Node &parameters) {
  debug_viz_ = parameters["debug_viz"].as<bool>(debug_viz_);

  num_iterations_ = parameters["num_iterations"].as<int>(num_iterations_);
  collision_distance_ =
      parameters["collision_distance"].as<double>(collision_distance_);

  nominal_force_gain_ =
      parameters["nominal_force_gain"].as<double>(nominal_force_gain_);
  internal_force_gain_ =
      parameters["internal_force_gain"].as<double>(internal_force_gain_);
  external_force_gain_ =
      parameters["external_force_gain"].as<double>(external_force_gain_);
  rotation_gain_ = parameters["rotation_gain"].as<double>(rotation_gain_);

  max_window_length_ =
      parameters["max_window_length"].as<double>(max_window_length_);

  velocity_decay_ = parameters["velocity_decay"].as<double>(velocity_decay_);
  alpha_decay_ = parameters["alpha_decay"].as<double>(alpha_decay_);

  max_acc_[0] = parameters["max_x_acc"].as<double>(max_acc_[0]);
  max_acc_[1] = parameters["max_y_acc"].as<double>(max_acc_[1]);
  max_acc_[2] = parameters["max_w_acc"].as<double>(max_acc_[2]);

  robot_radius_ = parameters["robot_radius"].as<double>(robot_radius_);

  offsets_ =
      navigation_interface::get_point_list(parameters, "robot_radius_offsets",
                                           {{-0.268, 0.000},
                                            {0.268, 0.000},
                                            {0.265, -0.185},
                                            {0.077, -0.185},
                                            {-0.077, -0.185},
                                            {-0.265, -0.185},
                                            {0.265, 0.185},
                                            {-0.265, 0.185},
                                            {-0.077, 0.185},
                                            {0.077, 0.185}});

  if (debug_viz_) {
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/sim_band/band", 100);
  }
}

// cppcheck-suppress unusedFunction
void SimBandPlanner::onMapDataChanged() {}
} // namespace sim_band_planner
