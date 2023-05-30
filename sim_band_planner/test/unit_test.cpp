#include <gridmap/map_data.h>
#include <gtest/gtest.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/console.h>
#include <sim_band_planner/distance_field.h>
#include <sim_band_planner/simulate.h>

#include <chrono>

TEST(test_plugin, test_plugin) {
  const double resolution = 0.02;
  const int size_x = 20 / resolution;
  const int size_y = 20 / resolution;

  map_manager::MapInfo map_info;
  map_info.meta_data.width = size_x;
  map_info.meta_data.height = size_y;
  const gridmap::MapDimensions map_dims(
      resolution, {-(size_x / 2) * resolution, -(size_y / 2) * resolution},
      {size_x, size_y});

  const std::vector<graph_map::Zone> zones;
  std::shared_ptr<gridmap::MapData> map_data =
      std::make_shared<gridmap::MapData>(map_info, map_dims, zones);

  cv::Mat cv_im = cv::Mat(map_data->grid.dimensions().size().y(),
                          map_data->grid.dimensions().size().x(), CV_8U,
                          reinterpret_cast<void *>(const_cast<uint8_t *>(
                              map_data->grid.cells().data())));

  cv::rectangle(cv_im, cv::Point(0.2 * size_x, 0.2 * size_y),
                cv::Point(0.5 * size_x, 0.6 * size_y),
                cv::Scalar(gridmap::OccupancyGrid::OCCUPIED), -1);
  cv::rectangle(cv_im, cv::Point(0.59 * size_x, 0.3 * size_y),
                cv::Point(0.9 * size_x, 0.6 * size_y),
                cv::Scalar(gridmap::OccupancyGrid::OCCUPIED), -1);
  cv::circle(cv_im, cv::Point(3 * size_x / 4.0, size_y / 2), 185,
             cv::Scalar(gridmap::OccupancyGrid::OCCUPIED), -1, cv::LINE_8);

  cv::circle(cv_im, cv::Point(0.4 * size_x, 0.78 * size_y), 1,
             cv::Scalar(gridmap::OccupancyGrid::OCCUPIED), -1, cv::LINE_8);

  const Eigen::Isometry2d start =
      Eigen::Translation2d(-(size_x / 4) * resolution + 0.5,
                           (size_y / 3) * resolution) *
      Eigen::Rotation2Dd(0);
  const Eigen::Isometry2d goal =
      Eigen::Translation2d((size_x / 4) * resolution,
                           -(size_y / 3) * resolution) *
      Eigen::Rotation2Dd(0);

  ROS_INFO_STREAM("planning...");

  const std::vector<Eigen::Vector2d> offsets = {
      {-0.268, 0.000},  {0.268, 0.000},   {0.265, -0.185}, {0.077, -0.185},
      {-0.077, -0.185}, {-0.265, -0.185}, {0.265, 0.185},  {-0.265, 0.185},
      {-0.077, 0.185},  {0.077, 0.185}};

  std::vector<Eigen::Isometry2d> poses;
  poses.push_back(start);
  poses.push_back(Eigen::Translation2d(-0.22 * size_x * resolution,
                                       +0.33 * size_y * resolution) *
                  Eigen::Rotation2Dd(0));
  poses.push_back(Eigen::Translation2d(-0.20 * size_x * resolution,
                                       +0.25 * size_y * resolution) *
                  Eigen::Rotation2Dd(-0.1));
  poses.push_back(Eigen::Translation2d(-0.15 * size_x * resolution,
                                       +0.25 * size_y * resolution) *
                  Eigen::Rotation2Dd(-0.2));
  poses.push_back(Eigen::Translation2d(+0.03 * size_x * resolution,
                                       +0.25 * size_y * resolution) *
                  Eigen::Rotation2Dd(-1.5));
  poses.push_back(Eigen::Translation2d(+0.03 * size_x * resolution,
                                       +0.10 * size_y * resolution) *
                  Eigen::Rotation2Dd(-1.5));
  poses.push_back(Eigen::Translation2d(+0.03 * size_x * resolution,
                                       +0.00 * size_y * resolution) *
                  Eigen::Rotation2Dd(-1.5));
  poses.push_back(Eigen::Translation2d(+0.04 * size_x * resolution,
                                       -0.25 * size_x * resolution) *
                  Eigen::Rotation2Dd(-1.5));
  poses.push_back(goal);

  const double linear_step = 0.8;
  const double anglar_step = 0.1;

  sim_band_planner::Band band(offsets);

  for (size_t p = 0; p < poses.size() - 1; ++p) {
    const Eigen::Isometry2d start_pose = poses[p];
    const Eigen::Isometry2d end_pose = poses[p + 1];

    const Eigen::Rotation2Dd start_qt(start_pose.linear());
    const Eigen::Rotation2Dd end_qt(end_pose.linear());

    const Eigen::Vector2d direction =
        end_pose.translation() - start_pose.translation();

    const double linear_dist = direction.norm() > 0 ? direction.norm() : 0.0;
    const double angular_dist = std::abs(
        Eigen::Rotation2Dd(end_pose.linear().inverse() * start_pose.linear())
            .smallestAngle());
    const double steps =
        std::max(linear_dist / linear_step, angular_dist / anglar_step);
    for (unsigned int s = 0; s < static_cast<unsigned int>(steps);
         ++s) // Final partial step truncated
    {
      const double fraction = s > 0 ? static_cast<double>(s) / steps : 0.0;
      const Eigen::Isometry2d interpolated_pose =
          Eigen::Translation2d(start_pose.translation() +
                               fraction * direction) *
          start_qt.slerp(fraction, end_qt);
      band.nodes.push_back(
          sim_band_planner::Node(interpolated_pose, band.radius_offsets));
    }
  }

  int num_iterations_ = 100;

  double collision_distance_ = 0.5;
  double nominal_force_gain_ = 0.02;
  double internal_force_gain_ = 0.002;
  double external_force_gain_ = 0.001;
  double rotation_gain_ = 0.005;

  double velocity_decay_ = 0.6;
  double alpha_decay_ = 1.0 - std::pow(0.001, 1.0 / 100.0);

  double robot_radius_ = 0.240;

  sim_band_planner::DistanceField distance_field(
      cv_im, map_data->grid.dimensions().origin().x(),
      map_data->grid.dimensions().origin().y(), resolution, robot_radius_);

  for (int i = 0; i < 1; ++i) {
    auto t0 = std::chrono::steady_clock::now();

    sim_band_planner::simulate(
        band, distance_field, num_iterations_, collision_distance_,
        nominal_force_gain_, internal_force_gain_, external_force_gain_,
        rotation_gain_, velocity_decay_, 1.0, alpha_decay_);

    std::cout << "simulate took: "
              << std::chrono::duration_cast<std::chrono::duration<double>>(
                     std::chrono::steady_clock::now() - t0)
                     .count()
              << std::endl;

    cv::Mat disp;
    cv::cvtColor(cv_im, disp, cv::COLOR_GRAY2BGR);

    for (const auto &n : poses) {
      const Eigen::Array2i mp =
          map_data->grid.dimensions().getCellIndex(n.translation());
      const Eigen::Rotation2Dd rot(n.linear());

      cv::circle(disp, cv::Point(mp.x(), mp.y()), (0.5 + 0.2) / resolution,
                 cv::Scalar(0, 0, 255), 1);

      const Eigen::Vector2d x_end =
          Eigen::Vector2d(mp.x(), mp.y()) + rot * Eigen::Vector2d(4, 0);
      const Eigen::Vector2d y_end =
          Eigen::Vector2d(mp.x(), mp.y()) + rot * Eigen::Vector2d(0, 4);

      cv::line(disp, cv::Point(mp.x(), mp.y()), cv::Point(x_end.x(), x_end.y()),
               cv::Scalar(255, 0, 0), 1);
      cv::line(disp, cv::Point(mp.x(), mp.y()), cv::Point(y_end.x(), y_end.y()),
               cv::Scalar(255, 0, 0), 1);
    }

    for (const auto &n : band.nodes) {
      const Eigen::Array2i mp =
          map_data->grid.dimensions().getCellIndex(n.pose.translation());
      const Eigen::Rotation2Dd rot(n.pose.linear());

      for (std::size_t c = 0; c < n.control_points.size(); ++c) {
        const sim_band_planner::ControlPoint &cp = n.control_points[c];
        const Eigen::Vector2d cp_pose =
            n.pose.translation() + n.pose.linear() * cp.offset;
        const Eigen::Array2i cp_mp =
            map_data->grid.dimensions().getCellIndex(cp_pose);
        cv::circle(disp, cv::Point(cp_mp.x(), cp_mp.y()), 1,
                   cv::Scalar(0, 255, 0), 1);

        const Eigen::Vector2d g_end =
            Eigen::Vector2d(cp_mp.x(), cp_mp.y()) +
            (cp.distance / resolution) * cp.gradient.cast<double>();
        cv::line(disp, cv::Point(cp_mp.x(), cp_mp.y()),
                 cv::Point(g_end.x(), g_end.y()), cv::Scalar(255, 0, 255), 1);
      }

      const Eigen::Vector2d x_end =
          Eigen::Vector2d(mp.x(), mp.y()) + rot * Eigen::Vector2d(6, 0);
      const Eigen::Vector2d y_end =
          Eigen::Vector2d(mp.x(), mp.y()) + rot * Eigen::Vector2d(0, 6);

      cv::line(disp, cv::Point(mp.x(), mp.y()), cv::Point(x_end.x(), x_end.y()),
               cv::Scalar(0, 0, 255), 1);
      cv::line(disp, cv::Point(mp.x(), mp.y()), cv::Point(y_end.x(), y_end.y()),
               cv::Scalar(0, 255, 0), 1);
    }

    //        cv::namedWindow("disp", cv::WINDOW_NORMAL);
    //        cv::imshow("disp", disp);
    //        cv::waitKey(0);
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
