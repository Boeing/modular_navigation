#include <gtest/gtest.h>

#include <sim_band_planner/distance_field.h>
#include <sim_band_planner/simulate.h>

#include <chrono>

#include <gridmap/map_data.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <ros/console.h>

TEST(test_plugin, test_plugin)
{
    const double resolution = 0.02;
    const int size_x = 20 / resolution;
    const int size_y = 20 / resolution;

    hd_map::Map hd_map;
    hd_map.info.meta_data.width = size_x;
    hd_map.info.meta_data.height = size_y;
    const gridmap::MapDimensions map_dims(resolution, {-(size_x / 2) * resolution, -(size_y / 2) * resolution},
                                          {size_x, size_y});

    std::shared_ptr<gridmap::MapData> map_data = std::make_shared<gridmap::MapData>(hd_map, map_dims);

    cv::Mat cv_im = cv::Mat(map_data->grid.dimensions().size().y(), map_data->grid.dimensions().size().x(), CV_8U,
                            reinterpret_cast<void*>(const_cast<uint8_t*>(map_data->grid.cells().data())));

    cv::rectangle(cv_im, cv::Point(0.2 * size_x, 0.2 * size_y), cv::Point(0.5 * size_x, 0.6 * size_y),
                  cv::Scalar(gridmap::OccupancyGrid::OCCUPIED), -1);
    cv::rectangle(cv_im, cv::Point(0.59 * size_x, 0.3 * size_y), cv::Point(0.9 * size_x, 0.6 * size_y),
                  cv::Scalar(gridmap::OccupancyGrid::OCCUPIED), -1);
    cv::circle(cv_im, cv::Point(3 * size_x / 4.0, size_y / 2), 185, cv::Scalar(gridmap::OccupancyGrid::OCCUPIED), -1,
               cv::LINE_8);
    cv::circle(cv_im, cv::Point(size_x / 2.0, 3 * size_y / 4.0), 6, cv::Scalar(gridmap::OccupancyGrid::OCCUPIED), -1,
               cv::LINE_8);
    cv::circle(cv_im, cv::Point(size_x / 2.0, size_y / 5.0), 1, cv::Scalar(gridmap::OccupancyGrid::OCCUPIED), -1,
               cv::LINE_8);

    const Eigen::Isometry2d start =
        Eigen::Translation2d(-(size_x / 4) * resolution + 0.5, (size_y / 3) * resolution) * Eigen::Rotation2Dd(0);
    const Eigen::Isometry2d goal =
        Eigen::Translation2d((size_x / 4) * resolution, -(size_y / 3) * resolution) * Eigen::Rotation2Dd(-M_PI / 2);

    ROS_INFO_STREAM("planning...");

    const std::vector<Eigen::Vector2d> offsets = {{-0.268, 0.000},  {0.268, 0.000},   {0.265, -0.185}, {0.077, -0.185},
                                                  {-0.077, -0.185}, {-0.265, -0.185}, {0.265, 0.185},  {-0.265, 0.185},
                                                  {-0.077, 0.185},  {0.077, 0.185}};

    sim_band_planner::Band band(offsets);
    band.nodes.push_back(sim_band_planner::Node(start, band.radius_offsets));
    band.nodes.push_back(sim_band_planner::Node(goal, band.radius_offsets));

    int num_iterations_ = 10;
    double internal_force_gain_ = 0.004;
    double external_force_gain_ = 0.0002;
    double min_distance_ = 0.2;
    double max_distance_ = 1.0;
    double min_overlap_ = 0.05;
    double robot_radius_ = 0.210;
    double rotation_factor_ = 2.0;
    double velocity_decay_ = 0.6;
    double alpha_decay_ = 1.0 - std::pow(0.001, 1.0 / 20.0);

    sim_band_planner::DistanceField distance_field(cv_im, map_data->grid.dimensions().origin().x(),
                                                   map_data->grid.dimensions().origin().y(), resolution, robot_radius_);

    for (int i = 0; i < 100000; ++i)
    {
        auto t0 = std::chrono::steady_clock::now();

        sim_band_planner::simulate(band, distance_field, num_iterations_, min_overlap_, min_distance_,
                                   internal_force_gain_, external_force_gain_, rotation_factor_, velocity_decay_, 1.0,
                                   alpha_decay_, max_distance_, 20);

        std::cout
            << "simulate took: "
            << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count()
            << std::endl;

        cv::Mat disp(map_data->grid.dimensions().size().y(), map_data->grid.dimensions().size().x(), CV_8SC3,
                     cv::Scalar(0, 0, 0));

        for (const auto& n : band.nodes)
        {
            const auto mp = map_data->grid.dimensions().getCellIndex(n.pose.translation());

            cv::circle(disp, cv::Point(mp.x(), mp.y()), (0.5 + 0.2) / resolution, cv::Scalar(255, 0, 0), 1);

            for (std::size_t c = 0; c < n.control_points.size(); ++c)
            {
                const sim_band_planner::ControlPoint& cp = n.control_points[c];
                const auto cp_pose = n.pose.translation() + n.pose.rotation() * cp.offset;
                const auto cp_mp = map_data->grid.dimensions().getCellIndex(cp_pose);
                cv::circle(disp, cv::Point(cp_mp.x(), cp_mp.y()), robot_radius_ / resolution, cv::Scalar(0, 255, 0), 1);
            }

            const auto x_end = Eigen::Vector2d(mp.x(), mp.y()) +
                               Eigen::Vector2d(Eigen::Rotation2Dd(n.pose.rotation()) * Eigen::Vector2d(6, 0));
            const auto y_end = Eigen::Vector2d(mp.x(), mp.y()) +
                               Eigen::Vector2d(Eigen::Rotation2Dd(n.pose.rotation()) * Eigen::Vector2d(0, 6));

            cv::line(disp, cv::Point(mp.x(), mp.y()), cv::Point(x_end.x(), x_end.y()), cv::Scalar(0, 0, 255), 1);
            cv::line(disp, cv::Point(mp.x(), mp.y()), cv::Point(y_end.x(), y_end.y()), cv::Scalar(0, 255, 0), 1);
        }

        cv::rectangle(disp, cv::Point(0.2 * size_x, 0.2 * size_y), cv::Point(0.5 * size_x, 0.6 * size_y),
                      cv::Scalar(0, 255, 255), -1);
        cv::rectangle(disp, cv::Point(0.59 * size_x, 0.3 * size_y), cv::Point(0.9 * size_x, 0.6 * size_y),
                      cv::Scalar(0, 255, 255), -1);
        cv::circle(disp, cv::Point(3 * size_x / 4.0, size_y / 2), 185, cv::Scalar(0, 255, 255), -1, cv::LINE_8);
        cv::circle(disp, cv::Point(size_x / 2.0, 3 * size_y / 4.0), 6, cv::Scalar(0, 255, 255), -1, cv::LINE_8);
        cv::circle(disp, cv::Point(size_x / 2.0, size_y / 5.0), 1, cv::Scalar(0, 255, 255), -1, cv::LINE_8);

        cv::namedWindow("disp", cv::WINDOW_NORMAL);
        cv::imshow("disp", disp);
        cv::waitKey(1);
    }
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
