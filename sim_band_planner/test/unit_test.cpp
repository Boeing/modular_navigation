#include <gtest/gtest.h>

#include <sim_band_planner/distance_field.h>
#include <sim_band_planner/simulate.h>

#include <chrono>

#include <costmap_2d/costmap_2d.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <ros/console.h>

TEST(test_plugin, test_plugin)
{
    const double resolution = 0.02;
    const int size_x = 400;
    const int size_y = 400;

    std::shared_ptr<costmap_2d::Costmap2D> costmap = std::make_shared<costmap_2d::Costmap2D>(size_x, size_y, resolution, -(size_x / 2) * resolution, -(size_y / 2) * resolution);

    cv::Mat cv_im = cv::Mat(size_x, size_y, CV_8UC1, reinterpret_cast<void*>(costmap->getCharMap()));

    cv::circle(cv_im, cv::Point(size_x / 4.0, size_y / 2), 116, cv::Scalar(255), -1, cv::LINE_8);
    cv::circle(cv_im, cv::Point(3 * size_x / 4.0, size_y / 2), 80, cv::Scalar(255), -1, cv::LINE_8);
    cv::circle(cv_im, cv::Point(size_x / 2.0, 3 * size_y / 4.0), 6, cv::Scalar(255), -1, cv::LINE_8);
    cv::circle(cv_im, cv::Point(size_x / 2.0, size_y / 5.0), 1, cv::Scalar(255), -1, cv::LINE_8);

    const Eigen::Isometry2d start = Eigen::Translation2d(0, -(size_y / 3) * resolution) * Eigen::Rotation2Dd(0);
    const Eigen::Isometry2d goal = Eigen::Translation2d(-(size_x / 4) * resolution, (size_y / 3) * resolution) * Eigen::Rotation2Dd(1.0);

    ROS_INFO_STREAM("planning...");

    sim_band_planner::Band band;
    band.nodes.push_back(sim_band_planner::Node(start));
    band.nodes.push_back(sim_band_planner::Node(goal));

    int num_iterations_ = 1;
    double internal_force_gain_ = 0.004;
    double external_force_gain_ = 0.002;
    double min_distance_ = 0.02;
    double min_overlap_ = 0.2;
    double robot_radius_ = 0.1;
    double rotation_factor_ = 1.0;
    double velocity_decay_ = 0.6;
    double alpha_decay_ = 1.0 / std::pow(0.001, 1.0 / 200.0);

    sim_band_planner::DistanceField distance_field(cv_im, costmap->getOriginX(), costmap->getOriginY(), costmap->getResolution(), robot_radius_);

    for (int i=0; i<100000; ++i)
    {

        sim_band_planner::simulate(band,
                 distance_field,
                 num_iterations_,
                 min_overlap_,
                 min_distance_,
                 internal_force_gain_,
                 external_force_gain_,
                 rotation_factor_,
                 velocity_decay_,
                 1.0,
                 alpha_decay_);


        cv::Mat disp;
        cv::cvtColor(cv_im, disp, cv::COLOR_GRAY2BGR);

        for (const auto& n : band.nodes)
        {
            unsigned int mx;
            unsigned int my;
            costmap->worldToMap(n.pose.translation().x(), n.pose.translation().y(), mx, my);
            cv::circle(disp, cv::Point(mx, my), std::abs(n.distance) / costmap->getResolution(), cv::Scalar(0, 255, 0), 1);
        }

        cv::circle(disp, cv::Point(size_x / 4.0, size_y / 2), 116, cv::Scalar(0, 255, 255), -1, cv::LINE_8);
        cv::circle(disp, cv::Point(3 * size_x / 4.0, size_y / 2), 80, cv::Scalar(0, 255, 255), -1, cv::LINE_8);
        cv::circle(disp, cv::Point(size_x / 2.0, 3 * size_y / 4.0), 6, cv::Scalar(0, 255, 255), -1, cv::LINE_8);
        cv::circle(disp, cv::Point(size_x / 2.0, size_y / 5.0), 1, cv::Scalar(0, 255, 255), -1, cv::LINE_8);

        cv::namedWindow("disp", cv::WINDOW_NORMAL);
        cv::imshow("disp", disp);
        cv::waitKey(500);
    }

//    cv::imwrite("test.png", disp);
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
