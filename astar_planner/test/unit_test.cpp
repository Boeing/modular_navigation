#include <gtest/gtest.h>

#include <astar_planner/plugin.h>

#include <chrono>

#include <costmap_2d/costmap_2d.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

TEST(test_plugin, test_plugin)
{
    const double robot_radius = 0.5;
    const double resolution = 0.02;
    const int size_x = 20 / resolution;
    const int size_y = 20 / resolution;

    std::shared_ptr<costmap_2d::Costmap2D> costmap = std::make_shared<costmap_2d::Costmap2D>(size_x, size_y, resolution, -(size_x / 2) * resolution, -(size_y / 2) * resolution);

    cv::Mat cv_im = cv::Mat(size_x, size_y, CV_8UC1, reinterpret_cast<void*>(costmap->getCharMap()));

    cv::circle(cv_im, cv::Point(size_x / 4.0, size_y / 2), 116, cv::Scalar(255), -1, cv::LINE_8);
    cv::circle(cv_im, cv::Point(3 * size_x / 4.0, size_y / 2), 80, cv::Scalar(255), -1, cv::LINE_8);
    cv::circle(cv_im, cv::Point(size_x / 2.0, 3 * size_y / 4.0), 6, cv::Scalar(255), -1, cv::LINE_8);
    cv::circle(cv_im, cv::Point(size_x / 2.0, size_y / 5.0), 1, cv::Scalar(255), -1, cv::LINE_8);

    auto t0 = std::chrono::steady_clock::now();

    cv::Mat inv_cv_im;
    cv::bitwise_not(cv_im, inv_cv_im);

    cv::Mat dist;
    cv::distanceTransform(inv_cv_im, dist, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32S);

    dist.setTo(0, dist < robot_radius / resolution);

    dist = - (dist - robot_radius) * resolution;

    cv::exp(dist, dist);

    dist.convertTo(cv_im, CV_8U, 255.0, 0);

    std::cout << "distance transform took: " << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count() << std::endl;

    astar_planner::AStarPlanner plugin;

    XmlRpc::XmlRpcValue params;
    params["neutral_cost"] = XmlRpc::XmlRpcValue(40.0);

    plugin.initialize(params, costmap);

    const Eigen::Isometry2d start = Eigen::Translation2d(0, -(size_y / 3) * resolution) * Eigen::Rotation2Dd(0);
    const Eigen::Isometry2d goal = Eigen::Translation2d(-(size_x / 4) * resolution, (size_y / 3) * resolution) * Eigen::Rotation2Dd(1.0);

    t0 = std::chrono::steady_clock::now();

    const auto result = plugin.plan(start, goal);

    std::cout << "planner took: " << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count() << std::endl;

    std::cout << "path size: " << result.path.nodes.size() << std::endl;
    std::cout << "path cost: " << result.cost << std::endl;

    const double cost_check = plugin.cost(result.path);

    std::cout << "path cost check: " << cost_check << std::endl;

    cv::Mat disp;
    cv::cvtColor(cv_im, disp, cv::COLOR_GRAY2BGR);
    cv::circle(disp, cv::Point(size_x / 4.0, size_y / 2), 116, cv::Scalar(0, 255, 0), -1, cv::LINE_8);
    cv::circle(disp, cv::Point(3 * size_x / 4.0, size_y / 2), 80, cv::Scalar(0, 255, 0), -1, cv::LINE_8);
    cv::circle(disp, cv::Point(size_x / 2.0, 3 * size_y / 4.0), 6, cv::Scalar(0, 255, 0), -1, cv::LINE_8);
    cv::circle(disp, cv::Point(size_x / 2.0, size_y / 5.0), 1, cv::Scalar(0, 255, 0), -1, cv::LINE_8);

    for (const auto& n : result.path.nodes)
    {
        unsigned int mx;
        unsigned int my;
        costmap->worldToMap(n.translation().x(), n.translation().y(), mx, my);
        cv::circle(disp, cv::Point(mx, my), 1, cv::Scalar(255, 0, 0), -1, cv::LINE_8);
    }

    cv::imwrite("test.png", disp);
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
