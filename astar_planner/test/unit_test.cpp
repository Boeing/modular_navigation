#include <gtest/gtest.h>

#include <astar_planner/plugin.h>

#include <chrono>

#include <gridmap/map_data.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

TEST(test_plugin, test_plugin)
{
    const double resolution = 0.02;
    const int size_x = 20 / resolution;
    const int size_y = 20 / resolution;

    std::shared_ptr<gridmap::MapData> map_data = std::make_shared<costmap_2d::MapData>(0.1, 0.9, 0.8);

    map_data->resize(size_x, size_y, resolution, -(size_x / 2) * resolution, -(size_y / 2) * resolution);

    std::cout << "map size: " << map_data->sizeX() << " " << map_data->sizeY() << std::endl;

    cv::Mat cv_im = cv::Mat(map_data->sizeY(), map_data->sizeX(), CV_64F,
                            reinterpret_cast<void*>(const_cast<double*>(map_data->data())));

    cv::circle(cv_im, cv::Point(size_x / 4.0, size_y / 2), 116, cv::Scalar(map_data->clampingThresMaxLog()), -1, cv::LINE_8);
    cv::circle(cv_im, cv::Point(3 * size_x / 4.0, size_y / 2), 80, cv::Scalar(map_data->clampingThresMaxLog()), -1, cv::LINE_8);
    cv::circle(cv_im, cv::Point(size_x / 2.0, 3 * size_y / 4.0), 6, cv::Scalar(map_data->clampingThresMaxLog()), -1, cv::LINE_8);
    cv::circle(cv_im, cv::Point(size_x / 2.0, size_y / 5.0), 1, cv::Scalar(map_data->clampingThresMaxLog()), -1, cv::LINE_8);

    auto t0 = std::chrono::steady_clock::now();

    astar_planner::AStarPlanner plugin;

    XmlRpc::XmlRpcValue params;
    params["neutral_cost"] = XmlRpc::XmlRpcValue(0.1);
    params["robot_radius"] = XmlRpc::XmlRpcValue(0.5);

    plugin.initialize(params, map_data);

    const Eigen::Isometry2d start = Eigen::Translation2d(0, -(size_y / 3) * resolution) * Eigen::Rotation2Dd(0);
    const Eigen::Isometry2d goal = Eigen::Translation2d(-(size_x / 4) * resolution, (size_y / 3) * resolution) * Eigen::Rotation2Dd(1.0);

    t0 = std::chrono::steady_clock::now();

    const auto result = plugin.plan(start, goal);

    std::cout << "planner took: " << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count() << std::endl;

    std::cout << "path size: " << result.path.nodes.size() << std::endl;
    std::cout << "path cost: " << result.cost << std::endl;

    const double cost_check = plugin.cost(result.path);

    std::cout << "path cost check: " << cost_check << std::endl;

    cv::Mat disp(map_data->sizeY(), map_data->sizeX(), CV_8SC3, cv::Scalar(0, 0, 0));
    cv::circle(disp, cv::Point(size_x / 4.0, size_y / 2), 116, cv::Scalar(0, 255, 0), -1, cv::LINE_8);
    cv::circle(disp, cv::Point(3 * size_x / 4.0, size_y / 2), 80, cv::Scalar(0, 255, 0), -1, cv::LINE_8);
    cv::circle(disp, cv::Point(size_x / 2.0, 3 * size_y / 4.0), 6, cv::Scalar(0, 255, 0), -1, cv::LINE_8);
    cv::circle(disp, cv::Point(size_x / 2.0, size_y / 5.0), 1, cv::Scalar(0, 255, 0), -1, cv::LINE_8);

    for (const auto& n : result.path.nodes)
    {
        unsigned int mx;
        unsigned int my;
        map_data->worldToMap(n.translation().x(), n.translation().y(), mx, my);
        cv::circle(disp, cv::Point(mx, my), 1, cv::Scalar(255, 0, 0), -1, cv::LINE_8);
    }

    cv::imwrite("test.png", disp);
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
