#include <gtest/gtest.h>

#include <astar_planner/astar.h>
#include <astar_planner/plugin.h>

#include <chrono>

#include <gridmap/map_data.h>

#include <hd_map/Map.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

TEST(test_astar, test_astar)
{
    const double neutral_cost = 0.2;
    const double exponential_weight = 2.0;
    const double resolution = 0.02;
    const double robot_radius = 0.5;
    const int size_x = 20 / resolution;
    const int size_y = 20 / resolution;

    cv::Mat obstacle_map = cv::Mat(size_x, size_y, CV_8U, cv::Scalar(0));
    cv::circle(obstacle_map, cv::Point(size_x / 4.0, size_y / 2), 116, cv::Scalar(255), -1, cv::LINE_8);
    cv::circle(obstacle_map, cv::Point(3 * size_x / 4.0, size_y / 2), 80, cv::Scalar(255), -1, cv::LINE_8);
    cv::circle(obstacle_map, cv::Point(size_x / 2.0, 3 * size_y / 4.0), 6, cv::Scalar(255), -1, cv::LINE_8);
    cv::circle(obstacle_map, cv::Point(size_x / 2.0, size_y / 5.0), 1, cv::Scalar(255), -1, cv::LINE_8);

    auto t0 = std::chrono::steady_clock::now();

    // dilate robot radius
    cv::Mat dilated;
    const int cell_inflation_radius = static_cast<int>(2.0 * robot_radius / resolution);
    auto ellipse = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(cell_inflation_radius, cell_inflation_radius));
    cv::dilate(obstacle_map, dilated, ellipse);

    // flip
    cv::bitwise_not(dilated, dilated);

    // allocate
    cv::Mat costmap = cv::Mat(dilated.size(), CV_32F);

    // find obstacle distances
    cv::distanceTransform(dilated, costmap, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);

    // inflate
    costmap = -exponential_weight * costmap * resolution;

    // negative exponent maps values to [1,0)
    cv::exp(costmap, costmap);

    std::cout
        << "preparing costmap took: "
        << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count()
        << std::endl;

    astar_planner::PathFinder astar(costmap.size().width, costmap.size().height, reinterpret_cast<float*>(costmap.data), neutral_cost);

    const astar_planner::Coord2D start_coord((size_x / 2), (size_y / 6));
    const astar_planner::Coord2D goal_coord((1 * size_x / 4), 7 * size_y / 8);

    t0 = std::chrono::steady_clock::now();

    const astar_planner::PathResult astar_result = astar.findPath(start_coord, goal_coord);

    std::cout
        << "planner took: "
        << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count()
        << std::endl;

    std::cout << "path size: " << astar_result.path.size() << std::endl;
    std::cout << "path cost: " << astar_result.cost << std::endl;

    cv::Mat costmap_u8; // (size_y, size_x, CV_8UC3); // , cv::Scalar(255, 255, 255));
    costmap.convertTo(costmap_u8, CV_8U, 255.0);

    cv::Mat disp;
    cv::cvtColor(costmap_u8, disp, cv::COLOR_GRAY2BGR);

    for (std::size_t i = 0; i < astar.gridMap().size(); ++i)
    {
        if (astar.gridMap()[i])
            disp.at<cv::Vec3b>(i) = cv::Vec3b(0, 255, 0);
    }

    cv::circle(disp, cv::Point(size_x / 4.0, size_y / 2), 116, cv::Scalar(0, 255, 255), -1, cv::LINE_8);
    cv::circle(disp, cv::Point(3 * size_x / 4.0, size_y / 2), 80, cv::Scalar(0, 255, 255), -1, cv::LINE_8);
    cv::circle(disp, cv::Point(size_x / 2.0, 3 * size_y / 4.0), 6, cv::Scalar(0, 255, 255), -1, cv::LINE_8);
    cv::circle(disp, cv::Point(size_x / 2.0, size_y / 5.0), 1, cv::Scalar(0, 255, 255), -1, cv::LINE_8);

    for (const auto& n : astar_result.path)
    {
        disp.at<cv::Vec3b>(n.y, n.x) = cv::Vec3b(255, 0, 0);
    }

    cv::imwrite("test.png", disp);
}

TEST(test_plugin, test_plugin)
{
    const double resolution = 0.02;
    const int size_x = 20 / resolution;
    const int size_y = 20 / resolution;

    hd_map::Map hd_map;
    hd_map.info.meta_data.width = size_x;
    hd_map.info.meta_data.height = size_y;
    const gridmap::MapDimensions map_dims(resolution, {-(size_x / 2) * resolution, -(size_y / 2) * resolution}, {size_x, size_y});

    std::shared_ptr<gridmap::MapData> map_data = std::make_shared<gridmap::MapData>(hd_map, map_dims);

    std::cout << "map size: " << map_data->grid.dimensions().size().transpose() << std::endl;

    cv::Mat cv_im = cv::Mat(map_data->grid.dimensions().size().y(),
                            map_data->grid.dimensions().size().x(),
                            CV_8U,
                            reinterpret_cast<void*>(const_cast<uint8_t*>(map_data->grid.cells().data())));

    cv::circle(cv_im, cv::Point(size_x / 4.0, size_y / 2), 116, cv::Scalar(gridmap::OccupancyGrid::OCCUPIED), -1, cv::LINE_8);
    cv::circle(cv_im, cv::Point(3 * size_x / 4.0, size_y / 2), 80, cv::Scalar(gridmap::OccupancyGrid::OCCUPIED), -1, cv::LINE_8);
    cv::circle(cv_im, cv::Point(size_x / 2.0, 3 * size_y / 4.0), 6, cv::Scalar(gridmap::OccupancyGrid::OCCUPIED), -1, cv::LINE_8);
    cv::circle(cv_im, cv::Point(size_x / 2.0, size_y / 5.0), 1, cv::Scalar(gridmap::OccupancyGrid::OCCUPIED), -1, cv::LINE_8);

    auto t0 = std::chrono::steady_clock::now();

    astar_planner::AStarPlanner plugin;

    XmlRpc::XmlRpcValue params;
    params["neutral_cost"] = XmlRpc::XmlRpcValue(0.2);
    params["robot_radius"] = XmlRpc::XmlRpcValue(0.5);
    params["down_sample"] = XmlRpc::XmlRpcValue(4);

    plugin.initialize(params, map_data);

    const Eigen::Isometry2d start = Eigen::Translation2d(0, -(size_y / 3) * resolution) * Eigen::Rotation2Dd(0);
    const Eigen::Isometry2d goal = Eigen::Translation2d(-(size_x / 4) * resolution, (size_y / 3) * resolution) * Eigen::Rotation2Dd(1.0);

    t0 = std::chrono::steady_clock::now();

    auto result = plugin.plan(start, goal);

    std::cout
        << "planner took: "
        << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count()
        << std::endl;

    std::cout << "path size: " << result.path.nodes.size() << std::endl;
    std::cout << "path cost: " << result.cost << std::endl;

    const double cost_check = plugin.cost(result.path);

    std::cout << "path cost check: " << cost_check << std::endl;

    cv::Mat disp(map_data->grid.dimensions().size().y(),
                 map_data->grid.dimensions().size().x(), CV_8SC3, cv::Scalar(0, 0, 0));
    cv::circle(disp, cv::Point(size_x / 4.0, size_y / 2), 116, cv::Scalar(0, 255, 0), -1, cv::LINE_8);
    cv::circle(disp, cv::Point(3 * size_x / 4.0, size_y / 2), 80, cv::Scalar(0, 255, 0), -1, cv::LINE_8);
    cv::circle(disp, cv::Point(size_x / 2.0, 3 * size_y / 4.0), 6, cv::Scalar(0, 255, 0), -1, cv::LINE_8);
    cv::circle(disp, cv::Point(size_x / 2.0, size_y / 5.0), 1, cv::Scalar(0, 255, 0), -1, cv::LINE_8);

    for (const auto& n : result.path.nodes)
    {
        const auto mp = map_data->grid.dimensions().getCellIndex(n.translation());
        cv::circle(disp, cv::Point(mp.x(), mp.y()), 1, cv::Scalar(255, 0, 0), 1, cv::LINE_8);

        const auto x_end = Eigen::Vector2d(mp.x(), mp.y()) + Eigen::Vector2d(Eigen::Rotation2Dd(n.rotation()) * Eigen::Vector2d(6, 0));
        const auto y_end = Eigen::Vector2d(mp.x(), mp.y()) + Eigen::Vector2d(Eigen::Rotation2Dd(n.rotation()) * Eigen::Vector2d(0, 6));

        cv::line(disp,
                 cv::Point(mp.x(), mp.y()),
                 cv::Point(x_end.x(), x_end.y()),
                 cv::Scalar(0, 0, 255), 1);

        cv::line(disp,
                 cv::Point(mp.x(), mp.y()),
                 cv::Point(y_end.x(), y_end.y()),
                 cv::Scalar(0, 255, 0), 1);
    }

    cv::imwrite("test_plugin.png", disp);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
