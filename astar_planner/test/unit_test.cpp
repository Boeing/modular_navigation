#include <gtest/gtest.h>

#include <astar_planner/astar.h>
#include <astar_planner/plugin.h>

#include <chrono>
#include <deque>
#include <random>

#include <gridmap/map_data.h>

#include <hd_map/Map.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

TEST(test_astar, test_2d_astar)
{
    const double resolution = 0.02;
    const double robot_radius = 0.420;
    const int size_x = static_cast<int>(20.0 / resolution);
    const int size_y = static_cast<int>(20.0 / resolution);

    auto costmap = std::make_shared<astar_planner::Costmap>();
    costmap->width = size_x;
    costmap->height = size_y;
    costmap->resolution = resolution;
    costmap->origin_x = -(size_x / 2) * resolution;
    costmap->origin_y = -(size_y / 2) * resolution;

    costmap->obstacle_map = cv::Mat(size_x, size_y, CV_8U, cv::Scalar(0));
    cv::circle(costmap->obstacle_map, cv::Point(size_x / 4.0, size_y / 2), 200, cv::Scalar(255), -1, cv::LINE_8);
    cv::circle(costmap->obstacle_map, cv::Point(3 * size_x / 4.0, size_y / 2), 80, cv::Scalar(255), -1, cv::LINE_8);
    cv::circle(costmap->obstacle_map, cv::Point(size_x / 2.0, 3 * size_y / 4.0), 6, cv::Scalar(255), -1, cv::LINE_8);
    cv::circle(costmap->obstacle_map, cv::Point(size_x / 2.0, size_y / 5.0), 1, cv::Scalar(255), -1, cv::LINE_8);

    costmap->traversal_cost = std::make_shared<cv::Mat>(size_y, size_x, CV_32F, cv::Scalar(1.0));

    auto t0 = std::chrono::steady_clock::now();

    // dilate robot radius
    cv::Mat dilated;
    const int cell_inflation_radius = static_cast<int>(2.0 * robot_radius / resolution);
    auto ellipse = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(cell_inflation_radius, cell_inflation_radius));
    cv::dilate(costmap->obstacle_map, dilated, ellipse);

    // flip
    cv::bitwise_not(dilated, dilated);

    // find obstacle distances
    cv::distanceTransform(dilated, costmap->distance_to_collision, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);

    std::cout
        << "preparing costmap took: "
        << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count()
        << std::endl;

    const astar_planner::State2D goal{size_x / 2, size_y / 2};

    astar_planner::Explore2DCache explore_cache(size_x, size_y);

    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0, size_x - 1);

    for (size_t i = 0; i < 100; ++i)
    {
        const astar_planner::State2D start{distribution(generator), distribution(generator)};

        t0 = std::chrono::steady_clock::now();

        const auto shortest_2d = astar_planner::shortestPath2D(start, goal, explore_cache, *costmap.get());

        std::cout
            << "path to: (" << start.x << ", " << start.y << ")"
            << " success: " << shortest_2d.success << " in " << shortest_2d.iterations << " took: "
            << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count()
            << std::endl;

        /*
        {
            cv::Mat gray = costmap->obstacle_map.clone();
            cv::Mat disp;
            cv::cvtColor(gray, disp, cv::COLOR_GRAY2RGB);

            for (const auto& node : explore_cache.explore_2d)
            {
                if (node)
                {
                    const double c = node->cost_so_far * 0.4;
                    disp.at<cv::Vec3b>(node->state.y, node->state.x) = cv::Vec3b(c, c, 0);
                }
            }

            auto node = explore_cache.explore_2d[costmap->to2DGridIndex(start)];
            if (node)
            {
                do
                {
                    disp.at<cv::Vec3b>(node->state.y, node->state.x) = cv::Vec3b(0, 200, 0);
                    node = node->parent;
                } while (node);
            }

            disp.at<cv::Vec3b>(start.y, start.x) = cv::Vec3b(255, 0, 0);
            disp.at<cv::Vec3b>(goal.y, goal.x) = cv::Vec3b(255, 0, 0);

            cv::namedWindow("2d", cv::WINDOW_NORMAL);
            cv::imshow("2d", disp);
            cv::waitKey(50);
        }
        */
    }
}

TEST(test_astar, test_hybrid_astar)
{
    const double resolution = 0.02;
    const double robot_radius = 0.420;
    const int size_x = static_cast<int>(20.0 / resolution);
    const int size_y = static_cast<int>(20.0 / resolution);

    auto costmap = std::make_shared<astar_planner::Costmap>();
    costmap->width = size_x;
    costmap->height = size_y;
    costmap->resolution = resolution;
    costmap->origin_x = -(size_x / 2.) * resolution;
    costmap->origin_y = -(size_y / 2.) * resolution;

    costmap->obstacle_map = cv::Mat(size_x, size_y, CV_8U, cv::Scalar(0));
    cv::rectangle(costmap->obstacle_map, cv::Point(0, 400), cv::Point(400, 900), cv::Scalar(255), -1, cv::LINE_8);
    //    cv::rectangle(costmap->obstacle_map, cv::Point(470, 50), cv::Point(800, 800), cv::Scalar(255), -1,
    //    cv::LINE_8);
    cv::rectangle(costmap->obstacle_map, cv::Point(200, 200), cv::Point(1000, 300), cv::Scalar(255), -1, cv::LINE_8);

    costmap->traversal_cost = std::make_shared<cv::Mat>(size_y, size_x, CV_32F, cv::Scalar(1.0));
    cv::rectangle(*costmap->traversal_cost, cv::Point(470, 50), cv::Point(800, 800), cv::Scalar(10.0), -1, cv::LINE_8);

    auto t0 = std::chrono::steady_clock::now();

    // dilate robot radius
    cv::Mat dilated;
    const int cell_inflation_radius = static_cast<int>(2.0 * robot_radius / resolution);
    auto ellipse = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(cell_inflation_radius, cell_inflation_radius));
    cv::dilate(costmap->obstacle_map, dilated, ellipse);

    // flip
    cv::bitwise_not(dilated, dilated);

    // find obstacle distances
    cv::distanceTransform(dilated, costmap->distance_to_collision, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);

    std::cout
        << "preparing costmap took: "
        << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count()
        << std::endl;

    const std::vector<Eigen::Vector2d> offsets = {{-0.268, 0.000},  {0.268, 0.000},   {0.265, -0.185}, {0.077, -0.185},
                                                  {-0.077, -0.185}, {-0.265, -0.185}, {0.265, 0.185},  {-0.265, 0.185},
                                                  {-0.077, 0.185},  {0.077, 0.185}};

    // backwards
    const Eigen::Isometry2d start = Eigen::Translation2d(-6, -7.1) * Eigen::Rotation2Dd(0.0);

    //    const Eigen::Isometry2d start = Eigen::Translation2d(-4, 0) * Eigen::Rotation2Dd(0);
    //    const Eigen::Isometry2d goal = Eigen::Translation2d(4, 0) * Eigen::Rotation2Dd(M_PI / 2.0);

    // corridor
    //    const Eigen::Isometry2d start = Eigen::Translation2d(-8, -8) * Eigen::Rotation2Dd(M_PI);
    const Eigen::Isometry2d goal = Eigen::Translation2d(5.5, 0) * Eigen::Rotation2Dd(M_PI);

    // strafe
    //    const Eigen::Isometry2d start = Eigen::Translation2d(-6, -8) * Eigen::Rotation2Dd(-M_PI / 2);
    //    const Eigen::Isometry2d goal = Eigen::Translation2d(-8, -8) * Eigen::Rotation2Dd(-M_PI / 2);

    const size_t max_iterations = 1e6;
    const astar_planner::CollisionChecker collision_checker(costmap);
    const double linear_resolution = 0.08;
    const double angular_resolution = 0.1;
    const bool allow_backwards = false;
    const bool allow_strafe = false;

    t0 = std::chrono::steady_clock::now();

    const astar_planner::PathResult astar_result =
        astar_planner::hybridAStar(start, goal, max_iterations, *costmap.get(), collision_checker, 0.48f,
                                   linear_resolution, angular_resolution, allow_backwards, allow_strafe);

    std::cout
        << "planner took: "
        << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count()
        << std::endl;

    std::cout << "path size: " << astar_result.path.size() << std::endl;
    std::cout << "success: " << astar_result.success << std::endl;
    std::cout << "iterations: " << astar_result.iterations << std::endl;

    cv::Mat costmap_u8;
    costmap->distance_to_collision.convertTo(costmap_u8, CV_8U, 1.0);

    cv::Mat disp;
    cv::cvtColor(costmap_u8, disp, cv::COLOR_GRAY2BGR);

    cv::rectangle(disp, cv::Point(0, 400), cv::Point(400, 900), cv::Scalar(0, 255, 255), -1, cv::LINE_8);
    cv::rectangle(disp, cv::Point(470, 50), cv::Point(800, 800), cv::Scalar(0, 50, 50), -1, cv::LINE_8);
    cv::rectangle(disp, cv::Point(200, 200), cv::Point(1000, 300), cv::Scalar(0, 255, 255), -1, cv::LINE_8);

    const int mx = static_cast<int>(std::round((goal.translation().x() - costmap->origin_x) / costmap->resolution));
    const int my = static_cast<int>(std::round((goal.translation().y() - costmap->origin_y) / costmap->resolution));
    cv::circle(disp, cv::Point(mx, my), 10, cv::Scalar(0, 255, 255), 2, cv::LINE_8);

    for (const auto& node : astar_result.explore_cache.explore_2d)
    {
        if (node)
        {
            const double c = node->cost_so_far * 0.2 * 0.5;
            disp.at<cv::Vec3b>(node->state.y, node->state.x) += cv::Vec3b(c, c, 0);
        }
    }

    for (auto node : astar_result.explore_3d)
    {
        if (!node.second->parent)
            continue;

        const int start_x =
            static_cast<int>(std::round((node.second->state.x - costmap->origin_x) / costmap->resolution));
        const int start_y =
            static_cast<int>(std::round((node.second->state.y - costmap->origin_y) / costmap->resolution));

        const int end_x =
            static_cast<int>(std::round((node.second->parent->state.x - costmap->origin_x) / costmap->resolution));
        const int end_y =
            static_cast<int>(std::round((node.second->parent->state.y - costmap->origin_y) / costmap->resolution));

        cv::line(disp, cv::Point(start_x, start_y), cv::Point(end_x, end_y), cv::Scalar(100, 100, 0), 1, cv::LINE_8);

        const Eigen::Vector2d x_end =
            Eigen::Vector2d(start_x, start_y) +
            Eigen::Vector2d(Eigen::Rotation2Dd(node.second->state.theta) * Eigen::Vector2d(4, 0));
        const Eigen::Vector2d y_end =
            Eigen::Vector2d(start_x, start_y) +
            Eigen::Vector2d(Eigen::Rotation2Dd(node.second->state.theta) * Eigen::Vector2d(0, 4));

        cv::line(disp, cv::Point(start_x, start_y), cv::Point(x_end.x(), x_end.y()), cv::Scalar(0, 0, 100), 1);
        cv::line(disp, cv::Point(start_x, start_y), cv::Point(y_end.x(), y_end.y()), cv::Scalar(0, 100, 0), 1);
    }

    if (astar_result.success)
    {

        for (size_t i = 0; i < astar_result.path.size(); ++i)
        {
            auto& node = astar_result.path[i];

            const Eigen::Vector2d position(node->state.x, node->state.y);
            const Eigen::Rotation2Dd rotation(node->state.theta);

            if (i % 20 == 0 || i == astar_result.path.size() - 1)
            {
                for (std::size_t c = 0; c < offsets.size(); ++c)
                {
                    const Eigen::Vector2d cp_pose = position + rotation * offsets[c];
                    const int cx =
                        static_cast<int>(std::round((cp_pose.x() - costmap->origin_x) / costmap->resolution));
                    const int cy =
                        static_cast<int>(std::round((cp_pose.y() - costmap->origin_y) / costmap->resolution));
                    cv::circle(disp, cv::Point(cx, cy), robot_radius / resolution, cv::Scalar(0, 255, 0), 1);
                }
            }

            if (!node->parent)
                continue;

            const int start_x = static_cast<int>(std::round((node->state.x - costmap->origin_x) / costmap->resolution));
            const int start_y = static_cast<int>(std::round((node->state.y - costmap->origin_y) / costmap->resolution));

            const int end_x =
                static_cast<int>(std::round((node->parent->state.x - costmap->origin_x) / costmap->resolution));
            const int end_y =
                static_cast<int>(std::round((node->parent->state.y - costmap->origin_y) / costmap->resolution));

            cv::line(disp, cv::Point(start_x, start_y), cv::Point(end_x, end_y), cv::Scalar(100, 255, 0), 1,
                     cv::LINE_8);

            const Eigen::Vector2d x_end =
                Eigen::Vector2d(start_x, start_y) +
                Eigen::Vector2d(Eigen::Rotation2Dd(node->state.theta) * Eigen::Vector2d(6, 0));
            const Eigen::Vector2d y_end =
                Eigen::Vector2d(start_x, start_y) +
                Eigen::Vector2d(Eigen::Rotation2Dd(node->state.theta) * Eigen::Vector2d(0, 6));

            cv::line(disp, cv::Point(start_x, start_y), cv::Point(x_end.x(), x_end.y()), cv::Scalar(0, 0, 255), 1);
            cv::line(disp, cv::Point(start_x, start_y), cv::Point(y_end.x(), y_end.y()), cv::Scalar(0, 255, 0), 1);
        }
    }

    for (const auto& n : astar_result.path)
    {
        const int x = static_cast<int>(std::round((n->state.x - costmap->origin_x) / costmap->resolution));
        const int y = static_cast<int>(std::round((n->state.y - costmap->origin_y) / costmap->resolution));
        disp.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 0, 0);
    }

    cv::imwrite("test.png", disp);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
