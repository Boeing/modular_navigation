#include <astar_planner/astar.h>
#include <astar_planner/plugin.h>
#include <astar_planner/visualisation.h>
#include <gridmap/map_data.h>
#include <gtest/gtest.h>
#include <hd_map/Map.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <deque>
#include <random>

class PlanningTest : public testing::Test
{
  protected:
    // cppcheck-suppress unusedFunction
    virtual void SetUp()
    {
        const Eigen::Vector2d origin = {-(size_x / 2.) * resolution, -(size_y / 2.) * resolution};

        const hd_map::Map hd_map;
        const gridmap::MapDimensions dimensions(resolution, origin, {size_x, size_y});
        map_data = std::make_shared<gridmap::MapData>(hd_map, dimensions);

        cv_im = cv::Mat(map_data->grid.dimensions().size().y(), map_data->grid.dimensions().size().x(), CV_8U,
                        reinterpret_cast<void*>(map_data->grid.cells().data()));
    }

    const double resolution = 0.02;
    const double robot_radius = 0.230;
    const double conservative_radius = 0.400;

    const int size_x = static_cast<int>(20.0 / resolution);
    const int size_y = static_cast<int>(20.0 / resolution);

    cv::Mat cv_im;
    std::shared_ptr<gridmap::MapData> map_data;

    const size_t max_iterations = 1e5;
    const double linear_resolution = 0.02;
    const double angular_resolution = 2 * M_PI / 16;

    const std::vector<Eigen::Vector2d> offsets = {{-0.268, 0.000},  {0.268, 0.000},   {0.265, -0.185}, {0.077, -0.185},
                                                  {-0.077, -0.185}, {-0.265, -0.185}, {0.265, 0.185},  {-0.265, 0.185},
                                                  {-0.077, 0.185},  {0.077, 0.185}};
};

TEST_F(PlanningTest, test_home_position)
{
    cv::circle(cv_im, cv::Point(250, 310), static_cast<int>(0.1 / resolution), cv::Scalar(255), -1);
    cv::circle(cv_im, cv::Point(250, 340), static_cast<int>(0.1 / resolution), cv::Scalar(255), -1);
    cv::circle(cv_im, cv::Point(316, 310), static_cast<int>(0.1 / resolution), cv::Scalar(255), -1);
    cv::circle(cv_im, cv::Point(316, 340), static_cast<int>(0.1 / resolution), cv::Scalar(255), -1);
    cv::rectangle(cv_im, cv::Point(200, 200), cv::Point(1000, 300), cv::Scalar(255), -1, cv::LINE_8);

    auto costmap = astar_planner::buildCostmap(*map_data, robot_radius);

    // Set unit traversal cost
    costmap->traversal_cost = std::make_shared<cv::Mat>(size_y, size_x, CV_32F, cv::Scalar(1.0));

    const Eigen::Isometry2d start = Eigen::Translation2d(-4.8, -2.2) * Eigen::Rotation2Dd(0);
    const Eigen::Isometry2d goal = Eigen::Translation2d(-4.42, -3.5) * Eigen::Rotation2Dd(M_PI);

    const astar_planner::CollisionChecker collision_checker(*costmap, offsets);

    const auto t0 = std::chrono::steady_clock::now();

    const astar_planner::PathResult astar_result =
        astar_planner::hybridAStar(start, goal, max_iterations, *costmap.get(), collision_checker, conservative_radius,
                                   linear_resolution, angular_resolution);

    std::cout
        << "planner took: "
        << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count()
        << std::endl;

    std::cout << "path size: " << astar_result.path.size() << std::endl;
    std::cout << "success: " << astar_result.success << std::endl;
    std::cout << "iterations: " << astar_result.iterations << std::endl;

    const cv::Mat disp = astar_planner::visualise(*costmap, astar_result);
    cv::imwrite("test_home_position.png", disp);
}

TEST_F(PlanningTest, test_out_of_lane)
{
    cv::rectangle(cv_im, cv::Point(200, 200), cv::Point(1000, 300), cv::Scalar(255), -1, cv::LINE_8);
    cv::rectangle(cv_im, cv::Point(200, 400), cv::Point(1000, 500), cv::Scalar(255), -1, cv::LINE_8);
    cv::rectangle(cv_im, cv::Point(0, 0), cv::Point(140, 500), cv::Scalar(255), -1, cv::LINE_8);

    auto costmap = astar_planner::buildCostmap(*map_data, robot_radius);

    // Set unit traversal cost
    costmap->traversal_cost = std::make_shared<cv::Mat>(size_y, size_x, CV_32F, cv::Scalar(1.0));

    const Eigen::Isometry2d start = Eigen::Translation2d(-4, -7.1) * Eigen::Rotation2Dd(M_PI);
    const Eigen::Isometry2d goal = Eigen::Translation2d(-4, -3.2) * Eigen::Rotation2Dd(M_PI);

    const astar_planner::CollisionChecker collision_checker(*costmap, offsets);

    const auto t0 = std::chrono::steady_clock::now();

    const astar_planner::PathResult astar_result =
        astar_planner::hybridAStar(start, goal, max_iterations, *costmap.get(), collision_checker, conservative_radius,
                                   linear_resolution, angular_resolution);

    std::cout
        << "planner took: "
        << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count()
        << std::endl;

    std::cout << "path size: " << astar_result.path.size() << std::endl;
    std::cout << "success: " << astar_result.success << std::endl;
    std::cout << "iterations: " << astar_result.iterations << std::endl;

    const cv::Mat disp = astar_planner::visualise(*costmap, astar_result);
    cv::imwrite("test_out_of_lane.png", disp);
}

TEST_F(PlanningTest, test_straight_line)
{
    cv::rectangle(cv_im, cv::Point(200, 200), cv::Point(1000, 320), cv::Scalar(255), -1, cv::LINE_8);
    cv::rectangle(cv_im, cv::Point(200, 420), cv::Point(1000, 500), cv::Scalar(255), -1, cv::LINE_8);

    auto costmap = astar_planner::buildCostmap(*map_data, robot_radius);

    // Set unit traversal cost
    costmap->traversal_cost = std::make_shared<cv::Mat>(size_y, size_x, CV_32F, cv::Scalar(1.0));

    const Eigen::Isometry2d start = Eigen::Translation2d(0.0, -3.1) * Eigen::Rotation2Dd(M_PI);
    const Eigen::Isometry2d goal = Eigen::Translation2d(6.0, -3.1) * Eigen::Rotation2Dd(0);

    const astar_planner::CollisionChecker collision_checker(*costmap, offsets);

    const auto t0 = std::chrono::steady_clock::now();

    const astar_planner::PathResult astar_result =
        astar_planner::hybridAStar(start, goal, max_iterations, *costmap.get(), collision_checker, conservative_radius,
                                   linear_resolution, angular_resolution);

    std::cout
        << "planner took: "
        << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count()
        << std::endl;

    std::cout << "path size: " << astar_result.path.size() << std::endl;
    std::cout << "success: " << astar_result.success << std::endl;
    std::cout << "iterations: " << astar_result.iterations << std::endl;

    const cv::Mat disp = astar_planner::visualise(*costmap, astar_result);
    cv::imwrite("test_straight_line.png", disp);
}

TEST_F(PlanningTest, test_avoid_zone)
{
    cv::rectangle(cv_im, cv::Point(200, 200), cv::Point(1000, 320), cv::Scalar(255), -1, cv::LINE_8);
    //    cv::rectangle(cv_im, cv::Point(200, 420), cv::Point(1000, 500), cv::Scalar(255), -1, cv::LINE_8);

    auto costmap = astar_planner::buildCostmap(*map_data, robot_radius);

    // Set unit traversal cost
    costmap->traversal_cost = std::make_shared<cv::Mat>(size_y, size_x, CV_32F, cv::Scalar(1.0));
    cv::rectangle(*costmap->traversal_cost, cv::Point(600, 200), cv::Point(800, 500), cv::Scalar(10.0), -1, cv::LINE_8);

    const Eigen::Isometry2d start = Eigen::Translation2d(0.0, -3.1) * Eigen::Rotation2Dd(M_PI);
    const Eigen::Isometry2d goal = Eigen::Translation2d(5.5, -3.1) * Eigen::Rotation2Dd(0);

    const astar_planner::CollisionChecker collision_checker(*costmap, offsets);

    const auto t0 = std::chrono::steady_clock::now();

    const astar_planner::PathResult astar_result =
        astar_planner::hybridAStar(start, goal, max_iterations, *costmap.get(), collision_checker, conservative_radius,
                                   linear_resolution, angular_resolution);

    std::cout
        << "planner took: "
        << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count()
        << std::endl;

    std::cout << "path size: " << astar_result.path.size() << std::endl;
    std::cout << "success: " << astar_result.success << std::endl;
    std::cout << "iterations: " << astar_result.iterations << std::endl;

    cv::Mat disp = astar_planner::visualise(*costmap, astar_result);
    //    cv::rectangle(disp, cv::Point(600, 200), cv::Point(800, 500), cv::Scalar(100, 0, 0), -1, cv::LINE_8);
    cv::imwrite("test_avoid_zone.png", disp);
}

TEST_F(PlanningTest, test_reverse)
{
    cv::rectangle(cv_im, cv::Point(200, 200), cv::Point(1000, 320), cv::Scalar(255), -1, cv::LINE_8);
    cv::rectangle(cv_im, cv::Point(200, 420), cv::Point(1000, 500), cv::Scalar(255), -1, cv::LINE_8);

    auto costmap = astar_planner::buildCostmap(*map_data, robot_radius);

    // Set unit traversal cost
    costmap->traversal_cost = std::make_shared<cv::Mat>(size_y, size_x, CV_32F, cv::Scalar(1.0));

    const Eigen::Isometry2d start = Eigen::Translation2d(0.0, -2.8) * Eigen::Rotation2Dd(0);
    const Eigen::Isometry2d goal = Eigen::Translation2d(-2.0, -2.8) * Eigen::Rotation2Dd(0);

    const astar_planner::CollisionChecker collision_checker(*costmap, offsets);

    const auto t0 = std::chrono::steady_clock::now();

    const astar_planner::PathResult astar_result =
        astar_planner::hybridAStar(start, goal, max_iterations, *costmap.get(), collision_checker, conservative_radius,
                                   linear_resolution, angular_resolution);

    std::cout
        << "planner took: "
        << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count()
        << std::endl;

    std::cout << "path size: " << astar_result.path.size() << std::endl;
    std::cout << "success: " << astar_result.success << std::endl;
    std::cout << "iterations: " << astar_result.iterations << std::endl;

    const cv::Mat disp = astar_planner::visualise(*costmap, astar_result);
    cv::imwrite("test_reverse.png", disp);
}

TEST_F(PlanningTest, test_strafe)
{
    cv::rectangle(cv_im, cv::Point(200, 200), cv::Point(1000, 320), cv::Scalar(255), -1, cv::LINE_8);
    cv::rectangle(cv_im, cv::Point(200, 420), cv::Point(1000, 500), cv::Scalar(255), -1, cv::LINE_8);

    auto costmap = astar_planner::buildCostmap(*map_data, robot_radius);

    // Set unit traversal cost
    costmap->traversal_cost = std::make_shared<cv::Mat>(size_y, size_x, CV_32F, cv::Scalar(1.0));

    const Eigen::Isometry2d start = Eigen::Translation2d(0.0, -2.8) * Eigen::Rotation2Dd(M_PI / 2.0);
    const Eigen::Isometry2d goal = Eigen::Translation2d(0.4, -2.8) * Eigen::Rotation2Dd(M_PI / 2.0);

    const astar_planner::CollisionChecker collision_checker(*costmap, offsets);

    const auto t0 = std::chrono::steady_clock::now();

    const astar_planner::PathResult astar_result =
        astar_planner::hybridAStar(start, goal, max_iterations, *costmap.get(), collision_checker, conservative_radius,
                                   linear_resolution, angular_resolution);

    std::cout
        << "planner took: "
        << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t0).count()
        << std::endl;

    std::cout << "path size: " << astar_result.path.size() << std::endl;
    std::cout << "success: " << astar_result.success << std::endl;
    std::cout << "iterations: " << astar_result.iterations << std::endl;

    const cv::Mat disp = astar_planner::visualise(*costmap, astar_result);
    cv::imwrite("test_strafe.png", disp);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
