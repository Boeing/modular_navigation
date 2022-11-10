#include <Eigen/Core>

#include <gridmap/grids/grid_2d.h>
#include <gridmap/operations/rasterize.h>
#include <gridmap/operations/raytrace.h>
#include <gtest/gtest.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// For logging reasons
#include "rclcpp/rclcpp.hpp"

/*
TEST(test_raytrace, test_raytrace)
{
    gridmap::MapDimensions map_dims(1, {0, 0}, {1000, 1000});

    gridmap::Grid2D<uint8_t> grid(map_dims);

    cv::Mat cv_im = cv::Mat(grid.dimensions().size().y(), grid.dimensions().size().x(), CV_8U,
                            reinterpret_cast<void*>(grid.cells().data()));

    for (int t=0; t<40; ++t)
    {
        const auto t0 = std::chrono::steady_clock::now();

        const std::vector<Eigen::Array2i> line = gridmap::drawLine(map_dims.size().x(), map_dims.size().y(), 0, 0);

        RCLCPP_INFO_STREAM(
                        "efla took "
                        << std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() -
t0).count());

        for (std::size_t i=0; i<255; ++i)
        {
            cv_im.at<uint8_t>(cv::Point(line[i].x(), line[i].y())) = static_cast<uint8_t>(i);
        }
    }

    cv::imwrite("grid.png", cv_im);
}
*/

TEST(test_polyfill, test_polyfill)
{
    gridmap::MapDimensions map_dims(1, {0, 0}, {1000, 1000});

    gridmap::Grid2D<uint8_t> grid(map_dims);

    cv::Mat cv_im = cv::Mat(grid.dimensions().size().y(), grid.dimensions().size().x(), CV_8U,
                            reinterpret_cast<void*>(grid.cells().data()));

    std::vector<Eigen::Array2i> polygon;
    polygon.push_back({500, 500});
    polygon.push_back({600, 500});
    polygon.push_back({600, 600});
    polygon.push_back({400, 600});

    auto lambda = [&cv_im](int x, int y) { cv_im.at<uint8_t>(y, x) = 255; };

    for (int t = 0; t < 40; ++t)
    {
        const auto t0 = std::chrono::steady_clock::now();

        gridmap::rasterPolygonFill(lambda, polygon, 400, 600, 500, 600);

        RCLCPP_INFO_STREAM(rclcpp::get_logger(""),
                           "rasterPolygonFill took " << std::chrono::duration_cast<std::chrono::duration<double>>(
                                                            std::chrono::steady_clock::now() - t0)
                                                            .count());
    }

    cv::imwrite("grid.png", cv_im);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
