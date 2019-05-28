#include <gtest/gtest.h>

#include <gridmap/grids/grid_2d.h>

#include <chrono>

#include <gridmap/map_data.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

TEST(test_plugin, test_plugin)
{
    gridmap::MapDimensions map_dims(1, {0, 0}, {1000, 1000});

    gridmap::Grid2D<uint8_t> grid(map_dims);

    cv::Mat cv_im = cv::Mat(grid.dimensions().size().y(), grid.dimensions().size().x(), CV_8U,
                            reinterpret_cast<void*>(grid.cells().data()));

    cv::circle(cv_im, cv::Point(500, 500), 400, cv::Scalar(255), -1);

    const Eigen::Array2i roi_size(900, 900);
    gridmap::AABB roi{grid.dimensions().size() / 2 - roi_size / 2, roi_size};

    gridmap::Grid2D<uint8_t> roi_grid(map_dims);  // {1, {0, 0}, roi_size});

    ROS_INFO_STREAM("roi_start: " << roi.roi_start.transpose());
    ROS_INFO_STREAM("roi_size: " << roi.roi_size.transpose());

    grid.copyTo(roi_grid, roi);

    cv::Mat cv_im_roi = cv::Mat(roi_grid.dimensions().size().y(), roi_grid.dimensions().size().x(), CV_8U,
                                reinterpret_cast<void*>(roi_grid.cells().data()));

    cv::imwrite("grid.png", cv_im);
    cv::imwrite("grid_roi.png", cv_im_roi);
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
