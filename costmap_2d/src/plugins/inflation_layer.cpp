#include <costmap_2d/plugins/inflation_layer.h>

#include <algorithm>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_math.h>

#include <pluginlib/class_list_macros.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>

PLUGINLIB_EXPORT_CLASS(costmap_2d::InflationLayer, costmap_2d::Layer)

namespace costmap_2d
{

InflationLayer::InflationLayer() : inflation_radius_(0.5)
{
}

InflationLayer::~InflationLayer()
{
}

void InflationLayer::onInitialize()
{
    ros::NodeHandle nh("~/" + name_);
    inflation_radius_ = nh.param("inflation_radius", inflation_radius_);
}

void InflationLayer::activate()
{
}

void InflationLayer::deactivate()
{
}

void InflationLayer::reset()
{
    onInitialize();
}

void InflationLayer::matchSize()
{
}

void InflationLayer::updateBounds(const double, const double, const double, double*, double*, double*, double*)
{
}

void InflationLayer::updateCosts(Costmap2D& master_grid, unsigned int min_i, unsigned int min_j, unsigned int max_i,
                                 unsigned int max_j)
{
    unsigned char* master_array = master_grid.getCharMap();
    const int size_x = static_cast<int>(master_grid.getSizeInCellsX());
    const int size_y = static_cast<int>(master_grid.getSizeInCellsY());
    cv::Mat cv_im = cv::Mat(size_y, size_x, CV_8UC1, reinterpret_cast<void*>(master_array));

    // copy a region of interest
    cv::Mat roi;
    const cv::Rect rect(min_i, min_j, (max_i - min_i), (max_j - min_j));
    cv_im(rect).copyTo(roi);

    // Dilate using a circle to get the inscribed radius
    const int cell_inflation_radius = static_cast<int>(inflation_radius_ / master_grid.getResolution());
    const auto ellipse =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(cell_inflation_radius, cell_inflation_radius));
    cv::Mat dilated;
    cv::dilate(roi, dilated, ellipse);

    // Set to inscribed radius cost
    dilated.setTo(costmap_2d::INSCRIBED_INFLATED_OBSTACLE, dilated > costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    roi = cv::max(dilated, roi);

    // Output
    roi.copyTo(cv_im(rect));
}
}
