#ifndef sim_band_planner_DISTANCE_FIELD_H
#define sim_band_planner_DISTANCE_FIELD_H

#include <Eigen/Geometry>

#include <chrono>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace sim_band_planner
{

inline void getRidgeFilteredImage(cv::InputArray _img, cv::OutputArray out)
{
    const int _ddepth = CV_32FC1;
    const int _dx = 2;
    const int _dy = 2;
    const int _ksize = 1;
    const double _scale = 2.5;
    const double _delta = 0.0;
    const int _borderType = cv::BORDER_CONSTANT;
    const int _out_dtype = CV_8UC1;

    cv::Mat img = _img.getMat();
    CV_Assert(img.channels() == 1 || img.channels() == 3);

    if (img.channels() == 3)
        cvtColor(img, img, cv::COLOR_BGR2GRAY);

    cv::Mat sbx, sby;
    Sobel(img, sbx, _ddepth, _dx, 0, _ksize, _scale, _delta, _borderType);
    Sobel(img, sby, _ddepth, 0, _dy, _ksize, _scale, _delta, _borderType);

    cv::Mat sbxx, sbyy, sbxy;
    Sobel(sbx, sbxx, _ddepth, _dx, 0, _ksize, _scale, _delta, _borderType);
    Sobel(sby, sbyy, _ddepth, 0, _dy, _ksize, _scale, _delta, _borderType);
    Sobel(sbx, sbxy, _ddepth, 0, _dy, _ksize, _scale, _delta, _borderType);

    cv::Mat sb2xx, sb2yy, sb2xy;
    multiply(sbxx, sbxx, sb2xx);
    multiply(sbyy, sbyy, sb2yy);
    multiply(sbxy, sbxy, sb2xy);

    cv::Mat sbxxyy;
    multiply(sbxx, sbyy, sbxxyy);

    cv::Mat rootex;
    rootex = (sb2xx + (sb2xy + sb2xy + sb2xy + sb2xy) - (sbxxyy + sbxxyy) + sb2yy);
    cv::Mat root;
    sqrt(rootex, root);
    cv::Mat ridgexp;
    ridgexp = ((sbxx + sbyy) + root);
    ridgexp.convertTo(out, _out_dtype, 0.5);
}

struct DistanceField
{
    unsigned int size_x;
    unsigned int size_y;
    double origin_x;
    double origin_y;
    double resolution;

    cv::Mat dist;
    cv::Mat dist_inv;
    cv::Mat dist_ridges;

    cv::Mat labels;
    cv::Mat labels_inv;

    std::vector<Eigen::Vector2f> label_to_index;
    std::vector<Eigen::Vector2f> label_to_index_inv;

    DistanceField() = default;

    DistanceField(const cv::Mat& cv_im, const double _origin_x, const double _origin_y, const double _resolution,
                  const double _robot_radius)
        : size_x(cv_im.cols), size_y(cv_im.rows), origin_x(_origin_x), origin_y(_origin_y), resolution(_resolution)
    {
        // Dilate robot radius
        cv::Mat dilated_im = cv_im;  // TODO see if this can be done in-place
        const int cell_inflation_radius = static_cast<int>(2.0 * _robot_radius / resolution);
        auto ellipse =
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(cell_inflation_radius, cell_inflation_radius));
        cv::dilate(cv_im, dilated_im, ellipse);

        // Invert the costmap data such that all objects are considered zeros
        cv::Mat inv_cv_im;
        cv::bitwise_not(dilated_im, inv_cv_im);

        // Calculate the distance transform to all objects (zero pixels)
        cv::Mat dist_u8;
        cv::distanceTransform(inv_cv_im, dist, labels, cv::DIST_L2, cv::DIST_MASK_PRECISE, cv::DIST_LABEL_PIXEL);
        dist.convertTo(dist_u8, CV_8U, 1.0, 0);

        // Calculate the distance transform to all non-objects from within objects
        cv::Mat dist_inv_u8;
        cv::distanceTransform(dilated_im, dist_inv, labels_inv, cv::DIST_L2, cv::DIST_MASK_PRECISE,
                              cv::DIST_LABEL_PIXEL);
        dist_inv.convertTo(dist_inv_u8, CV_8U, 1.0, 0);

        // Construct label lookup vectors
        label_to_index.reserve(size_x * size_y);
        label_to_index_inv.reserve(size_x * size_y);
        for (int row = 0; row < dist_u8.rows; ++row)
        {
            for (int col = 0; col < dist_u8.cols; ++col)
            {
                if (dist_u8.at<uchar>(row, col) == 0)
                {
                    label_to_index.push_back(Eigen::Vector2f(col, row));
                }

                if (dist_inv_u8.at<uchar>(row, col) == 0)
                {
                    label_to_index_inv.push_back(Eigen::Vector2f(col, row));
                }
            }
        }

        // Construct a distance transform to all saddle points (ridges) of cost
        cv::Mat blurred;
        cv::GaussianBlur(dist, blurred, cv::Size(7, 7), 0, 0);
        cv::Mat ridges;
        getRidgeFilteredImage(blurred, ridges);
        cv::Mat thresh;
        cv::threshold(ridges, thresh, 0, 255, cv::THRESH_BINARY_INV);
        cv::rectangle(thresh, cv::Rect(cv::Point(0, 0), thresh.size()), cv::Scalar(255), 1);
        cv::distanceTransform(thresh, dist_ridges, cv::DIST_L2, cv::DIST_MASK_PRECISE);

        cv::Mat dist_ridges_u8;
        dist_ridges.convertTo(dist_ridges_u8, CV_8U, 1.0, 0);
    }

    bool worldToMap(const double wx, const double wy, unsigned int& mx, unsigned int& my) const
    {
        if (wx < origin_x || wy < origin_y)
            return false;

        mx = static_cast<unsigned int>((wx - origin_x) / resolution);
        my = static_cast<unsigned int>((wy - origin_y) / resolution);

        if (mx < size_x && my < size_y)
            return true;

        return false;
    }

    inline float distance(const unsigned int mx, const unsigned int my) const
    {
        const float d = dist.at<float>(my, mx);
        if (d > 0)
        {
            return d;
        }
        return -dist_inv.at<float>(my, mx);
    }

    inline float distanceToSaddle(const unsigned int mx, const unsigned int my) const
    {
        return dist_ridges.at<float>(my, mx);
    }

    inline Eigen::Vector2f gradient(const unsigned int mx, const unsigned int my) const
    {
        if (dist.at<float>(my, mx) > 0)
        {
            return positiveGradient(mx, my);
        }
        else
        {
            negativeGradient(mx, my);
        }
    }

    inline Eigen::Vector2f negativeGradient(const unsigned int mx, const unsigned int my) const
    {
        return (label_to_index_inv[labels_inv.at<int>(my, mx) - 1] - Eigen::Vector2f(mx, my)).normalized();
    }

    inline Eigen::Vector2f positiveGradient(const unsigned int mx, const unsigned int my) const
    {
        return (label_to_index[labels.at<int>(my, mx) - 1] - Eigen::Vector2f(mx, my)).normalized();
    }

    double distance(const Eigen::Vector2d& pose) const
    {
        unsigned int cell_x;
        unsigned int cell_y;
        float px = 0;

        if (worldToMap(pose.x(), pose.y(), cell_x, cell_y))
        {
            // get cost for this cell
            px = distance(cell_x, cell_y);
        }

        const double distance = static_cast<double>(px) * resolution;
        return distance;
    }
};
}

#endif
