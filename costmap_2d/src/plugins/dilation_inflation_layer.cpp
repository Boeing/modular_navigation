#include <algorithm>
#include <boost/thread.hpp>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/footprint.h>
#include <costmap_2d/plugins/dilation_inflation_layer.h>

#include <pluginlib/class_list_macros.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>

#include <chrono>
#include <fstream>
#include <ostream>

PLUGINLIB_EXPORT_CLASS(costmap_2d::DilationInflationLayer, costmap_2d::Layer)

namespace costmap_2d
{

DilationInflationLayer::DilationInflationLayer()
    : resolution_(1), inflation_radius_(0), inscribed_radius_(0), weight_(0), inflate_unknown_(false),
      cell_inflation_radius_(0), cached_cell_inflation_radius_(0), inflation_cells_({}), dsrv_(nullptr), seen_(nullptr),
      seen_size_(0), cached_costs_(nullptr), cached_distances_(nullptr),
      last_min_x_(-std::numeric_limits<float>::max()), last_min_y_(-std::numeric_limits<float>::max()),
      last_max_x_(std::numeric_limits<float>::max()), last_max_y_(std::numeric_limits<float>::max()),
      need_reinflation_(false)
{
    inflation_access_ = new boost::recursive_mutex();
}

void DilationInflationLayer::onInitialize()
{
    {
        boost::unique_lock<boost::recursive_mutex> lock(*inflation_access_);
        ros::NodeHandle nh("~/" + name_), g_nh;
        current_ = true;
        if (seen_)
            delete[] seen_;
        seen_ = NULL;
        seen_size_ = 0;
        need_reinflation_ = false;

        dynamic_reconfigure::Server<costmap_2d::DilationInflationPluginConfig>::CallbackType cb =
            boost::bind(&DilationInflationLayer::reconfigureCB, this, _1, _2);

        if (dsrv_ != nullptr)
        {
            dsrv_->clearCallback();
            dsrv_->setCallback(cb);
        }
        else
        {
            dsrv_ = new dynamic_reconfigure::Server<costmap_2d::DilationInflationPluginConfig>(
                ros::NodeHandle("~/" + name_));
            dsrv_->setCallback(cb);
        }
    }

    matchSize();
}

void DilationInflationLayer::reconfigureCB(costmap_2d::DilationInflationPluginConfig& config, uint32_t)
{
    setInflationParameters(config.inflation_radius, config.cost_scaling_factor);

    if (enabled_ != config.enabled || inflate_unknown_ != config.inflate_unknown)
    {
        enabled_ = config.enabled;
        inflate_unknown_ = config.inflate_unknown;
        need_reinflation_ = true;
    }
}

void DilationInflationLayer::matchSize()
{
    boost::unique_lock<boost::recursive_mutex> lock(*inflation_access_);
    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    resolution_ = costmap->getResolution();
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    //    computeCaches();

    const unsigned int size_x = costmap->getSizeInCellsX();
    const unsigned int size_y = costmap->getSizeInCellsY();
    if (seen_)
        delete[] seen_;
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
}

void DilationInflationLayer::updateBounds(const double, const double, const double, double* min_x, double* min_y,
                                          double* max_x, double* max_y)
{
    if (need_reinflation_)
    {
        last_min_x_ = *min_x;
        last_min_y_ = *min_y;
        last_max_x_ = *max_x;
        last_max_y_ = *max_y;
        *min_x = -std::numeric_limits<double>::max();
        *min_y = -std::numeric_limits<double>::max();
        *max_x = std::numeric_limits<double>::max();
        *max_y = std::numeric_limits<double>::max();
        need_reinflation_ = false;
    }
    else
    {
        // Only increase the area to update costs of inflation

        const double tmp_min_x = last_min_x_;
        const double tmp_min_y = last_min_y_;
        const double tmp_max_x = last_max_x_;
        const double tmp_max_y = last_max_y_;
        last_min_x_ = *min_x;
        last_min_y_ = *min_y;
        last_max_x_ = *max_x;
        last_max_y_ = *max_y;
        *min_x = std::min(tmp_min_x, *min_x) - inflation_radius_;
        *min_y = std::min(tmp_min_y, *min_y) - inflation_radius_;
        *max_x = std::max(tmp_max_x, *max_x) + inflation_radius_;
        *max_y = std::max(tmp_max_y, *max_y) + inflation_radius_;
    }
}

void DilationInflationLayer::onFootprintChanged()
{
    inscribed_radius_ = layered_costmap_->getInscribedRadius();
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    //    computeCaches();
    need_reinflation_ = true;

    ROS_DEBUG("DilationInflationLayer::onFootprintChanged(): num footprint points: %lu,"
              " inscribed_radius_ = %.3f, inflation_radius_ = %.3f",
              layered_costmap_->getFootprint().size(), inscribed_radius_, inflation_radius_);
}

void DilationInflationLayer::updateCosts(Costmap2D& master_grid, unsigned int min_i, unsigned int min_j,
                                         unsigned int max_i, unsigned int max_j)
{
    auto start = std::chrono::high_resolution_clock::now();
    unsigned char* master_array = master_grid.getCharMap();
    const unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();
    cv::Mat cv_im = cv::Mat(size_y, size_x, CV_8UC1, (void*)master_array);

    // Dilate using a circle to get the inscribed radius
    cv::Mat dilated_im;
    auto ellipse =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(cell_inflation_radius_, cell_inflation_radius_));
    cv::dilate(cv_im, dilated_im,
               cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(cell_inflation_radius_, cell_inflation_radius_)));
    // Set to inscribed radius cost
    dilated_im.setTo(costmap_2d::INSCRIBED_INFLATED_OBSTACLE, dilated_im > costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

    // Additional dilation to use for a decaying cost with smooth connection to the inscribed radius
    // Dilate by cell_inflation_radius/2 so that the 0.5 Gaussian CDF lands here
    cv::Mat double_dilated;
    cv::dilate(
        dilated_im, double_dilated,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(cell_inflation_radius_ / 2, cell_inflation_radius_ / 2)));

    // Gaussian blur with sigma = cell_inflation_radius_/2/6.
    // Use 6 sigma for ~100% CDF and halve the radius as the centre is already half the radius away (double dilation)
    int cell_radius =
        (cell_inflation_radius_ / 2) ? ((cell_inflation_radius_ / 2) % 2) == 1 : (cell_inflation_radius_ / 2 + 1);
    cv::Mat blurred_map_;
    cv::GaussianBlur(double_dilated, blurred_map_, cv::Size(cell_radius, cell_radius), cell_inflation_radius_ / 12);

    // Make sure that everything sits between 0 and 252
    cv::Mat blurred_map;
    cv::normalize(blurred_map_, blurred_map, 0, costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1, cv::NORM_MINMAX, CV_8UC1);

    // Overlay the lethal and inscribed radius costs
    cv::Mat m1, m2;
    m1 = cv::max(blurred_map, dilated_im);
    m2 = cv::max(m1, cv_im);

    // Output
    for (unsigned int j = min_j; j < max_j; j++)
        for (unsigned int i = min_i; i < max_i; i++)
            master_grid.setCost(i, j, m2.at<uchar>(j, i));

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();


    std::ofstream duration_file;
    auto file_path = "/home/leng-vongchanh/duration_dilate_gauss.log";
    duration_file.open(file_path, std::ios_base::app);

    duration_file << duration << std::endl;
    duration_file.close();
}


// void DilationInflationLayer::computeCaches()
//{
//    if (cell_inflation_radius_ == 0)
//        return;
//
//    // based on the inflation radius... compute distance and cost caches
//    if (cell_inflation_radius_ != cached_cell_inflation_radius_)
//    {
//        deleteKernels();
//
//        cached_costs_ = new unsigned char*[cell_inflation_radius_ + 2];
//        cached_distances_ = new double*[cell_inflation_radius_ + 2];
//
//        for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
//        {
//            cached_costs_[i] = new unsigned char[cell_inflation_radius_ + 2];
//            cached_distances_[i] = new double[cell_inflation_radius_ + 2];
//            for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
//            {
//                cached_distances_[i][j] = hypot(i, j);
//            }
//        }
//
//        cached_cell_inflation_radius_ = cell_inflation_radius_;
//    }
//
//    for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
//    {
//        for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
//        {
//            cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
//        }
//    }
//}

void DilationInflationLayer::deleteKernels()
{
    if (cached_distances_ != nullptr)
    {
        for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
        {
            if (cached_distances_[i])
                delete[] cached_distances_[i];
        }
        if (cached_distances_)
            delete[] cached_distances_;
        cached_distances_ = nullptr;
    }

    if (cached_costs_ != nullptr)
    {
        for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
        {
            if (cached_costs_[i])
                delete[] cached_costs_[i];
        }
        delete[] cached_costs_;
        cached_costs_ = nullptr;
    }
}

void DilationInflationLayer::setInflationParameters(const double inflation_radius, const double cost_scaling_factor)
{
    if (weight_ != cost_scaling_factor || inflation_radius_ != inflation_radius)
    {
        // Lock here so that reconfiguring the inflation radius doesn't cause segfaults
        // when accessing the cached arrays
        boost::unique_lock<boost::recursive_mutex> lock(*inflation_access_);

        inflation_radius_ = inflation_radius;
        cell_inflation_radius_ = cellDistance(inflation_radius_);
        weight_ = cost_scaling_factor;
        need_reinflation_ = true;
        //        computeCaches();
    }
}

}  // namespace costmap_2d
