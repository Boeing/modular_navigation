#ifndef GRIDMAP_PROBABILITY_GRID_H
#define GRIDMAP_PROBABILITY_GRID_H

#include <Eigen/Geometry>

#include <gridmap/grids/grid_2d.h>

#include <cmath>
#include <mutex>
#include <vector>

#include "rcpputils/asserts.hpp"

namespace gridmap
{

inline double logodds(const double probability)
{
    return std::log(probability / (1 - probability));
}

inline double probability(const double logodds)
{
    return 1.0 - (1.0 / (1.0 + std::exp(logodds)));
}

class ProbabilityGrid : public Grid2D<double>
{
  public:
    explicit ProbabilityGrid(const MapDimensions& map_dims, const double clamping_thres_min = 0.1192,
                             const double clamping_thres_max = 0.971, const double occ_prob_thres = 0.8)
        : Grid2D<double>(map_dims)
    {
        rcpputils::assert_true(clamping_thres_min > 0.);
        rcpputils::assert_true(clamping_thres_min < 1.);

        rcpputils::assert_true(clamping_thres_max > 0.);
        rcpputils::assert_true(clamping_thres_max < 1.);

        rcpputils::assert_true(occ_prob_thres > 0.);
        rcpputils::assert_true(occ_prob_thres < 1.);

        clamping_thres_min_log_ = logodds(clamping_thres_min);
        clamping_thres_max_log_ = logodds(clamping_thres_max);
        occ_prob_thres_log_ = logodds(occ_prob_thres);
    }

    virtual ~ProbabilityGrid() = default;

    inline void update(const Eigen::Array2i& cell_index, const double log_odds)
    {
        const auto it = static_cast<std::size_t>(index(cell_index));
        cells_[it] = std::max(clamping_thres_min_log_, std::min(clamping_thres_max_log_, cells_[it] + log_odds));
    }

    inline bool occupied(const std::size_t& cell_index) const
    {
        return cells_[cell_index] >= occ_prob_thres_log_;
    }

    inline bool occupied(const Eigen::Array2i& cell_index) const
    {
        return cells_[static_cast<std::size_t>(index(cell_index))] >= occ_prob_thres_log_;
    }

    void setMinThres(const Eigen::Array2i& cell_index)
    {
        cells_[static_cast<std::size_t>(index(cell_index))] = clamping_thres_min_log_;
    }

    void setMaxThres(const Eigen::Array2i& cell_index)
    {
        cells_[static_cast<std::size_t>(index(cell_index))] = clamping_thres_min_log_;
    }

    void setOccThres(const Eigen::Array2i& cell_index)
    {
        cells_[static_cast<std::size_t>(index(cell_index))] = occ_prob_thres_log_;
    }

    double ocupancyThres() const
    {
        return probability(occ_prob_thres_log_);
    }

    double occupancyThresLog() const
    {
        return occ_prob_thres_log_;
    }

    void setOccupancyThres(const double prob)
    {
        occ_prob_thres_log_ = logodds(prob);
    }

    void setClampingThresMin(const double prob)
    {
        clamping_thres_min_log_ = logodds(prob);
    }

    double clampingThresMin() const
    {
        return probability(clamping_thres_min_log_);
    }

    double clampingThresMinLog() const
    {
        return clamping_thres_min_log_;
    }

    void setClampingThresMax(const double prob)
    {
        clamping_thres_max_log_ = logodds(prob);
    }

    double clampingThresMax() const
    {
        return probability(clamping_thres_max_log_);
    }

    double clampingThresMaxLog() const
    {
        return clamping_thres_max_log_;
    }

  protected:
    double clamping_thres_min_log_;
    double clamping_thres_max_log_;
    double occ_prob_thres_log_;
};
}  // namespace gridmap

#endif
