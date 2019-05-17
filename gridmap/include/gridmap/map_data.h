#ifndef GRIDMAP_MAP_DATA_H
#define GRIDMAP_MAP_DATA_H

#include <cmath>
#include <mutex>
#include <vector>

#include <Eigen/Geometry>

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

class MapData
{
  public:
    MapData(const double clamping_thres_min, const double clamping_thres_max, const double occ_prob_thres);

    virtual ~MapData();

    void resize(const unsigned int size_x, const unsigned int size_y, const double resolution, const double origin_x,
                const double origin_y);

    unsigned int distance(double world_dist) const;

    std::unique_lock<std::recursive_mutex> getLock() const;

    double* data();
    const double* data() const;

    double value(const unsigned int mx, const unsigned int my) const;

    void update(const unsigned int index, const double value);
    void update(const unsigned int mx, const unsigned int my, const double value);

    void set(const unsigned int mx, const unsigned int my, const double value);
    void setMinThres(const unsigned int mx, const unsigned int my);
    void setMaxThres(const unsigned int mx, const unsigned int my);

    void mapToWorld(const unsigned int mx, const unsigned int my, double& wx, double& wy) const;

    bool worldToMap(const double wx, const double wy, unsigned int& mx, unsigned int& my) const;

    Eigen::Vector2i worldToMapNoBounds(const Eigen::Vector2d& world) const;

    void worldToMapEnforceBounds(const double wx, const double wy, int& mx, int& my) const;

    Eigen::Vector2i size() const
    {
        return Eigen::Vector2i(size_x_, size_y_);
    }

    unsigned int sizeX() const
    {
        return size_x_;
    };
    unsigned int sizeY() const
    {
        return size_y_;
    };

    double originX() const
    {
        return origin_x_;
    };
    double originY() const
    {
        return origin_y_;
    };

    double resolution() const
    {
        return resolution_;
    };

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

    inline unsigned int index(unsigned int mx, unsigned int my) const
    {
        return my * size_x_ + mx;
    }

    inline std::pair<unsigned int, unsigned int> cell(unsigned int index) const
    {
        unsigned int my = index / size_x_;
        return {index - (my * size_x_), my};
    }

  private:
    mutable std::recursive_mutex mutex_;

  protected:
    unsigned int size_x_;
    unsigned int size_y_;

    double resolution_;
    double origin_x_;
    double origin_y_;

    double clamping_thres_min_log_;
    double clamping_thres_max_log_;
    double occ_prob_thres_log_;

    std::vector<double> costmap_;
};
}

#endif
