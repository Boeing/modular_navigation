#include <cmath>
#include <cstdio>
#include <gridmap/map_data.h>

namespace gridmap
{

MapData::MapData(const double clamping_thres_min, const double clamping_thres_max, const double occ_prob_thres)
    : size_x_(0), size_y_(0), resolution_(0.02), origin_x_(0), origin_y_(0),
      clamping_thres_min_log_(logodds(clamping_thres_min)), clamping_thres_max_log_(logodds(clamping_thres_max)),
      occ_prob_thres_log_(logodds(occ_prob_thres))
{
}

MapData::~MapData()
{
}

void MapData::resize(const unsigned int size_x, const unsigned int size_y, const double resolution,
                     const double origin_x, const double origin_y)
{
    size_x_ = size_x;
    size_y_ = size_y;
    resolution_ = resolution;
    origin_x_ = origin_x;
    origin_y_ = origin_y;
    costmap_ = std::vector<double>(size_x_ * size_y_, 0.0);
}

unsigned int MapData::distance(double world_dist) const
{
    return static_cast<unsigned int>(std::max(0.0, std::ceil(world_dist / resolution_)));
}

std::unique_lock<std::recursive_mutex> MapData::getLock() const
{
    return std::unique_lock<std::recursive_mutex>(mutex_);
}

double* MapData::data()
{
    return costmap_.data();
}

const double* MapData::data() const
{
    return costmap_.data();
}

double MapData::value(const unsigned int mx, const unsigned int my) const
{
    return costmap_[index(mx, my)];
}

void MapData::update(const unsigned int index, const double value)
{
    costmap_[index] = std::max(clamping_thres_min_log_, std::min(clamping_thres_max_log_, costmap_[index] + value));
}

void MapData::update(const unsigned int mx, const unsigned int my, const double value)
{
    const auto it = index(mx, my);
    costmap_[it] = std::max(clamping_thres_min_log_, std::min(clamping_thres_max_log_, costmap_[it] + value));
}

void MapData::set(const unsigned int mx, const unsigned int my, const double value)
{
    costmap_[index(mx, my)] = value;
}

void MapData::setMinThres(const unsigned int mx, const unsigned int my)
{
    costmap_[index(mx, my)] = clamping_thres_min_log_;
}

void MapData::setMaxThres(const unsigned int mx, const unsigned int my)
{
    costmap_[index(mx, my)] = clamping_thres_max_log_;
}

void MapData::mapToWorld(const unsigned int mx, const unsigned int my, double& wx, double& wy) const
{
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
}

bool MapData::worldToMap(const double wx, const double wy, unsigned int& mx, unsigned int& my) const
{
    if (wx < origin_x_ || wy < origin_y_)
        return false;

    mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
    my = static_cast<unsigned int>((wy - origin_y_) / resolution_);

    if (mx < size_x_ && my < size_y_)
        return true;

    return false;
}

Eigen::Vector2i MapData::worldToMapNoBounds(const Eigen::Vector2d& world) const
{
    return Eigen::Vector2i(static_cast<int>((world.x() - origin_x_) / resolution_),
                           static_cast<int>((world.y() - origin_y_) / resolution_));
}

void MapData::worldToMapEnforceBounds(const double wx, const double wy, int& mx, int& my) const
{
    if (wx < origin_x_)
    {
        mx = 0;
    }
    else if (wx > resolution_ * size_x_ + origin_x_)
    {
        mx = static_cast<int>(size_x_) - 1;
    }
    else
    {
        mx = static_cast<int>((wx - origin_x_) / resolution_);
    }

    if (wy < origin_y_)
    {
        my = 0;
    }
    else if (wy > resolution_ * size_y_ + origin_y_)
    {
        my = static_cast<int>(size_y_) - 1;
    }
    else
    {
        my = static_cast<int>((wy - origin_y_) / resolution_);
    }
}
}
