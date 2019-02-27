#include <algorithm>
#include <boost/thread.hpp>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/footprint.h>
#include <costmap_2d/plugins/inflation_layer.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::InflationLayer, costmap_2d::Layer)

namespace costmap_2d
{

InflationLayer::InflationLayer()
    : resolution_(1), inflation_radius_(0), inscribed_radius_(0), weight_(0), inflate_unknown_(false),
      cell_inflation_radius_(0), cached_cell_inflation_radius_(0), inflation_cells_({}), dsrv_(nullptr), seen_(nullptr),
      seen_size_(0), cached_costs_(nullptr), cached_distances_(nullptr),
      last_min_x_(-std::numeric_limits<float>::max()), last_min_y_(-std::numeric_limits<float>::max()),
      last_max_x_(std::numeric_limits<float>::max()), last_max_y_(std::numeric_limits<float>::max()),
      need_reinflation_(false)
{
    inflation_access_ = new boost::recursive_mutex();
}

void InflationLayer::onInitialize()
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

        dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>::CallbackType cb =
            boost::bind(&InflationLayer::reconfigureCB, this, _1, _2);

        if (dsrv_ != nullptr)
        {
            dsrv_->clearCallback();
            dsrv_->setCallback(cb);
        }
        else
        {
            dsrv_ = new dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>(ros::NodeHandle("~/" + name_));
            dsrv_->setCallback(cb);
        }
    }

    matchSize();
}

void InflationLayer::reconfigureCB(costmap_2d::InflationPluginConfig& config, uint32_t)
{
    setInflationParameters(config.inflation_radius, config.cost_scaling_factor);

    if (enabled_ != config.enabled || inflate_unknown_ != config.inflate_unknown)
    {
        enabled_ = config.enabled;
        inflate_unknown_ = config.inflate_unknown;
        need_reinflation_ = true;
    }
}

void InflationLayer::matchSize()
{
    boost::unique_lock<boost::recursive_mutex> lock(*inflation_access_);
    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    resolution_ = costmap->getResolution();
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    computeCaches();

    const unsigned int size_x = costmap->getSizeInCellsX();
    const unsigned int size_y = costmap->getSizeInCellsY();
    if (seen_)
        delete[] seen_;
    seen_size_ = size_x * size_y;
    seen_ = new bool[seen_size_];
}

void InflationLayer::updateBounds(const double, const double, const double, double* min_x, double* min_y, double* max_x,
                                  double* max_y)
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

void InflationLayer::onFootprintChanged()
{
    inscribed_radius_ = layered_costmap_->getInscribedRadius();
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    computeCaches();
    need_reinflation_ = true;

    ROS_DEBUG("InflationLayer::onFootprintChanged(): num footprint points: %lu,"
              " inscribed_radius_ = %.3f, inflation_radius_ = %.3f",
              layered_costmap_->getFootprint().size(), inscribed_radius_, inflation_radius_);
}

void InflationLayer::updateCosts(Costmap2D& master_grid, unsigned int min_i, unsigned int min_j, unsigned int max_i,
                                 unsigned int max_j)
{
    boost::unique_lock<boost::recursive_mutex> lock(*inflation_access_);
    if (!enabled_ || (cell_inflation_radius_ == 0))
        return;

    // make sure the inflation list is empty at the beginning of the cycle (should always be true)
    ROS_ASSERT_MSG(inflation_cells_.empty(), "The inflation list must be empty at the beginning of inflation");

    unsigned char* master_array = master_grid.getCharMap();
    const unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

    if (seen_ == nullptr)
    {
        ROS_WARN("InflationLayer::updateCosts(): seen_ array is NULL");
        seen_size_ = size_x * size_y;
        seen_ = new bool[seen_size_];
    }
    else if (seen_size_ != size_x * size_y)
    {
        ROS_WARN("InflationLayer::updateCosts(): seen_ array size is wrong");
        delete[] seen_;
        seen_size_ = size_x * size_y;
        seen_ = new bool[seen_size_];
    }
    memset(seen_, false, size_x * size_y * sizeof(bool));

    // We need to include in the inflation cells outside the bounding
    // box min_i...max_j, by the amount cell_inflation_radius_.  Cells
    // up to that distance outside the box can still influence the costs
    // stored in cells inside the box.
    min_i = u_int(std::max<int>(0, int(min_i) - cell_inflation_radius_));
    min_j = u_int(std::max<int>(0, int(min_j) - cell_inflation_radius_));
    max_i = u_int(std::min<int>(int(size_x), int(max_i) + cell_inflation_radius_));
    max_j = u_int(std::min<int>(int(size_y), int(max_j) + cell_inflation_radius_));

    // Inflation list; we append cells to visit in a list associated with its distance to the nearest obstacle
    // We use a map<distance, list> to emulate the priority queue used before, with a notable performance boost

    // Start with lethal obstacles: by definition distance is 0.0
    std::vector<CellData>& obs_bin = inflation_cells_[0.0];
    for (unsigned int j = min_j; j < max_j; j++)
    {
        for (unsigned int i = min_i; i < max_i; i++)
        {
            int index = master_grid.getIndex(i, j);
            unsigned char cost = master_array[index];
            if (cost == costmap_2d::LETHAL_OBSTACLE)
            {
                obs_bin.push_back(CellData(index, i, j, i, j));
            }
        }
    }

    // Process cells by increasing distance; new cells are appended to the corresponding distance bin, so they
    // can overtake previously inserted but farther away cells
    std::map<double, std::vector<CellData>>::iterator bin;
    for (bin = inflation_cells_.begin(); bin != inflation_cells_.end(); ++bin)
    {
        for (unsigned int i = 0; i < bin->second.size(); ++i)
        {
            // process all cells at distance dist_bin.first
            const CellData& cell = bin->second[i];

            const unsigned int index = cell.index_;

            // ignore if already visited
            if (seen_[index])
            {
                continue;
            }

            seen_[index] = true;

            unsigned int mx = cell.x_;
            unsigned int my = cell.y_;
            unsigned int sx = cell.src_x_;
            unsigned int sy = cell.src_y_;

            // assign the cost associated with the distance from an obstacle to the cell
            const unsigned char cost = costLookup(mx, my, sx, sy);
            const unsigned char old_cost = master_array[index];
            if (old_cost == costmap_2d::NO_INFORMATION &&
                (inflate_unknown_ ? (cost > costmap_2d::FREE_SPACE)
                                  : (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)))
                master_array[index] = cost;
            else
                master_array[index] = std::max(old_cost, cost);

            // attempt to put the neighbors of the current cell onto the inflation list
            if (mx > 0)
                enqueue(index - 1, mx - 1, my, sx, sy);
            if (my > 0)
                enqueue(index - size_x, mx, my - 1, sx, sy);
            if (mx < size_x - 1)
                enqueue(index + 1, mx + 1, my, sx, sy);
            if (my < size_y - 1)
                enqueue(index + size_x, mx, my + 1, sx, sy);
        }
    }

    inflation_cells_.clear();
}

/**
 * @brief  Given an index of a cell in the costmap, place it into a list pending for obstacle inflation
 * @param  grid The costmap
 * @param  index The index of the cell
 * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
 * @param  src_x The x index of the obstacle point inflation started at
 * @param  src_y The y index of the obstacle point inflation started at
 */
inline void InflationLayer::enqueue(const unsigned int index, const unsigned int mx, const unsigned int my,
                                    const unsigned int src_x, const unsigned int src_y)
{
    if (!seen_[index])
    {
        // we compute our distance table one cell further than the inflation radius dictates so we can make the check
        // below
        double distance = distanceLookup(mx, my, src_x, src_y);

        // we only want to put the cell in the list if it is within the inflation radius of the obstacle point
        if (distance > cell_inflation_radius_)
            return;

        // push the cell data onto the inflation list and mark
        inflation_cells_[distance].push_back(CellData(index, mx, my, src_x, src_y));
    }
}

void InflationLayer::computeCaches()
{
    if (cell_inflation_radius_ == 0)
        return;

    // based on the inflation radius... compute distance and cost caches
    if (cell_inflation_radius_ != cached_cell_inflation_radius_)
    {
        deleteKernels();

        cached_costs_ = new unsigned char*[cell_inflation_radius_ + 2];
        cached_distances_ = new double*[cell_inflation_radius_ + 2];

        for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
        {
            cached_costs_[i] = new unsigned char[cell_inflation_radius_ + 2];
            cached_distances_[i] = new double[cell_inflation_radius_ + 2];
            for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
            {
                cached_distances_[i][j] = hypot(i, j);
            }
        }

        cached_cell_inflation_radius_ = cell_inflation_radius_;
    }

    for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
    {
        for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
        {
            cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
        }
    }
}

void InflationLayer::deleteKernels()
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

void InflationLayer::setInflationParameters(const double inflation_radius, const double cost_scaling_factor)
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
        computeCaches();
    }
}

}  // namespace costmap_2d
