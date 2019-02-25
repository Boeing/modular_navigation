#include <costmap_2d/footprint.h>
#include <costmap_2d/layered_costmap.h>

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

using std::vector;

namespace costmap_2d
{

LayeredCostmap::LayeredCostmap(std::string global_frame, bool rolling_window, bool track_unknown)
    : costmap_(), global_frame_(global_frame), rolling_window_(rolling_window), current_(false), bx0_(0), bxn_(0),
      by0_(0), byn_(0), initialized_(false), size_locked_(false), circumscribed_radius_(1.0), inscribed_radius_(0.1)
{
    if (track_unknown)
        costmap_.setDefaultValue(255);
    else
        costmap_.setDefaultValue(0);
}

LayeredCostmap::~LayeredCostmap()
{
}

void LayeredCostmap::resizeMap(unsigned int size_x, unsigned int size_y, double resolution, double origin_x,
                               double origin_y, bool size_locked)
{
    boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));
    size_locked_ = size_locked;
    costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
    for (vector<boost::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin)
    {
        (*plugin)->matchSize();
    }
}

void LayeredCostmap::updateMap(double robot_x, double robot_y, double robot_yaw)
{
    boost::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));

    if (rolling_window_)
    {
        const double new_origin_x = robot_x - costmap_.getSizeInMetersX() / 2;
        const double new_origin_y = robot_y - costmap_.getSizeInMetersY() / 2;
        costmap_.updateOrigin(new_origin_x, new_origin_y);
    }

    if (plugins_.size() == 0)
        return;

    if (costmap_.getSizeInCellsX() == 0 || costmap_.getSizeInCellsY() == 0)
    {
        ROS_WARN_STREAM("0 sized map detected, skipping update. x_size: " << costmap_.getSizeInCellsX() << ", y_size: "
                                                                          << costmap_.getSizeInCellsY());
        return;
    }

    double minx = std::numeric_limits<double>::max();
    double miny = std::numeric_limits<double>::max();
    double maxx = -std::numeric_limits<double>::max();
    double maxy = -std::numeric_limits<double>::max();

    for (vector<boost::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin)
    {
        double prev_minx = minx;
        double prev_miny = miny;
        double prev_maxx = maxx;
        double prev_maxy = maxy;

        (*plugin)->updateBounds(robot_x, robot_y, robot_yaw, &minx, &miny, &maxx, &maxy);

        if (minx > prev_minx || miny > prev_miny || maxx < prev_maxx || maxy < prev_maxy)
        {
            ROS_WARN_THROTTLE(1.0,
                              "Illegal bounds change, was [tl: (%f, %f), br: (%f, %f)], but "
                              "is now [tl: (%f, %f), br: (%f, %f)]. The offending layer is %s",
                              prev_minx, prev_miny, prev_maxx, prev_maxy, minx, miny, maxx, maxy,
                              (*plugin)->getName().c_str());
        }
    }

    int x0;
    int xn;
    int y0;
    int yn;
    costmap_.worldToMapNoBounds(minx, miny, x0, y0);
    costmap_.worldToMapNoBounds(maxx, maxy, xn, yn);

    if (xn < x0 || yn < y0)
        return;

    const unsigned int u_x0 = static_cast<unsigned int>(std::max(0, std::min(x0, int(costmap_.getSizeInCellsX()) - 1)));
    const unsigned int u_xn = static_cast<unsigned int>(std::min(int(costmap_.getSizeInCellsX()), std::max(xn, 0) + 1));
    const unsigned int u_y0 = static_cast<unsigned int>(std::max(0, std::min(y0, int(costmap_.getSizeInCellsY()) - 1)));
    const unsigned int u_yn = static_cast<unsigned int>(std::min(int(costmap_.getSizeInCellsY()), std::max(yn, 0) + 1));

    ROS_DEBUG_STREAM("Updating area " << u_x0 << " => " << u_xn << " " << u_y0 << " -> " << u_yn);

    // Reset only the section to be updated
    costmap_.resetMap(u_x0, u_y0, u_xn, u_yn);

    for (vector<boost::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin)
    {
        (*plugin)->updateCosts(costmap_, u_x0, u_y0, u_xn, u_yn);
    }

    bx0_ = u_x0;
    bxn_ = u_xn;
    by0_ = u_y0;
    byn_ = u_yn;

    initialized_ = true;
}

bool LayeredCostmap::isCurrent()
{
    current_ = true;
    for (vector<boost::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin)
    {
        current_ = current_ && (*plugin)->isCurrent();
    }
    return current_;
}

void LayeredCostmap::setFootprint(const std::vector<geometry_msgs::Point>& footprint_spec)
{
    footprint_ = footprint_spec;
    costmap_2d::calculateMinAndMaxDistances(footprint_spec, inscribed_radius_, circumscribed_radius_);

    for (vector<boost::shared_ptr<Layer>>::iterator plugin = plugins_.begin(); plugin != plugins_.end(); ++plugin)
    {
        (*plugin)->onFootprintChanged();
    }
}

}  // namespace costmap_2d
