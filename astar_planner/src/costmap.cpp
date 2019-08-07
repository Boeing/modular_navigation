#include <astar_planner/costmap.h>
#include <opencv2/imgproc.hpp>

namespace astar_planner
{

std::shared_ptr<Costmap> buildCostmap(const gridmap::MapData& map_data, const double robot_radius)
{
    auto grid = std::make_shared<Costmap>();

    cv::Mat dilated;
    {
        {
            auto lock = map_data.grid.getLock();

            grid->width = map_data.grid.dimensions().size().x();
            grid->height = map_data.grid.dimensions().size().y();

            grid->resolution = map_data.grid.dimensions().resolution();

            const int size_x = map_data.grid.dimensions().size().x();
            const int size_y = map_data.grid.dimensions().size().y();

            grid->origin_x = map_data.grid.dimensions().origin().x();
            grid->origin_y = map_data.grid.dimensions().origin().y();

            const cv::Mat raw(size_y, size_x, CV_8U, reinterpret_cast<void*>(const_cast<uint8_t*>(map_data.grid.cells().data())));
            grid->obstacle_map = raw.clone();
        }

        // dilate robot radius
        const int cell_inflation_radius = static_cast<int>(2.0 * robot_radius / grid->resolution);
        auto ellipse = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(cell_inflation_radius, cell_inflation_radius));
        cv::dilate(grid->obstacle_map, dilated, ellipse);
    }

    // flip
    cv::bitwise_not(dilated, dilated);

    // allocate
    grid->distance_to_collision = cv::Mat(dilated.size(), CV_32F);

    // find obstacle distances
    cv::distanceTransform(dilated, grid->distance_to_collision, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);

    return grid;
}

}
