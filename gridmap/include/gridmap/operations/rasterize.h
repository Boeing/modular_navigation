#ifndef GRIDMAP_RASTERIZE_H
#define GRIDMAP_RASTERIZE_H

#include <gridmap/operations/raytrace.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gridmap
{

std::vector<Eigen::Array2i> rasterPolygonFill(const std::vector<Eigen::Array2i>& polygon, const int min_y,
                                              const int max_y)
{
    std::vector<Eigen::Array2i> cells;
    for (int cell_y = min_y; cell_y <= max_y; ++cell_y)
    {
        std::vector<int> nodes_x;
        std::size_t j = polygon.size() - 1;
        for (std::size_t i = 0; i < polygon.size(); i++)
        {
            if ((polygon[i].y() < static_cast<double>(cell_y) && polygon[j].y() >= static_cast<double>(cell_y)) ||
                (polygon[j].y() < static_cast<double>(cell_y) && polygon[i].y() >= static_cast<double>(cell_y)))
            {
                nodes_x.push_back(static_cast<int>(polygon[i].x() + (cell_y - polygon[i].y()) /
                                                                        (polygon[j].y() - polygon[i].y()) *
                                                                        (polygon[j].x() - polygon[i].x())));
            }
            j = i;
        }

        std::sort(nodes_x.begin(), nodes_x.end());

        for (std::size_t i = 0; i < nodes_x.size(); i += 2)
        {
            for (int cell_x = nodes_x[i]; cell_x < nodes_x[i + 1]; ++cell_x)
                cells.push_back({cell_x, cell_y});
        }
    }
    return cells;
}

std::vector<Eigen::Array2i> connectPolygon(const std::vector<Eigen::Array2i>& polygon)
{
    std::vector<Eigen::Array2i> connected;
    for (int i = 0; i < static_cast<int>(polygon.size()) - 1; ++i)
    {
        std::vector<Eigen::Array2i> segment = drawLine(polygon[i], polygon[i + 1]);
        if (!segment.empty())
        {
            if (!connected.empty() && (segment.front() == connected.back()).all())
            {
                if (segment.size() > 1)
                    connected.insert(connected.end(), segment.begin() + 1, segment.end());
            }
            else
            {
                connected.insert(connected.end(), segment.begin(), segment.end());
            }
        }
    }
    return connected;
}

}

#endif
