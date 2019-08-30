#ifndef GRIDMAP_RASTERIZE_H
#define GRIDMAP_RASTERIZE_H

#include <gridmap/operations/raytrace.h>

#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gridmap
{

struct Point2D
{
    int x, y;
};

inline int min3(int x, int y, int z)
{
    return x < y ? (x < z ? x : z) : (y < z ? y : z);
}

inline int max3(int x, int y, int z)
{
    return x > y ? (x > z ? x : z) : (y > z ? y : z);
}

inline int orient2d(const Point2D& a, const Point2D& b, const Point2D& c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

template <class ActionType> void drawTri(ActionType at, const Point2D& v0, const Point2D& v1, const Point2D& v2)
{
    // Compute triangle bounding box
    int minX = min3(v0.x, v1.x, v2.x);
    int minY = min3(v0.y, v1.y, v2.y);
    int maxX = max3(v0.x, v1.x, v2.x);
    int maxY = max3(v0.y, v1.y, v2.y);

    // Triangle setup
    int A01 = v0.y - v1.y, B01 = v1.x - v0.x;
    int A12 = v1.y - v2.y, B12 = v2.x - v1.x;
    int A20 = v2.y - v0.y, B20 = v0.x - v2.x;

    // Barycentric coordinates at minX/minY corner
    Point2D p = {minX, minY};
    int w0_row = orient2d(v1, v2, p);
    int w1_row = orient2d(v2, v0, p);
    int w2_row = orient2d(v0, v1, p);

    // Rasterize
    for (p.y = minY; p.y <= maxY; p.y++)
    {
        // Barycentric coordinates at start of row
        int w0 = w0_row;
        int w1 = w1_row;
        int w2 = w2_row;

        for (p.x = minX; p.x <= maxX; p.x++)
        {
            // If p is on or inside all edges, render pixel.
            if ((w0 | w1 | w2) >= 0)
                at(p.x, p.y, w0, w1, w2);

            // One step to the right
            w0 += A12;
            w1 += A20;
            w2 += A01;
        }

        // One row step
        w0_row += B12;
        w1_row += B20;
        w2_row += B01;
    }
}

template <class ActionType>
inline void rasterPolygonFill(ActionType at, const std::vector<Eigen::Array2i>& polygon, const int min_x,
                              const int max_x, const int min_y, const int max_y)
{
    std::vector<int> nodes_x(polygon.size() + 1);
    for (int cell_y = min_y; cell_y < max_y; ++cell_y)
    {
        int n_nodes = 0;
        std::size_t j = polygon.size() - 1;
        for (std::size_t i = 0; i < polygon.size(); i++)
        {
            if ((polygon[i].y() < cell_y && polygon[j].y() >= cell_y) ||
                (polygon[j].y() < cell_y && polygon[i].y() >= cell_y))
            {
                nodes_x[n_nodes++] = (static_cast<int>(polygon[i].x() + static_cast<double>(cell_y - polygon[i].y()) /
                                                                            (polygon[j].y() - polygon[i].y()) *
                                                                            (polygon[j].x() - polygon[i].x())));
            }
            j = i;
        }

        std::sort(nodes_x.begin(), nodes_x.begin() + n_nodes);

        for (int i = 0; i < n_nodes; i += 2)
        {
            if (nodes_x[i] >= max_x)
            {
                break;
            }
            if (nodes_x[i + 1] > min_x)
            {
                if (nodes_x[i] < min_x)
                    nodes_x[i] = min_x;
                if (nodes_x[i + 1] > max_x)
                    nodes_x[i + 1] = max_x;
                for (int cell_x = nodes_x[i]; cell_x < nodes_x[i + 1]; ++cell_x)
                    at(cell_x, cell_y);
            }
        }
    }
}

inline std::vector<Eigen::Array2i> connectPolygon(const std::vector<Eigen::Array2i>& polygon)
{
    std::vector<Eigen::Array2i> connected;
    for (int i = 0; i < static_cast<int>(polygon.size()) - 1; ++i)
    {
        std::vector<Eigen::Array2i> segment =
            drawLine(polygon[i].x(), polygon[i].y(), polygon[i + 1].x(), polygon[i + 1].y());
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
}  // namespace gridmap

#endif
