#ifndef SIM_BAND_PLANNER_MOVING_WINDOW_H
#define SIM_BAND_PLANNER_MOVING_WINDOW_H

#include <Eigen/Geometry>

#include <vector>

#include <sim_band_planner/band.h>

#include <navigation_interface/types/path.h>

namespace sim_band_planner
{

struct MovingWindow
{
    const navigation_interface::Path nominal_path;

    std::size_t end_i;
    Band window;

    MovingWindow(const navigation_interface::Path& _path) : nominal_path(_path), end_i(0)
    {
    }

    void updateWindow(const Eigen::Isometry2d& pose, const double max_length)
    {
        if (!window.nodes.empty())
        {
            // trim the window
            const std::pair<std::size_t, double> closest = window.closestSegment(pose);
            window.nodes.erase(window.nodes.begin(), window.nodes.begin() + closest.first);
        }
        else
        {
            // start appending at the closest robot pose
            auto closest = nominal_path.closestSegment(pose);
            end_i = closest.first;
        }

        double distance = window.length();
        const std::size_t start_append = end_i;
        std::size_t end_append = end_i;
        while (distance < max_length && end_append != nominal_path.nodes.size())
        {
            if (end_append != 0)
                distance += ((nominal_path.nodes[end_append - 1]).translation() -
                             (nominal_path.nodes[end_append]).translation())
                                .norm();
            ++end_append;
        }

        end_i = end_append;

        for (std::size_t i = start_append; i < end_append; ++i)
        {
            window.nodes.push_back(Node(nominal_path.nodes[i]));
        }
    }
};
}

#endif
