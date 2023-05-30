#ifndef ASTAR_PLANNER_VISUALISATION_H
#define ASTAR_PLANNER_VISUALISATION_H

#include <Eigen/Geometry>

#include <astar_planner/astar.h>
#include <astar_planner/costmap.h>

namespace astar_planner {

cv::Mat visualise(const Costmap &costmap, const PathResult &astar_result);
cv::Mat visualise(cv::Mat &disp, const Costmap &costmap,
                  const PathResult &astar_result);

void drawDot(const Costmap &costmap, const PathResult &astar_result,
             const Eigen::Isometry2d &goal, const std::string &graph_path,
             const double linear_resolution, const double angular_resolution);

bool drawPathSVG(const PathResult &astar_result, const std::string &svg_path);

} // namespace astar_planner

#endif
