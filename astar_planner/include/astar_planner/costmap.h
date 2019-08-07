#ifndef ASTAR_PLANNER_COSTMAP_H
#define ASTAR_PLANNER_COSTMAP_H

#include <opencv2/core.hpp>

#include <astar_planner/node.h>

#include <gridmap/map_data.h>

#include <memory>

#include <Eigen/Geometry>

namespace astar_planner
{

struct Costmap
{
    cv::Mat obstacle_map;
    cv::Mat distance_to_collision;

    int width;
    int height;

    double resolution;
    double origin_x;
    double origin_y;

    float inflation_radius;

    inline std::size_t to2DGridIndex(const State2D& state) const
    {
        return static_cast<std::size_t>(width * state.y + state.x);
    }
};

std::shared_ptr<Costmap> buildCostmap(const gridmap::MapData& map_data, const double robot_radius);

class CollisionChecker
 {
 public:
     explicit CollisionChecker(const std::shared_ptr<const Costmap>& costmap) :
         costmap_(costmap),
         offsets_({
                  {-0.268, 0.000},
                  { 0.268, 0.000},
                  { 0.265,-0.185},
                  { 0.077,-0.185},
                  {-0.077,-0.185},
                  {-0.265,-0.185},
                  { 0.265, 0.185},
                  {-0.265, 0.185},
                  {-0.077, 0.185},
                  { 0.077, 0.185}
                  })
     {
     }
     virtual ~CollisionChecker() = default;

     bool isWithinBounds(const State3D& state) const
     {
         const int mx = static_cast<int>((state.x - costmap_->origin_x) / costmap_->resolution - 0.5);
         const int my = static_cast<int>((state.y - costmap_->origin_y) / costmap_->resolution - 0.5);
         return (mx >=0 && mx < costmap_->width && my >=0 && my < costmap_->height);
     }

     bool isValid(const State3D& state) const
     {
         return clearance(state) > 0.0;
     }

     double clearance(const State3D& state) const
     {
         double min_distance = std::numeric_limits<double>::max();
         for (const auto& offset : offsets_)
         {
             const Eigen::Vector2d offset_xy = Eigen::Vector2d(state.x, state.y) + Eigen::Vector2d(Eigen::Rotation2Dd(state.theta) * offset);

             const int mx = static_cast<int>(std::round((offset_xy.x() - costmap_->origin_x) / costmap_->resolution));
             const int my = static_cast<int>(std::round((offset_xy.y() - costmap_->origin_y) / costmap_->resolution));

             double distance = 0;
             if (mx >=0 && mx < costmap_->distance_to_collision.cols
                     && my >=0 && my < costmap_->distance_to_collision.rows)
             {
                 distance = static_cast<double>(costmap_->distance_to_collision.at<float>(my, mx));
             }
             min_distance = std::min(min_distance, distance);
         }
         return min_distance;
     }

     bool isWithinBounds(const State2D& state) const
     {
         return (state.x >=0 && state.x < costmap_->width && state.y >=0 && state.y < costmap_->height);
     }

     bool isValid(const State2D& state) const
     {
         // TODO simple radius check from centre of robot
         return clearance(state) > 0.0;
     }

     double clearance(const State2D& state) const
     {
         if (isWithinBounds(state))
         {
             return static_cast<double>(costmap_->distance_to_collision.at<float>(state.y, state.x));
         }
         return 0;
     }

  private:
     const std::shared_ptr<const Costmap> costmap_;
     const std::vector<Eigen::Vector2d> offsets_;
 };

}

#endif
