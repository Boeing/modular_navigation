#ifndef ASTAR_PLANNER_COSTMAP_H
#define ASTAR_PLANNER_COSTMAP_H

#include <Eigen/Geometry>

#include <astar_planner/node.h>
#include <gridmap/map_data.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <memory>

namespace astar_planner
{

struct Costmap
{
    // uint8 map of obstacles
    cv::Mat obstacle_map;

    // float map of traversal cost scale (1.f default)
    std::shared_ptr<cv::Mat> traversal_cost;

    // float map of distance to nearest obstacle (in pixels)
    cv::Mat distance_to_collision;

    int width;
    int height;

    double resolution;
    double origin_x;
    double origin_y;

    double inflation_radius;

    Costmap(const gridmap::MapData& map_data, const double robot_radius)
    {
        inflation_radius = robot_radius;

        width = map_data.grid.dimensions().size().x();
        height = map_data.grid.dimensions().size().y();

        resolution = map_data.grid.dimensions().resolution();

        origin_x = map_data.grid.dimensions().origin().x();
        origin_y = map_data.grid.dimensions().origin().y();

        obstacle_map = obstacle_map;

        const int size_x = map_data.grid.dimensions().size().x();
        const int size_y = map_data.grid.dimensions().size().y();
        const cv::Mat raw(size_y, size_x, CV_8U,
                          reinterpret_cast<void*>(const_cast<uint8_t*>(map_data.grid.cells().data())));
        obstacle_map = raw.clone();
    }

    void processObstacleMap()
    {
        cv::Mat dilated;
        {
            // dilate robot radius
            const int cell_inflation_radius = static_cast<int>(2.0 * inflation_radius / resolution);
            auto ellipse =
                cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(cell_inflation_radius, cell_inflation_radius));
            cv::dilate(obstacle_map, dilated, ellipse);
        }

        // flip because distanceTransform finds the distance to the nearest ZERO pixel
        cv::bitwise_not(dilated, dilated);

        // allocate
        distance_to_collision = cv::Mat(dilated.size(), CV_32F);

        // find obstacle distances
        cv::distanceTransform(dilated, distance_to_collision, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);
    }

    void processObstacleMap(const gridmap::AABB& local_region)
    {
        cv::Rect roi(local_region.roi_start.x(), local_region.roi_start.y(), local_region.roi_size.x(),
                     local_region.roi_size.y());

        cv::Mat obstacle_map_roi = obstacle_map(roi);

        cv::Mat dilated(obstacle_map.size(), CV_8U, 255);
        cv::Mat dilated_roi = dilated(roi);
        {
            // dilate robot radius
            const int cell_inflation_radius = static_cast<int>(2.0 * inflation_radius / resolution);
            auto ellipse =
                cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(cell_inflation_radius, cell_inflation_radius));

            cv::dilate(obstacle_map_roi, dilated_roi, ellipse);
        }

        // flip because distanceTransform finds the distance to the nearest ZERO pixel
        cv::bitwise_not(dilated_roi, dilated_roi);

        // allocate. Set distance to 0.0 by default.
        distance_to_collision = cv::Mat(dilated.size(), CV_32F, 0.0f);
        cv::Mat distance_to_collision_roi = distance_to_collision(roi);

        // find obstacle distances
        cv::distanceTransform(dilated_roi, distance_to_collision_roi, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);
    }

    inline Eigen::Array2i getCellIndex(const Eigen::Vector2d& point) const
    {
        return Eigen::Array2i(std::round((point.x() - origin_x) / resolution),
                              std::round((point.y() - origin_y) / resolution));
    }

    inline std::size_t to2DGridIndex(const State2D& state) const
    {
        return static_cast<std::size_t>(width * state.y + state.x);
    }
};

class CollisionChecker
{
  public:
    CollisionChecker(const Costmap& costmap, const std::vector<Eigen::Vector2d>& offsets,
                     const double conservative_radius)
        : costmap_(costmap), offsets_(offsets), conservative_radius_(conservative_radius)
    {
        ROS_ASSERT(conservative_radius >= costmap.inflation_radius);
        // check that the conservative radius is larger than the width of the robot
        for (const auto& offset : offsets_)
            ROS_ASSERT_MSG(conservative_radius >= std::abs(offset.y()) + costmap.inflation_radius,
                           "conservative_radius: %f offset.y(): %f inflation_radius: %f", conservative_radius,
                           offset.y(), costmap.inflation_radius);

        // build collision cost lookup table
        collision_cost_lut_ = std::vector<double>(lut_size_, 0);
        for (int i = 0; i < lut_size_; ++i)
        {
            const double distance_to_collision_m = i * lut_step_size_;
            collision_cost_lut_[i] = std::min(10.0, std::max(1.0, lut_max_dist_ / (distance_to_collision_m)));
        }
    }

    double collisionCost(const int distance_to_collision_px) const
    {
        if (distance_to_collision_px < lut_size_)
        {
            return collision_cost_lut_[distance_to_collision_px];
        }
        else
        {
            return 1.0;
        }
    }

    bool isWithinBounds(const State3D& state) const
    {
        const Eigen::Array2i map_cell = costmap_.getCellIndex({state.x, state.y});
        return (map_cell.x() >= 0 && map_cell.x() < costmap_.width && map_cell.y() >= 0 &&
                map_cell.y() < costmap_.height);
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
            const double st = std::sin(state.theta);
            const double ct = std::cos(state.theta);
            const double offset_x = state.x + (offset.x() * ct - offset.y() * st);
            const double offset_y = state.y + (offset.x() * st + offset.y() * ct);

            const Eigen::Array2i map_cell = costmap_.getCellIndex({offset_x, offset_y});

            double d = 0;
            if (map_cell.x() >= 0 && map_cell.x() < costmap_.distance_to_collision.cols && map_cell.y() >= 0 &&
                map_cell.y() < costmap_.distance_to_collision.rows)
            {
                d = static_cast<double>(costmap_.distance_to_collision.at<float>(map_cell.y(), map_cell.x()));
            }
            min_distance = std::min(min_distance, d);
        }
        return min_distance;
    }

    bool isWithinBounds(const State2D& state) const
    {
        return (state.x >= 0 && state.x < costmap_.width && state.y >= 0 && state.y < costmap_.height);
    }

    bool isValid(const State2D& state) const
    {
        return clearance(state) > 0.0;
    }

    double clearance(const State2D& state) const
    {
        // assumes the state is valid!!
        return static_cast<double>(costmap_.distance_to_collision.at<float>(state.y, state.x));
    }

    const std::vector<Eigen::Vector2d> offsets() const
    {
        return offsets_;
    }

    const Costmap& costmap() const
    {
        return costmap_;
    }

    double conservativeRadius() const
    {
        return conservative_radius_;
    }

  private:
    const Costmap& costmap_;
    const std::vector<Eigen::Vector2d>& offsets_;
    const double conservative_radius_;

    const double lut_max_dist_ = 1.2;
    const double lut_step_size_ = 0.02;
    const int lut_size_ = lut_max_dist_ / lut_step_size_;
    std::vector<double> collision_cost_lut_;
};
}  // namespace astar_planner

#endif
