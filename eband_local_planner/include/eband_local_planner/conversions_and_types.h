#ifndef EBAND_LOCAL_PLANNER_CONVERSIONS_AND_TYPES_H
#define EBAND_LOCAL_PLANNER_CONVERSIONS_AND_TYPES_H

#include <ros/ros.h>

#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d_ros.h>

namespace eband_local_planner
{

// defines a bubble - pose of center & radius of according hypersphere (expansion)
struct Bubble
{
    geometry_msgs::PoseStamped center;
    double expansion;
};

enum AddAtPosition
{
    add_front,
    add_back
};

double normalize_angle_positive(const double angle);
double normalize_angle(const double angle);

double distance2D(const geometry_msgs::Pose& start, const geometry_msgs::Pose& end);
double rotationZ(const geometry_msgs::Pose& start, const geometry_msgs::Pose& end);

geometry_msgs::Pose2D convert(const geometry_msgs::Pose& pose);
geometry_msgs::Pose convert(const geometry_msgs::Pose2D& pose);
std::vector<geometry_msgs::PoseStamped> convert(const std::vector<Bubble>& band);

double costToDistance(const unsigned char cost, const double costmap_weight);

std::vector<Eigen::Vector2i> drawLine(const Eigen::Vector2i& start, const Eigen::Vector2i& end);

bool validPath(const geometry_msgs::Pose& start, const geometry_msgs::Pose& end, const costmap_2d::Costmap2D& costmap,
               const double costmap_weight, const double min_distance);

double obstacleDistance(const geometry_msgs::Pose& center_pose, const costmap_2d::Costmap2D& costmap,
                        const double costmap_weight, const double inflation_radius);
std::vector<Bubble> convert(const std::vector<geometry_msgs::PoseStamped>& plan, const costmap_2d::Costmap2D& costmap,
                            const double costmap_weight, const double inflation_radius);

/**
 * @brief  Transforms the global plan of the robot from the planner frame to the local frame. This replaces the
 * transformGlobalPlan as defined in the base_local_planner/goal_functions.h main difference is that it additionally
 * outputs counter indicating which part of the plan has been transformed.
 * @param tf A reference to a transform listener
 * @param global_plan The plan to be transformed
 * @param costmap A reference to the costmap being used so the window size for transforming can be computed
 * @param global_frame The frame to transform the plan to
 * @param transformed_plan Populated with the transformed plan
 * @param number of start and end frame counted from the end of the global plan
 */
bool transformGlobalPlan(const tf2_ros::Buffer& tf_buffer, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                         costmap_2d::Costmap2DROS& costmap, const std::string& global_frame,
                         std::vector<geometry_msgs::PoseStamped>& transformed_plan,
                         std::vector<int>& start_end_counts_from_end);

/**
 * @brief Gets the footprint of the robot and computes the circumscribed radius for the eband approach
 * @param costmap A reference to the costmap from which the radius is computed
 * @return radius in meters
 */
double getCircumscribedRadius(costmap_2d::Costmap2DROS& costmap);
}

#endif
