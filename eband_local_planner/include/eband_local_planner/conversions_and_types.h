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

struct Bubble
{
    geometry_msgs::Pose center;
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
std::vector<geometry_msgs::Pose> convert(const std::vector<Bubble>& band);

double costToDistance(const unsigned char cost, const double costmap_weight);

std::vector<Eigen::Vector2i> drawLine(const Eigen::Vector2i& start, const Eigen::Vector2i& end);

bool validPath(const geometry_msgs::Pose& start, const geometry_msgs::Pose& end, const costmap_2d::Costmap2D& costmap,
               const double costmap_weight, const double min_distance);

double obstacleDistance(const geometry_msgs::Pose& center_pose, const costmap_2d::Costmap2D& costmap,
                        const double costmap_weight, const double inflation_radius);
std::vector<Bubble> convert(const std::vector<geometry_msgs::Pose>& plan, const costmap_2d::Costmap2D& costmap,
                            const double costmap_weight, const double inflation_radius);

std::vector<geometry_msgs::Pose> transform(const std::vector<geometry_msgs::PoseStamped>& plan, const tf2_ros::Buffer& tf_buffer, const std::string& frame_id);

double getCircumscribedRadius(costmap_2d::Costmap2DROS& costmap);

}

#endif
