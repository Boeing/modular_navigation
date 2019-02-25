#include <eband_local_planner/conversions_and_types.h>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <vector>

namespace eband_local_planner
{

double normalize_angle_positive(const double angle)
{
    return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}

double normalize_angle(const double angle)
{
    double a = normalize_angle_positive(angle);
    if (a > M_PI)
        a -= 2.0 * M_PI;
    return a;
}

double distance2D(const geometry_msgs::Pose& start, const geometry_msgs::Pose& end)
{
    const double dx = start.position.x - end.position.x;
    const double dy = start.position.y - end.position.y;
    return std::sqrt((dx * dx) + (dy * dy));
}

double rotationZ(const geometry_msgs::Pose& start, const geometry_msgs::Pose& end)
{
    const tf2::Quaternion q_start(start.orientation.x, start.orientation.y, start.orientation.z, start.orientation.w);
    const tf2::Quaternion q_end(end.orientation.x, end.orientation.y, end.orientation.z, end.orientation.w);
    return normalize_angle(tf2::getYaw(q_end) - tf2::getYaw(q_start));
}

geometry_msgs::Pose2D convert(const geometry_msgs::Pose& pose)
{
    tf2::Quaternion qt(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    geometry_msgs::Pose2D ret;
    ret.x = pose.position.x;
    ret.y = pose.position.y;
    ret.theta = tf2::getYaw(qt);
    return ret;
}

geometry_msgs::Pose convert(const geometry_msgs::Pose2D& pose)
{
    tf2::Quaternion frame_quat;
    frame_quat.setRPY(0, 0, pose.theta);

    geometry_msgs::Pose ret;
    ret.position.x = pose.x;
    ret.position.y = pose.y;
    ret.position.z = 0.0;
    ret.orientation.x = frame_quat.x();
    ret.orientation.y = frame_quat.y();
    ret.orientation.z = frame_quat.z();
    ret.orientation.w = frame_quat.w();
    return ret;
}

std::vector<geometry_msgs::Pose> convert(const std::vector<Bubble>& band)
{
    std::vector<geometry_msgs::Pose> plan(band.size());
    for (std::size_t i = 0; i < band.size(); i++)
    {
        plan[i] = band[i].center;
    }
    return plan;
}

double costToDistance(const unsigned char cost, const double costmap_weight)
{
    if (cost >= costmap_2d::LETHAL_OBSTACLE || cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
        return 0.0;
    }
    else if (cost == costmap_2d::NO_INFORMATION)
    {
        return 0.0;
    }
    else
    {
        const unsigned char _cost = std::max(static_cast<unsigned char>(1), cost);
        const double factor = static_cast<double>(_cost) / (costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1);
        return -log(factor) / costmap_weight;
    }
}

std::vector<Eigen::Vector2i> drawLine(const Eigen::Vector2i& start, const Eigen::Vector2i& end)
{
    if (start == end)
        return {end};

    double x1 = start.x();
    double x2 = end.x();

    double y1 = start.y();
    double y2 = end.y();

    const bool steep = (std::abs(y2 - y1) > std::abs(x2 - x1));
    if (steep)
    {
        std::swap(x1, y1);
        std::swap(x2, y2);
    }

    bool reverse = false;
    if (x1 > x2)
    {
        std::swap(x1, x2);
        std::swap(y1, y2);
        reverse = true;
    }

    const double dx = x2 - x1;
    const double dy = std::abs(y2 - y1);

    double error = dx / 2.0;
    const int ystep = (y1 < y2) ? 1 : -1;
    int y = static_cast<int>(y1);

    const int max_x = static_cast<int>(x2);

    std::vector<Eigen::Vector2i> line;
    for (int x = static_cast<int>(x1); x < max_x; ++x)
    {
        if (steep)
        {
            line.push_back({y, x});
        }
        else
        {
            line.push_back({x, y});
        }

        error -= dy;
        if (error < 0)
        {
            y += ystep;
            error += dx;
        }
    }

    if (reverse)
        std::reverse(line.begin(), line.end());

    return line;
}

bool validPath(const geometry_msgs::Pose& start, const geometry_msgs::Pose& end, const costmap_2d::Costmap2D& costmap,
               const double costmap_weight, const double min_distance)
{
    unsigned int start_cell_x, start_cell_y;
    if (!costmap.worldToMap(start.position.x, start.position.y, start_cell_x, start_cell_y))
        return false;

    unsigned int end_cell_x, end_cell_y;
    if (!costmap.worldToMap(end.position.x, end.position.y, end_cell_x, end_cell_y))
        return false;

    const std::vector<Eigen::Vector2i> line =
        drawLine(Eigen::Vector2i(start_cell_x, start_cell_y), Eigen::Vector2i(end_cell_x, end_cell_y));

    for (std::size_t i = 0; i < line.size(); ++i)
    {
        const Eigen::Vector2i& p = line.at(i);
        unsigned char cost = costmap.getCost(static_cast<unsigned int>(p.x()), static_cast<unsigned int>(p.y()));
        const double distance = costToDistance(cost, costmap_weight);
        if (distance < min_distance)
        {
            ROS_WARN_STREAM("Collision at: cell=[" << p.x() << ", " << p.y() << "] at iteration: " << i
                                                   << " distance: " << distance << " cost: " << static_cast<int>(cost)
                                                   << " tiny: " << min_distance);
            return false;
        }
    }

    return true;
}

double obstacleDistance(const geometry_msgs::Pose& center_pose, const costmap_2d::Costmap2D& costmap,
                        const double costmap_weight, const double inflation_radius)
{
    unsigned int cell_x, cell_y;
    unsigned char disc_cost;
    if (!costmap.worldToMap(center_pose.position.x, center_pose.position.y, cell_x, cell_y))
    {
        // probably at the edge of the costmap - this value should be recovered soon
        disc_cost = 1;
    }
    else
    {
        // get cost for this cell
        disc_cost = costmap.getCost(cell_x, cell_y);
    }

    if (disc_cost == 0)
    {
        return inflation_radius;
    }
    else
    {
        return std::min(costToDistance(disc_cost, costmap_weight), inflation_radius);
    }
}

std::vector<Bubble> convert(const std::vector<geometry_msgs::Pose>& plan, const costmap_2d::Costmap2D& costmap,
                            const double costmap_weight, const double inflation_radius)
{
    std::vector<Bubble> band(plan.size());
    for (std::size_t i = 0; i < plan.size(); i++)
    {
        band[i].center = plan[i];
        const double distance = obstacleDistance(band[i].center, costmap, costmap_weight, inflation_radius);

        if (distance <= 0.0)
        {
            throw std::runtime_error("Failed to build band because pose " + std::to_string(i) + " is in collision");
        }

        band[i].expansion = distance;
    }

    return band;
}

std::vector<geometry_msgs::Pose> transform(const std::vector<geometry_msgs::PoseStamped>& plan, const tf2_ros::Buffer& tf_buffer, const std::string& frame_id)
{
    if (plan.empty())
        return {};

    tf2::Transform transform_;
    try
    {
        const geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(
                    frame_id,
                    ros::Time(),
                    plan.front().header.frame_id,
                    plan.front().header.stamp,
                    plan.front().header.frame_id);
        tf2::convert(transform.transform, transform_);
    }
    catch (const tf2::TransformException& e)
    {
        throw std::runtime_error("Failed to transform plan: " + std::string(e.what()));
    }

    std::vector<geometry_msgs::Pose> transformed;
    for (const geometry_msgs::PoseStamped& pose_stamped : plan)
    {
        tf2::Transform pose_;
        tf2::convert(pose_stamped.pose, pose_);

        const tf2::Transform transformed_pose_ = transform_ * pose_;

        geometry_msgs::Pose pose;
        pose.position.x = transformed_pose_.getOrigin().x();
        pose.position.y = transformed_pose_.getOrigin().y();
        pose.position.z = transformed_pose_.getOrigin().z();

        pose.orientation.w = transformed_pose_.getRotation().w();
        pose.orientation.x = transformed_pose_.getRotation().x();
        pose.orientation.y = transformed_pose_.getRotation().y();
        pose.orientation.z = transformed_pose_.getRotation().z();

        transformed.push_back(pose);
    }

    return transformed;
}

double getCircumscribedRadius(costmap_2d::Costmap2DROS& costmap)
{
    std::vector<geometry_msgs::Point> footprint(costmap.getRobotFootprint());
    double max_distance_sqr = 0;
    for (size_t i = 0; i < footprint.size(); ++i)
    {
        geometry_msgs::Point& p = footprint[i];
        double distance_sqr = p.x * p.x + p.y * p.y;
        if (distance_sqr > max_distance_sqr)
        {
            max_distance_sqr = distance_sqr;
        }
    }
    return sqrt(max_distance_sqr);
}
}
