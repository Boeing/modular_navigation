#include <angles/angles.h>
#include <global_planner/orientation_filter.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace global_planner
{

namespace
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

double shortest_angular_distance(const double from, const double to)
{
    return normalize_angle(to - from);
}

void set_angle(geometry_msgs::PoseStamped* pose, double angle)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, angle);
    tf2::convert(q, pose->pose.orientation);
}

}

void OrientationFilter::processPath(std::vector<geometry_msgs::PoseStamped>& path)
{
    const std::size_t n = path.size();
    switch (omode_)
    {
        case FORWARD:
        {
            const std::size_t max_i = n > 0 ? n - 1 : 0UL;
            for (std::size_t i = 0; i < max_i; ++i)
            {
                setAngleBasedOnPositionDerivative(path, i);
            }
            break;
        }
        case BACKWARD:
        {
            const std::size_t max_i = n > 0 ? n - 1 : 0UL;
            for (std::size_t i = 0; i < max_i; ++i)
            {
                setAngleBasedOnPositionDerivative(path, i);
                set_angle(&path[i], normalize_angle(tf2::getYaw(path[i].pose.orientation) + M_PI));
            }
            break;
        }
        case LEFTWARD:
        {
            const std::size_t max_i = n > 0 ? n - 1 : 0UL;
            for (std::size_t i = 0; i < max_i; ++i)
            {
                setAngleBasedOnPositionDerivative(path, i);
                set_angle(&path[i], normalize_angle(tf2::getYaw(path[i].pose.orientation) - M_PI_2));
            }
            break;
        }
        case RIGHTWARD:
        {
            const std::size_t max_i = n > 0 ? n - 1 : 0UL;
            for (std::size_t i = 0; i < max_i; ++i)
            {
                setAngleBasedOnPositionDerivative(path, i);
                set_angle(&path[i], normalize_angle(tf2::getYaw(path[i].pose.orientation) + M_PI_2));
            }
            break;
        }
        case INTERPOLATE:
        {
            if (n > 1)
                interpolate(path, 0, n - 1);
            break;
        }
        case FORWARDTHENINTERPOLATE:
        {
            const std::size_t max_i = n > 0 ? n - 1 : 0UL;
            for (std::size_t i = 0; i < max_i; ++i)
            {
                setAngleBasedOnPositionDerivative(path, i);
            }

            if (n > 3)
            {
                std::size_t i = n - 3;
                const double last = tf2::getYaw(path[i].pose.orientation);
                while (i > 0)
                {
                    const double new_angle = tf2::getYaw(path[i - 1].pose.orientation);
                    double diff = fabs(shortest_angular_distance(new_angle, last));
                    if (diff > 0.35)
                        break;
                    else
                        i--;
                }
                interpolate(path, i, n - 1);
            }
            break;
        }
        case NONE:
            throw std::runtime_error("NONE OrientationFilter mode used");
    }
}

void OrientationFilter::setAngleBasedOnPositionDerivative(std::vector<geometry_msgs::PoseStamped>& path, const std::size_t index)
{
    assert (index < path.size());

    const std::size_t index0 = index > window_size_ ? index - window_size_ : 0UL;
    const std::size_t index1 = std::min(path.size() - 1, index + window_size_);

    const double x0 = path[index0].pose.position.x;
    const double y0 = path[index0].pose.position.y;
    const double x1 = path[index1].pose.position.x;
    const double y1 = path[index1].pose.position.y;

    const double angle = atan2(y1 - y0, x1 - x0);
    set_angle(&path[index], angle);
}

void OrientationFilter::interpolate(std::vector<geometry_msgs::PoseStamped>& path, const std::size_t start_index, const std::size_t end_index)
{
    assert (start_index < path.size());
    assert (end_index < path.size());
    assert (start_index < end_index);

    const double start_yaw = tf2::getYaw(path[start_index].pose.orientation);
    const double end_yaw = tf2::getYaw(path[end_index].pose.orientation);
    const double diff = shortest_angular_distance(start_yaw, end_yaw);

    const std::size_t step = end_index - start_index;
    const double increment = diff / static_cast<double>(step);

    for (std::size_t i = start_index; i <= end_index; ++i)
    {
        const double angle = start_yaw + increment * i;
        set_angle(&path[i], angle);
    }
}

};
