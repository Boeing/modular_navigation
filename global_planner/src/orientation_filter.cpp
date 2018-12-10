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
}  // namespace

void set_angle(geometry_msgs::PoseStamped* pose, double angle)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, angle);
    tf2::convert(q, pose->pose.orientation);
}

void OrientationFilter::processPath(const geometry_msgs::PoseStamped& start,
                                    std::vector<geometry_msgs::PoseStamped>& path)
{
    unsigned int n = path.size();
    switch (omode_)
    {
        case FORWARD:
            for (unsigned int i = 0; i < n - 1; i++)
            {
                setAngleBasedOnPositionDerivative(path, i);
            }
            break;
        case BACKWARD:
            for (unsigned int i = 0; i < n - 1; i++)
            {
                setAngleBasedOnPositionDerivative(path, i);
                set_angle(&path[i], normalize_angle(tf2::getYaw(path[i].pose.orientation) + M_PI));
            }
            break;
        case LEFTWARD:
            for (unsigned int i = 0; i < n - 1; i++)
            {
                setAngleBasedOnPositionDerivative(path, i);
                set_angle(&path[i], normalize_angle(tf2::getYaw(path[i].pose.orientation) - M_PI_2));
            }
            break;
        case RIGHTWARD:
            for (unsigned int i = 0; i < n - 1; i++)
            {
                setAngleBasedOnPositionDerivative(path, i);
                set_angle(&path[i], normalize_angle(tf2::getYaw(path[i].pose.orientation) + M_PI_2));
            }
            break;
        case INTERPOLATE:
            path[0].pose.orientation = start.pose.orientation;
            interpolate(path, 0, n - 1);
            break;
        case FORWARDTHENINTERPOLATE:
        {
            for (unsigned int i = 0; i < n - 1; i++)
            {
                setAngleBasedOnPositionDerivative(path, i);
            }

            unsigned int i = n - 3;
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

            path[0].pose.orientation = start.pose.orientation;
            interpolate(path, i, n - 1);
            break;
        }
        case NONE:
            assert(true && "NONE OrientationFilter mode used");
            break;
        default:
            assert(true && "Unknown OrientationFilter mode used");
            break;
    }
}

void OrientationFilter::setAngleBasedOnPositionDerivative(std::vector<geometry_msgs::PoseStamped>& path,
                                                          unsigned int index)
{
    unsigned int index0 = u_int(std::max<int>(0, index - window_size_));
    unsigned int index1 = u_int(std::min<int>((int)path.size() - 1, index + window_size_));

    double x0 = path[index0].pose.position.x, y0 = path[index0].pose.position.y, x1 = path[index1].pose.position.x,
           y1 = path[index1].pose.position.y;

    double angle = atan2(y1 - y0, x1 - x0);
    set_angle(&path[index], angle);
}

void OrientationFilter::interpolate(std::vector<geometry_msgs::PoseStamped>& path, unsigned int start_index,
                                    unsigned int end_index)
{
    const double start_yaw = tf2::getYaw(path[start_index].pose.orientation),
                 end_yaw = tf2::getYaw(path[end_index].pose.orientation);
    double diff = shortest_angular_distance(start_yaw, end_yaw);
    double increment = diff / (end_index - start_index);
    for (unsigned int i = start_index; i <= end_index; i++)
    {
        double angle = start_yaw + increment * i;
        set_angle(&path[i], angle);
    }
}
};  // namespace global_planner
