#include <eband_local_planner/conversions_and_types.h>

#include <tf2/utils.h>

#include <string>
#include <vector>

namespace eband_local_planner
{

void PoseToPose2D(const geometry_msgs::Pose pose, geometry_msgs::Pose2D& pose2D)
{
    tf2::Quaternion qt(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    pose2D.x = pose.position.x;
    pose2D.y = pose.position.y;
    pose2D.theta = tf2::getYaw(qt);

    return;
}


void Pose2DToPose(geometry_msgs::Pose& pose, const geometry_msgs::Pose2D pose2D)
{
    // use tf-pkg to convert angles
    tf2::Quaternion frame_quat;
    frame_quat.setRPY(0, 0, pose2D.theta);

    // set position
    pose.position.x = pose2D.x;
    pose.position.y = pose2D.y;
    pose.position.z = 0.0;

    // set quaternion
    pose.orientation.x = frame_quat.x();
    pose.orientation.y = frame_quat.y();
    pose.orientation.z = frame_quat.z();
    pose.orientation.w = frame_quat.w();

    return;
}


bool transformGlobalPlan(const tf2_ros::Buffer& tf_buffer, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                         costmap_2d::Costmap2DROS& costmap, const std::string& global_frame,
                         std::vector<geometry_msgs::PoseStamped>& transformed_plan, std::vector<int>& start_end_counts)
{
    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

    // initiate refernce variables
    transformed_plan.clear();

    try
    {
        if (global_plan.empty())
        {
            ROS_ERROR("Received plan with zero length");
            return false;
        }

        const geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id);
        tf2::Transform transform_;
        tf2::convert(transform.transform, transform_);

        const geometry_msgs::TransformStamped robot_pose = tf_buffer.lookupTransform(costmap.getBaseFrameID(), plan_pose.header.frame_id, ros::Time(), ros::Duration(1.0));

        // we'll keep points on the plan that are within the window that we're looking at
        double dist_threshold =
            std::max(costmap.getCostmap()->getSizeInMetersX() / 2.0, costmap.getCostmap()->getSizeInMetersY() / 2.0);

        unsigned int i = 0;
        double sq_dist_threshold = dist_threshold * dist_threshold;
        double sq_dist = DBL_MAX;

        // initiate start_end_count
        std::vector<int> start_end_count;
        start_end_count.assign(2, 0);

        // we know only one direction and that is forward!
        // initiate search with previous start_end_counts this is necessary to work with the sampling based planners
        // path may severall time enter and leave moving window
        ROS_ASSERT((start_end_counts.at(0) > 0) && (start_end_counts.at(0) <= int(global_plan.size())));
        i = static_cast<unsigned int>(global_plan.size()) - static_cast<unsigned int>(start_end_counts.at(0));

        // we need to loop to a point on the plan that is within a certain distance of the robot
        while (i < global_plan.size() && sq_dist > sq_dist_threshold)
        {
            double x_diff = robot_pose.transform.translation.x - global_plan[i].pose.position.x;
            double y_diff = robot_pose.transform.translation.y - global_plan[i].pose.position.y;
            sq_dist = x_diff * x_diff + y_diff * y_diff;

            // not yet in reach - get next frame
            if (sq_dist > sq_dist_threshold)
            {
                ++i;
            }
            else
            {
                // set counter for start of transformed intervall - from back as beginning of plan might be prunned
                start_end_count.at(0) = global_plan.size() - i;
            }
        }

        // now we'll transform until points are outside of our distance threshold
        while (i < global_plan.size() && sq_dist < sq_dist_threshold)
        {
            double x_diff = robot_pose.transform.translation.x - global_plan[i].pose.position.x;
            double y_diff = robot_pose.transform.translation.y - global_plan[i].pose.position.y;
            sq_dist = x_diff * x_diff + y_diff * y_diff;

            const geometry_msgs::PoseStamped& pose = global_plan[i];

            tf2::Transform pose_;
            tf2::convert(pose, pose_);

            const tf2::Transform transformed_pose_ = transform_ * pose_;

            geometry_msgs::PoseStamped transformed_pose;
            transformed_pose.header.stamp = transform.header.stamp;
            transformed_pose.header.frame_id = transform.header.frame_id;

            transformed_pose.pose.position.x = transformed_pose_.getOrigin().x();
            transformed_pose.pose.position.y = transformed_pose_.getOrigin().y();
            transformed_pose.pose.position.z = transformed_pose_.getOrigin().z();

            transformed_pose.pose.orientation.w = transformed_pose_.getRotation().w();
            transformed_pose.pose.orientation.x = transformed_pose_.getRotation().x();
            transformed_pose.pose.orientation.y = transformed_pose_.getRotation().y();
            transformed_pose.pose.orientation.z = transformed_pose_.getRotation().z();

            transformed_plan.push_back(transformed_pose);

            // set counter for end of transformed intervall - from back as beginning of plan might be prunned
            start_end_count.at(1) = global_plan.size() - i;

            ++i;
        }

        // write to reference variable
        start_end_counts = start_end_count;
    }
    catch (tf2::TransformException& ex)
    {
        ROS_ERROR_STREAM("TransformException: " << ex.what());
        return false;
    }

    return true;
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

}  // namespace eband_local_planner
