#include <sim_band_planner/plugin.h>

#include <navigation_interface/params.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(sim_band_planner, SimBandPlanner, sim_band_planner::SimBandPlanner,
                        navigation_interface::TrajectoryPlanner)


namespace sim_band_planner
{

namespace
{

cv::Mat getPaddedROI(const cv::Mat& input, int top_left_x, int top_left_y, int width, int height, cv::Scalar paddingColor)
{
    const int bottom_right_x = top_left_x + width;
    const int bottom_right_y = top_left_y + height;

    cv::Mat output;
    if (top_left_x < 0 || top_left_y < 0 || bottom_right_x > input.cols || bottom_right_y > input.rows)
    {
        int border_left = 0, border_right = 0, border_top = 0, border_bottom = 0;

        if (top_left_x < 0) {
            width = width + top_left_x;
            border_left = -1 * top_left_x;
            top_left_x = 0;
        }
        if (top_left_y < 0) {
            height = height + top_left_y;
            border_top = -1 * top_left_y;
            top_left_y = 0;
        }
        if (bottom_right_x > input.cols) {
            width = width - (bottom_right_x - input.cols);
            border_right = bottom_right_x - input.cols;
        }
        if (bottom_right_y > input.rows) {
            height = height - (bottom_right_y - input.rows);
            border_bottom = bottom_right_y - input.rows;
        }

        cv::Rect R(top_left_x, top_left_y, width, height);
        cv::copyMakeBorder(input(R), output, border_top, border_bottom, border_left, border_right, cv::BORDER_CONSTANT, paddingColor);
    }
    else
    {
        cv::Rect R(top_left_x, top_left_y, width, height);
        input(R).copyTo(output);
    }
    return output;
}

}

SimBandPlanner::SimBandPlanner() : costmap_(nullptr), viz_(nullptr)
{
}

SimBandPlanner::~SimBandPlanner()
{
}

bool SimBandPlanner::setPath(const navigation_interface::Path& path)
{
    if (path.nodes.empty())
        return false;

    moving_window_.reset(new MovingWindow(path));

    return true;
}

void SimBandPlanner::clearPath()
{
    moving_window_.reset();
}

boost::optional<std::string> SimBandPlanner::pathId() const
{
    if (moving_window_)
        return boost::optional<std::string>(moving_window_->nominal_path.id);
    else
        return {};
}

boost::optional<navigation_interface::Path> SimBandPlanner::path() const
{
    if (moving_window_)
        return boost::optional<navigation_interface::Path>(moving_window_->nominal_path);
    else
        return {};
}

navigation_interface::TrajectoryPlanner::Result SimBandPlanner::plan(const navigation_interface::KinodynamicState& robot_state, const Eigen::Isometry2d& map_to_odom)
{
    navigation_interface::TrajectoryPlanner::Result result;

    if (!moving_window_)
    {
        result.outcome = navigation_interface::TrajectoryPlanner::Outcome::FAILED;
        return result;
    }

    try
    {
        moving_window_->updateWindow(robot_state.pose, max_window_length_);

        // Take a window of the planning scene at the robot pose
        const Eigen::Isometry2d robot_pose = map_to_odom * robot_state.pose;

        // Construct and cv::Mat from the costmap data
        const cv::Mat cv_im = cv::Mat(costmap_->getSizeInCellsY(), costmap_->getSizeInCellsX(), CV_8UC1,
                                      reinterpret_cast<void*>(costmap_->getCharMap()));

        const int size_x = 400;
        const int size_y = 400;

        unsigned int robot_map_x;
        unsigned int robot_map_y;
        if (!costmap_->worldToMap(robot_pose.translation().x(), robot_pose.translation().y(), robot_map_x, robot_map_y))
        {
            throw std::runtime_error("Robot is not on map!");
        }

        const int top_left_x = static_cast<int>(robot_map_x) - size_x/2;
        const int top_right_y = static_cast<int>(robot_map_y) - size_y/2;

        cv::Mat local_costmap = getPaddedROI(cv_im, top_left_x, top_right_y, size_x, size_y, cv::Scalar(254));
        local_costmap.setTo(0, local_costmap == 255);
        local_costmap.setTo(255, local_costmap > 253);

        // Update the world origin for the local costmap
        double origin_x = costmap_->getOriginX() + top_left_x * costmap_->getResolution();
        double origin_y = costmap_->getOriginY() + top_right_y * costmap_->getResolution();

        DistanceField distance_field(local_costmap, origin_x, origin_y, costmap_->getResolution(), 0.5);

        simulate(moving_window_->window,
                 distance_field,
                 num_iterations_,
                 min_overlap_,
                 min_distance_,
                 internal_force_gain_,
                 external_force_gain_,
                 rotation_factor_,
                 velocity_decay_,
                 1.0,
                 alpha_decay_);

        // debug viz
        {
            viz_->deleteAllMarkers();
            for (const auto& node : moving_window_->window.nodes)
            {
                ROS_INFO_STREAM("node: " << node.pose.translation().transpose() << " " << node.distance << " " << node.gradient.transpose());
                viz_->publishSphere(Eigen::Vector3d(node.pose.translation().x(), node.pose.translation().y(), 0.0), rviz_visual_tools::colors::GREEN, node.distance * 2);
            }
            viz_->trigger();
        }

        // Transform to odom frame
        result.trajectory.header.frame_id = "odom";
        result.cost = 0;
        result.outcome = navigation_interface::TrajectoryPlanner::Outcome::SUCCESSFUL;
        const Eigen::Isometry2d odom_to_map = map_to_odom.inverse();
        for (const auto& node : moving_window_->window.nodes)
        {
            const double velocity = 0.15 * std::min(1.0, node.distance);
            result.trajectory.states.push_back({odom_to_map * node.pose, Eigen::Vector3d(velocity, 0, 0)});
        }

        result.path_start_i = moving_window_->start_i;
        result.path_end_i = moving_window_->end_i;
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Optimization failed: " << e.what());
        result.outcome = navigation_interface::TrajectoryPlanner::Outcome::FAILED;
        return result;
    }

    return result;
}

bool SimBandPlanner::valid(const navigation_interface::Trajectory& trajectory) const
{

}

double SimBandPlanner::cost(const navigation_interface::Trajectory& trajectory) const
{

}

void SimBandPlanner::initialize(const XmlRpc::XmlRpcValue& parameters,
                        const std::shared_ptr<const costmap_2d::Costmap2D>& costmap)
{
    costmap_ = costmap;

//    viz_.reset(new rviz_visual_tools::RvizVisualTools("map", "debug"));

    num_iterations_ = navigation_interface::get_config_with_default_warn<int>(parameters, "num_iterations", num_iterations_, XmlRpc::XmlRpcValue::TypeInt);
    internal_force_gain_ = navigation_interface::get_config_with_default_warn<double>(parameters, "internal_force_gain", internal_force_gain_, XmlRpc::XmlRpcValue::TypeDouble);
    external_force_gain_ = navigation_interface::get_config_with_default_warn<double>(parameters, "external_force_gain", external_force_gain_, XmlRpc::XmlRpcValue::TypeDouble);
    min_distance_ = navigation_interface::get_config_with_default_warn<double>(parameters, "min_distance", min_distance_, XmlRpc::XmlRpcValue::TypeDouble);
    min_overlap_ = navigation_interface::get_config_with_default_warn<double>(parameters, "min_overlap", min_overlap_, XmlRpc::XmlRpcValue::TypeDouble);
    max_window_length_ = navigation_interface::get_config_with_default_warn<double>(parameters, "max_window_length", max_window_length_, XmlRpc::XmlRpcValue::TypeDouble);
    robot_radius_ = navigation_interface::get_config_with_default_warn<double>(parameters, "robot_radius", robot_radius_, XmlRpc::XmlRpcValue::TypeDouble);
    rotation_factor_ = navigation_interface::get_config_with_default_warn<double>(parameters, "rotation_factor", rotation_factor_, XmlRpc::XmlRpcValue::TypeDouble);
    velocity_decay_ = navigation_interface::get_config_with_default_warn<double>(parameters, "velocity_decay", velocity_decay_, XmlRpc::XmlRpcValue::TypeDouble);
    alpha_decay_ = navigation_interface::get_config_with_default_warn<double>(parameters, "alpha_decay", alpha_decay_, XmlRpc::XmlRpcValue::TypeDouble);
}

}
