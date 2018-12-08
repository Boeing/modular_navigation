#include <global_planner/planner_core.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <pluginlib/class_list_macros.h>

#include <global_planner/astar.h>
#include <global_planner/dijkstra.h>
#include <global_planner/gradient_path.h>
#include <global_planner/grid_path.h>
#include <global_planner/quadratic_calculator.h>

PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

namespace global_planner
{

GlobalPlanner::GlobalPlanner()
  : tf_buffer_(nullptr),
    global_costmap_(nullptr),
    local_costmap_(nullptr),
    allow_unknown_(true)
{
}

GlobalPlanner::~GlobalPlanner()
{
}

void GlobalPlanner::initialize(std::string name,
                            std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                            std::shared_ptr<costmap_2d::Costmap2DROS> global_costmap,
                            std::shared_ptr<costmap_2d::Costmap2DROS> local_costmap)
{
    tf_buffer_ = tf_buffer;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    ros::NodeHandle private_nh("~/" + name);

    const unsigned int cx = global_costmap->getCostmap()->getSizeInCellsX();
    const unsigned int cy = global_costmap->getCostmap()->getSizeInCellsY();

    bool use_quadratic;
    private_nh.param("use_quadratic", use_quadratic, true);
    if (use_quadratic)
        p_calc_ = std::make_shared<QuadraticCalculator>(cx, cy);
    else
        p_calc_ = std::make_shared<PotentialCalculator>(cx, cy);

    bool use_dijkstra;
    private_nh.param("use_dijkstra", use_dijkstra, true);
    if (use_dijkstra)
    {
        auto de = std::make_shared<DijkstraExpansion>(p_calc_, cx, cy);
        de->setPreciseStart(true);
        planner_ = de;
    }
    else
        planner_ = std::make_shared<AStarExpansion>(p_calc_, cx, cy);

    bool use_grid_path;
    private_nh.param("use_grid_path", use_grid_path, false);
    if (use_grid_path)
        path_maker_ = std::make_shared<GridPath>(p_calc_);
    else
        path_maker_ = std::make_shared<GradientPath>(p_calc_);

    orientation_filter_ = std::make_shared<OrientationFilter>();

    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

    private_nh.param("allow_unknown", allow_unknown_, true);
    planner_->setHasUnknown(allow_unknown_);
    private_nh.param("publish_scale", publish_scale_, 100);

    dsrv_ = std::unique_ptr<dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>>(
        new dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>(private_nh)
    );
    dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>::CallbackType cb =
        boost::bind(&GlobalPlanner::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

void GlobalPlanner::reconfigureCB(global_planner::GlobalPlannerConfig& config, uint32_t level)
{
    planner_->setLethalCost(config.lethal_cost);
    path_maker_->setLethalCost(config.lethal_cost);
    planner_->setNeutralCost(config.neutral_cost);
    planner_->setFactor(config.cost_factor);
    publish_potential_ = config.publish_potential;
    orientation_filter_->setMode(config.orientation_mode);
    orientation_filter_->setWindowSize(config.orientation_window_size);
}

/*
void GlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy)
{
    wx = costmap_->getOriginX() + (mx + convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my + convert_offset_) * costmap_->getResolution();
}

bool GlobalPlanner::worldToMap(double wx, double wy, double& mx, double& my)
{
    double origin_x = costmap_->getOriginX();
    double origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}
*/

nav_core::PlanResult GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal)
{
    std::lock_guard<std::mutex> lock(mutex_);

    nav_core::PlanResult result;

    const std::string robot_frame = local_costmap_->getBaseFrameID();
    const std::string local_frame = local_costmap_->getGlobalFrameID();
    const std::string global_frame = global_costmap_->getGlobalFrameID();
    const ros::Time now = ros::Time::now();

    if (goal.header.frame_id != global_frame)
    {
        ROS_ERROR_STREAM("Goal must be in global_frame: " << global_frame);
        result.success = false;
        return result;
    }

    if (start.header.frame_id != global_frame)
    {
        ROS_ERROR_STREAM("Start must be in global_frame: " << global_frame);
        result.success = false;
        return result;
    }

    //
    // Copy the local costmap data
    // We can't spend too much time doing this because the local planner needs access to it
    //
    cv::Mat local_costmap;
    std::vector<unsigned char> local_costmap_data;
    unsigned int l_x_size;
    unsigned int l_y_size;
    double l_resolution;
    double l_origin_x;
    double l_origin_y;
    {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> g(*local_costmap_->getCostmap()->getMutex());
        l_x_size = local_costmap_->getCostmap()->getSizeInCellsX();
        l_y_size = local_costmap_->getCostmap()->getSizeInCellsY();
        l_resolution = local_costmap_->getCostmap()->getResolution();
        l_origin_x = local_costmap_->getCostmap()->getOriginX();
        l_origin_y = local_costmap_->getCostmap()->getOriginY();
        const unsigned int _size = l_x_size * l_y_size;
        local_costmap_data.resize(_size);
        std::copy_n(local_costmap_->getCostmap()->getCharMap(), _size, local_costmap_data.begin());
        local_costmap = cv::Mat(l_y_size, l_x_size, CV_8UC1, local_costmap_data.data());
    }

    //
    // Global costmap properties
    //
    const unsigned int g_x_size = global_costmap_->getCostmap()->getSizeInCellsX();
    const unsigned int g_y_size = global_costmap_->getCostmap()->getSizeInCellsY();
    const double g_resolution = global_costmap_->getCostmap()->getResolution();
    const double g_origin_x = global_costmap_->getCostmap()->getOriginX();
    const double g_origin_y = global_costmap_->getCostmap()->getOriginY();

    //
    // Merged map
    // resolution changes
    // origin stays the same
    //
    const double scale = g_resolution / l_resolution;
    const unsigned int mm_size_x = g_x_size * scale;
    const unsigned int mm_size_y = g_y_size * scale;
    const double mm_origin_x = g_origin_x;
    const double mm_origin_y = g_origin_y;
    const double mm_resolution = l_resolution;

//    ROS_INFO_STREAM("scale: " << scale);

//    ROS_INFO_STREAM("l_x_size: " << l_x_size);
//    ROS_INFO_STREAM("l_y_size: " << l_y_size);
//    ROS_INFO_STREAM("l_resolution: " << l_resolution);
//    ROS_INFO_STREAM("l_origin_x: " << l_origin_x);
//    ROS_INFO_STREAM("l_origin_y: " << l_origin_y);

//    ROS_INFO_STREAM("g_x_size: " << g_x_size);
//    ROS_INFO_STREAM("g_y_size: " << g_y_size);
//    ROS_INFO_STREAM("g_resolution: " << g_resolution);
//    ROS_INFO_STREAM("g_origin_x: " << g_origin_x);
//    ROS_INFO_STREAM("g_origin_y: " << g_origin_y);

//    ROS_INFO_STREAM("mm_size_x: " << mm_size_x);
//    ROS_INFO_STREAM("mm_size_y: " << mm_size_y);
//    ROS_INFO_STREAM("mm_resolution: " << mm_resolution);
//    ROS_INFO_STREAM("mm_origin_x: " << mm_origin_x);
//    ROS_INFO_STREAM("mm_origin_y: " << mm_origin_y);

    //
    // Copy the global costmap data but resize to match the local costmap resolution
    //
    cv::Mat merged_costmap;
    std::vector<unsigned char> merged_costmap_data;
    cv::Mat global_costmap(g_y_size, g_x_size, CV_8UC1, global_costmap_->getCostmap()->getCharMap());
    cv::resize(global_costmap, merged_costmap, cv::Size(mm_size_y, mm_size_x), 0, 0, cv::INTER_LINEAR);

//    ROS_INFO_STREAM("merged_costmap: " << merged_costmap.size);

    //
    // Super impose the local costmap data
    //
    int merged_local_x = (l_origin_x - mm_origin_x) / mm_resolution;
    int merged_local_y = (l_origin_y - mm_origin_y) / mm_resolution;
    local_costmap.copyTo(merged_costmap(cv::Rect(merged_local_x, merged_local_y, local_costmap.cols, local_costmap.rows)));
    outlineMap(merged_costmap.data, mm_size_x, mm_size_y, costmap_2d::LETHAL_OBSTACLE);

//    cv::imwrite("/home/boeing/local.png", local_costmap);
//    cv::imwrite("/home/boeing/global.png", global_costmap);
//    cv::imwrite("/home/boeing/merged.png", merged_costmap);

    //
    // Calculate start and end map coordinates
    //
    const double start_x = (start.pose.position.x - mm_origin_x) / mm_resolution - 0.5;
    const double start_y = (start.pose.position.y - mm_origin_y) / mm_resolution - 0.5;
    const double goal_x = (goal.pose.position.x - mm_origin_x) / mm_resolution - 0.5;
    const double goal_y = (goal.pose.position.y - mm_origin_y) / mm_resolution - 0.5;

//    ROS_INFO_STREAM("START: " << start.pose.position.x << " " << start.pose.position.y);
//    ROS_INFO_STREAM("GOAL: " << goal.pose.position.x << " " << goal.pose.position.y);

//    ROS_INFO_STREAM("START: " << start_x << " " << start_y);
//    ROS_INFO_STREAM("GOAL: " << goal_x << " " << goal_y);

    if (start_x < 0 || start_x > mm_size_x || start_y < 0 || start_y > mm_size_y)
    {
        ROS_WARN("The robot's start position is outside the global costmap");
        result.success = false;
        return result;
    }

    if (goal_x < 0 || goal_x > mm_size_x || goal_y < 0 || goal_y > mm_size_y)
    {
        ROS_WARN("The goal position is outside the global costmap");
        result.success = false;
        return result;
    }

    // TODO clear the starting / end cell within the costmap because we know it can't be an obstacle

    p_calc_->setSize(mm_size_x, mm_size_y);
    planner_->setSize(mm_size_x, mm_size_y);
    path_maker_->setSize(mm_size_x, mm_size_y);

    std::vector<float> potential_array(mm_size_x * mm_size_y);

    bool found_legal = planner_->calculatePotentials(
                merged_costmap.data,
                start_x, start_y,
                goal_x, goal_y,
                mm_size_x * mm_size_y * 2,
                &potential_array[0]);

    if (publish_potential_)
    {
        nav_msgs::OccupancyGrid grid;

        grid.header.frame_id = global_frame;
        grid.header.stamp = now;
        grid.info.resolution = mm_resolution;
        grid.info.width = mm_size_x;
        grid.info.height = mm_size_y;
        grid.info.origin.position.x = mm_origin_x;
        grid.info.origin.position.y = mm_origin_y;
        grid.info.origin.orientation.w = 1.0;

        grid.data.resize(mm_size_x * mm_size_y);

        float max = 0.0;
        for (unsigned int i = 0; i < grid.data.size(); i++)
        {
            const float p = potential_array[i];
            if (p < POT_HIGH)
            {
                if (p > max)
                {
                    max = p;
                }
            }
        }

        for (unsigned int i = 0; i < grid.data.size(); i++)
        {
            if (potential_array[i] >= POT_HIGH)
            {
                grid.data[i] = -1;
            }
            else
                grid.data[i] = potential_array[i] * publish_scale_ / max;
        }
        potential_pub_.publish(grid);
    }

    if (found_legal)
    {
        // extract the plan
        std::vector<std::pair<float, float>> path;
        if (!path_maker_->getPath(&potential_array[0], start_x, start_y, goal_x, goal_y, path))
        {
            ROS_ERROR("NO PATH!");
            result.success = false;
            return result;
        }

        for (int i = path.size() - 1; i >= 0; i--)
        {
            std::pair<float, float> point = path[i];

            geometry_msgs::PoseStamped pose;
            pose.header.stamp = now;
            pose.header.frame_id = global_frame;
            pose.pose.position.x = mm_origin_x + (point.first + 0.5) * mm_resolution;
            pose.pose.position.y = mm_origin_y + (point.second + 0.5) * mm_resolution;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            result.plan.push_back(pose);
        }
        geometry_msgs::PoseStamped goal_copy = goal;
        goal_copy.header.stamp = now;
        result.plan.push_back(goal_copy);
    }
    else
    {
        ROS_ERROR("Failed to get a plan.");
    }

    // add orientations if needed
    orientation_filter_->processPath(start, result.plan);

    // publish the plan for visualization purposes
    publishPlan(result.plan);

    result.success = true;
    return result;
}

void GlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
    assert (!path.empty());
    nav_msgs::Path gui_path;
    gui_path.header = path.front().header;
    gui_path.poses = path;
    plan_pub_.publish(gui_path);
}

void GlobalPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value)
{
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

}  // end namespace global_planner
