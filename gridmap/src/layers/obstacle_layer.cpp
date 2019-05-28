#include <gridmap/layers/obstacle_layer.h>
#include <gridmap/params.h>

#include <opencv2/imgproc.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(gridmap::ObstacleLayer, gridmap::Layer)

namespace gridmap
{

namespace
{

std::unordered_map<std::string, std::shared_ptr<gridmap::DataSource>>
    loadDataSources(XmlRpc::XmlRpcValue parameters, const std::string& global_frame,
                    pluginlib::ClassLoader<gridmap::DataSource>& loader,
                    const std::shared_ptr<tf2_ros::Buffer>& tf_buffer)
{
    std::unordered_map<std::string, std::shared_ptr<gridmap::DataSource>> plugin_ptrs;
    const std::string param_name = "data_sources";
    if (parameters.hasMember(param_name))
    {
        XmlRpc::XmlRpcValue& value = parameters[param_name];
        if (value.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            throw std::runtime_error(param_name + " has incorrect type, expects a TypeArray");
        }

        for (int32_t i = 0; i < value.size(); ++i)
        {
            std::string pname = static_cast<std::string>(value[i]["name"]);
            std::string type = static_cast<std::string>(value[i]["type"]);

            try
            {
                ROS_INFO_STREAM("Loading plugin: " << pname << " type: " << type);
                std::shared_ptr<gridmap::DataSource> plugin_ptr =
                    std::shared_ptr<gridmap::DataSource>(loader.createUnmanagedInstance(type));
                XmlRpc::XmlRpcValue params = parameters[pname];
                plugin_ptr->initialize(pname, global_frame, params, tf_buffer);
                plugin_ptrs[pname] = plugin_ptr;
            }
            catch (const pluginlib::PluginlibException& e)
            {
                throw std::runtime_error("Exception while loading plugin '" + pname + "': " + std::string(e.what()));
            }
            catch (const std::exception& e)
            {
                throw std::runtime_error("Exception while loading plugin '" + pname + "': " + std::string(e.what()));
            }
        }
    }

    return plugin_ptrs;
}
}

ObstacleLayer::ObstacleLayer()
    : ds_loader_("gridmap", "gridmap::DataSource"), debug_viz_running_(false), time_decay_running_(false)
{
}

ObstacleLayer::~ObstacleLayer()
{
    debug_viz_running_ = false;
    time_decay_running_ = false;
    if (time_decay_ && time_decay_thread_.joinable())
        time_decay_thread_.join();
    if (debug_viz_ && debug_viz_thread_.joinable())
        debug_viz_thread_.join();
}

void ObstacleLayer::draw(OccupancyGrid& grid)
{
    const auto lock = probability_grid_->getLock();
    const int size = dimensions().cells();
    for (int index = 0; index < size; ++index)
    {
        grid.cells()[index] = probability_grid_->occupied(index) ? OccupancyGrid::OCCUPIED : OccupancyGrid::FREE;
    }
}

void ObstacleLayer::draw(OccupancyGrid& grid, const AABB& bb)
{
    const auto lock = probability_grid_->getLock();
    const int y_size = bb.roi_start.y() + bb.roi_size.y();
    for (int y = bb.roi_start.y(); y < y_size; y++)
    {
        const int index_start = dimensions().size().x() * y + bb.roi_start.x();
        const int index_end = dimensions().size().x() * y + bb.roi_start.x() + bb.roi_size.x();
        for (int index = index_start; index < index_end; ++index)
        {
            grid.cells()[index] = probability_grid_->occupied(index) ? OccupancyGrid::OCCUPIED : OccupancyGrid::FREE;
        }
    }
}

void ObstacleLayer::update(OccupancyGrid& grid)
{
    const auto lock = probability_grid_->getLock();
    const int size = dimensions().cells();
    for (int index = 0; index < size; ++index)
    {
        if (probability_grid_->occupied(index))
        {
            grid.cells()[index] = OccupancyGrid::OCCUPIED;
        }
    }
}

void ObstacleLayer::update(OccupancyGrid& grid, const AABB& bb)
{
    const auto lock = probability_grid_->getLock();
    const int y_size = bb.roi_start.y() + bb.roi_size.y();
    for (int y = bb.roi_start.y(); y < y_size; y++)
    {
        const int index_start = dimensions().size().x() * y + bb.roi_start.x();
        const int index_end = dimensions().size().x() * y + bb.roi_start.x() + bb.roi_size.x();
        for (int index = index_start; index < index_end; ++index)
        {
            if (probability_grid_->occupied(index))
                grid.cells()[index] = OccupancyGrid::OCCUPIED;
        }
    }
}

void ObstacleLayer::onInitialize(const XmlRpc::XmlRpcValue& parameters)
{

    clamping_thres_min_ =
        get_config_with_default_warn<double>(parameters, "clamping_thres_min", 0.1192, XmlRpc::XmlRpcValue::TypeDouble);
    clamping_thres_max_ =
        get_config_with_default_warn<double>(parameters, "clamping_thres_max", 0.971, XmlRpc::XmlRpcValue::TypeDouble);
    occ_prob_thres_ =
        get_config_with_default_warn<double>(parameters, "occ_prob_thres", 0.8, XmlRpc::XmlRpcValue::TypeDouble);

    time_decay_ = get_config_with_default_warn<bool>(parameters, "time_decay", false, XmlRpc::XmlRpcValue::TypeBoolean);
    if (time_decay_)
    {
        time_decay_frequency_ = get_config_with_default_warn<double>(parameters, "time_decay_frequency", 0.1,
                                                                     XmlRpc::XmlRpcValue::TypeDouble);
        time_decay_step_ =
            get_config_with_default_warn<double>(parameters, "time_decay_step", 0.1, XmlRpc::XmlRpcValue::TypeDouble);
    }

    data_sources_ = loadDataSources(parameters, globalFrame(), ds_loader_, tfBuffer());

    debug_viz_ = get_config_with_default_warn<bool>(parameters, "debug_viz", true, XmlRpc::XmlRpcValue::TypeBoolean);
    if (debug_viz_)
    {
        debug_viz_rate_ =
            get_config_with_default_warn<double>(parameters, "debug_viz_rate", 4.0, XmlRpc::XmlRpcValue::TypeDouble);
    }
}

void ObstacleLayer::onMapChanged(const nav_msgs::OccupancyGrid&)
{
    probability_grid_ =
        std::make_shared<ProbabilityGrid>(dimensions(), clamping_thres_min_, clamping_thres_max_, occ_prob_thres_);

    for (auto plugin : data_sources_)
    {
        plugin.second->setMapData(probability_grid_);
    }

    if (debug_viz_)
    {
        ros::NodeHandle nh(name());
        debug_viz_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("costmap", 1);
        if (debug_viz_running_)
        {
            debug_viz_running_ = false;
            time_decay_thread_.join();
        }
        debug_viz_running_ = true;
        debug_viz_thread_ = std::thread(&ObstacleLayer::debugVizThread, this, debug_viz_rate_);
    }

    if (time_decay_)
    {
        if (debug_viz_running_)
        {
            time_decay_running_ = false;
            time_decay_thread_.join();
        }
        time_decay_running_ = true;
        time_decay_thread_ =
            std::thread(&ObstacleLayer::timeDecayThread, this, time_decay_frequency_, time_decay_step_);
    }
}

void ObstacleLayer::clearRadius(const Eigen::Vector2i& cell_index, const int cell_radius)
{
    auto lock = probability_grid_->getLock();
    cv::Mat cv_im = cv::Mat(probability_grid_->dimensions().size().y(), probability_grid_->dimensions().size().x(),
                            CV_64F, reinterpret_cast<void*>(probability_grid_->cells().data()));
    cv::circle(cv_im, cv::Point(cell_index.x(), cell_index.y()), cell_radius,
               cv::Scalar(probability_grid_->clampingThresMinLog()));
}

void ObstacleLayer::debugVizThread(const double frequency)
{
    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = globalFrame();
    grid.info.resolution = probability_grid_->dimensions().resolution();
    grid.info.origin.orientation.w = 1.0;

    const int size_x = 400;
    const int size_y = 400;

    grid.info.width = size_x;
    grid.info.height = size_y;
    grid.data.resize(grid.info.width * grid.info.height);

    ros::Rate rate(frequency);
    while (debug_viz_running_ && ros::ok())
    {
        if (debug_viz_pub_.getNumSubscribers() != 0)
        {
            // Get robot pose
            const geometry_msgs::TransformStamped tr =
                tfBuffer()->lookupTransform(globalFrame(), "base_link", ros::Time(0));

            const Eigen::Array2i robot_map =
                probability_grid_->dimensions().getCellIndex({tr.transform.translation.x, tr.transform.translation.y});

            const int top_left_x = robot_map.x() - size_x / 2;
            const int top_left_y = robot_map.y() - size_y / 2;

            grid.info.origin.position.x = probability_grid_->dimensions().origin().x() +
                                          top_left_x * probability_grid_->dimensions().resolution();
            grid.info.origin.position.y = probability_grid_->dimensions().origin().y() +
                                          top_left_y * probability_grid_->dimensions().resolution();

            grid.header.stamp = ros::Time::now();

            auto lock = probability_grid_->getLock();

            int8_t max = 0;

            int roi_index = 0;
            const int y_size = top_left_y + size_y;
            for (int y = top_left_y; y < y_size; y++)
            {
                const int index_start = probability_grid_->dimensions().size().x() * y + top_left_x;
                const int index_end = index_start + size_x;
                for (int index = index_start; index < index_end; ++index)
                {
                    grid.data[roi_index] = static_cast<int8_t>(probability(probability_grid_->cells()[index]) * 100.0);

                    max = std::max(grid.data[roi_index], max);
                    ++roi_index;
                }
            }

            const int max_occ = 100.0 * probability_grid_->ocupancyThres();
            for (int i = 0; i < grid.data.size(); ++i)
            {
                if (grid.data[i] >= max_occ)
                    grid.data[i] = 101;
            }

            debug_viz_pub_.publish(grid);
        }

        rate.sleep();
    }
}

void ObstacleLayer::timeDecayThread(const double frequency, const double log_odds_decay)
{
    ros::Rate rate(frequency);
    while (time_decay_running_ && ros::ok())
    {
        rate.sleep();
    }
}
}
