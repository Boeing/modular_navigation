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
                    const std::vector<Eigen::Vector2d>& robot_footprint,
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
                XmlRpc::XmlRpcValue params = parameters[pname];
                std::shared_ptr<gridmap::DataSource> plugin_ptr =
                    std::shared_ptr<gridmap::DataSource>(loader.createUnmanagedInstance(type));
                plugin_ptr->initialize(pname, global_frame, params, robot_footprint, tf_buffer);
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
}  // namespace

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

bool ObstacleLayer::draw(OccupancyGrid& grid)
{
    std::lock_guard<std::timed_mutex> g(map_mutex_);

    if (!probability_grid_)
        return false;

    if (!isDataOk())
        return false;

    // cppcheck-suppress unreadVariable
    const auto lock = probability_grid_->getLock();
    const int size = dimensions().cells();
    for (int index = 0; index < size; ++index)
    {
        grid.cells()[index] = probability_grid_->occupied(index) ? OccupancyGrid::OCCUPIED : OccupancyGrid::FREE;
    }
    return true;
}

bool ObstacleLayer::draw(OccupancyGrid& grid, const AABB& bb)
{
    ROS_ASSERT(((bb.roi_start + bb.roi_size) <= grid.dimensions().size()).all());

    std::lock_guard<std::timed_mutex> g(map_mutex_);

    if (!probability_grid_)
        return false;

    if (!isDataOk())
        return false;

    // cppcheck-suppress unreadVariable
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
    return true;
}

bool ObstacleLayer::update(OccupancyGrid& grid)
{
    std::lock_guard<std::timed_mutex> g(map_mutex_);

    if (!probability_grid_)
        return false;

    if (!isDataOk())
        return false;

    // cppcheck-suppress unreadVariable
    const auto lock = probability_grid_->getLock();
    const int size = dimensions().cells();
    for (int index = 0; index < size; ++index)
    {
        if (probability_grid_->occupied(index))
        {
            grid.cells()[index] = OccupancyGrid::OCCUPIED;
        }
    }
    return true;
}

bool ObstacleLayer::update(OccupancyGrid& grid, const AABB& bb)
{
    ROS_ASSERT(((bb.roi_start + bb.roi_size) <= grid.dimensions().size()).all());

    std::lock_guard<std::timed_mutex> g(map_mutex_);

    if (!probability_grid_)
        return false;

    if (!isDataOk())
        return false;

    // cppcheck-suppress unreadVariable
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
    return true;
}

void ObstacleLayer::onInitialize(const XmlRpc::XmlRpcValue& parameters)
{
    clamping_thres_min_ = get_config_with_default_warn<double>(parameters, "clamping_thres_min", clamping_thres_min_,
                                                               XmlRpc::XmlRpcValue::TypeDouble);
    clamping_thres_max_ = get_config_with_default_warn<double>(parameters, "clamping_thres_max", clamping_thres_max_,
                                                               XmlRpc::XmlRpcValue::TypeDouble);
    occ_prob_thres_ = get_config_with_default_warn<double>(parameters, "occ_prob_thres", occ_prob_thres_,
                                                           XmlRpc::XmlRpcValue::TypeDouble);

    data_sources_ = loadDataSources(parameters, globalFrame(), ds_loader_, robotFootprint(), tfBuffer());

    time_decay_ =
        get_config_with_default_warn<bool>(parameters, "time_decay", time_decay_, XmlRpc::XmlRpcValue::TypeBoolean);
    if (time_decay_)
    {
        time_decay_frequency_ = get_config_with_default_warn<double>(
            parameters, "time_decay_frequency", time_decay_frequency_, XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(time_decay_frequency_ > 0);
        alpha_decay_ = get_config_with_default_warn<double>(parameters, "alpha_decay", alpha_decay_,
                                                            XmlRpc::XmlRpcValue::TypeDouble);
    }

    debug_viz_ =
        get_config_with_default_warn<bool>(parameters, "debug_viz", debug_viz_, XmlRpc::XmlRpcValue::TypeBoolean);
    if (debug_viz_)
    {
        debug_viz_rate_ = get_config_with_default_warn<double>(parameters, "debug_viz_rate", debug_viz_rate_,
                                                               XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(debug_viz_rate_ > 0);
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
            debug_viz_thread_.join();
        }
        debug_viz_running_ = true;
        debug_viz_thread_ = std::thread(&ObstacleLayer::debugVizThread, this, debug_viz_rate_);
    }

    if (time_decay_)
    {
        if (time_decay_running_)
        {
            time_decay_running_ = false;
            time_decay_thread_.join();
        }
        time_decay_running_ = true;
        time_decay_thread_ = std::thread(&ObstacleLayer::timeDecayThread, this, time_decay_frequency_, alpha_decay_);
    }
}

bool ObstacleLayer::clear()
{
    std::lock_guard<std::timed_mutex> g(map_mutex_);

    if (!probability_grid_)
        return false;

    // cppcheck-suppress unreadVariable
    auto lock = probability_grid_->getLock();
    std::fill(probability_grid_->cells().begin(), probability_grid_->cells().end(), 0.0);

    return true;
}

bool ObstacleLayer::clearRadius(const Eigen::Vector2i& cell_index, const int cell_radius)
{
    std::lock_guard<std::timed_mutex> g(map_mutex_);

    if (!probability_grid_)
        return false;

    // cppcheck-suppress unreadVariable
    auto lock = probability_grid_->getLock();
    cv::Mat cv_im = cv::Mat(probability_grid_->dimensions().size().y(), probability_grid_->dimensions().size().x(),
                            CV_64F, reinterpret_cast<void*>(probability_grid_->cells().data()));
    cv::circle(cv_im, cv::Point(cell_index.x(), cell_index.y()), cell_radius,
               cv::Scalar(probability_grid_->clampingThresMinLog()), -1);

    return true;
}

bool ObstacleLayer::isDataOk() const
{
    bool ok = true;
    for (const auto& ds : data_sources_)
    {
        const bool ds_ok = ds.second->isDataOk();
        if (!ds_ok)
            ROS_WARN_STREAM("'" << ds.first << "' has stale data");
        ok &= ds_ok;
    }
    return ok;
}

void ObstacleLayer::debugVizThread(const double frequency)
{
    nav_msgs::OccupancyGrid grid;
    ros::Rate rate(frequency);
    const std::chrono::milliseconds period(static_cast<long>(rate.expectedCycleTime().toSec()));

    while (debug_viz_running_ && ros::ok())
    {
        {
            std::unique_lock<std::timed_mutex> _lock(map_mutex_, period);
            if (_lock.owns_lock() && debug_viz_pub_.getNumSubscribers() != 0 && probability_grid_)
            {
                try
                {
                    grid.header.frame_id = globalFrame();
                    grid.info.resolution = static_cast<float>(probability_grid_->dimensions().resolution());
                    grid.info.origin.orientation.w = 1.0;

                    const int size_x = static_cast<int>(8.0 / probability_grid_->dimensions().resolution());
                    const int size_y = static_cast<int>(8.0 / probability_grid_->dimensions().resolution());

                    // Get robot pose
                    const geometry_msgs::TransformStamped tr =
                        tfBuffer()->lookupTransform(globalFrame(), "base_link", ros::Time(0));

                    const Eigen::Array2i robot_map = probability_grid_->dimensions().getCellIndex(
                        {tr.transform.translation.x, tr.transform.translation.y});

                    const int top_left_x = std::max(0, robot_map.x() - size_x / 2);
                    const int top_left_y = std::max(0, robot_map.y() - size_y / 2);

                    const int actual_size_x =
                        std::min(probability_grid_->dimensions().size().x() - 1, top_left_x + size_x) - top_left_x;
                    const int actual_size_y =
                        std::min(probability_grid_->dimensions().size().y() - 1, top_left_y + size_y) - top_left_y;

                    grid.info.width = static_cast<unsigned int>(actual_size_x);
                    grid.info.height = static_cast<unsigned int>(actual_size_y);

                    const std::size_t capacity = static_cast<size_t>(actual_size_x * actual_size_y);
                    if (grid.data.size() != capacity)
                        grid.data.resize(capacity);

                    grid.info.origin.position.x = probability_grid_->dimensions().origin().x() +
                                                  top_left_x * probability_grid_->dimensions().resolution();
                    grid.info.origin.position.y = probability_grid_->dimensions().origin().y() +
                                                  top_left_y * probability_grid_->dimensions().resolution();

                    grid.header.stamp = ros::Time::now();

                    auto lock = probability_grid_->getLock();

                    ROS_ASSERT((top_left_x + actual_size_x) <= probability_grid_->dimensions().size().x());
                    ROS_ASSERT((top_left_y + actual_size_y) <= probability_grid_->dimensions().size().y());

                    int roi_index = 0;
                    const int y_size = top_left_y + actual_size_y;
                    for (int y = top_left_y; y < y_size; y++)
                    {
                        const int index_start = probability_grid_->dimensions().size().x() * y + top_left_x;
                        const int index_end = index_start + actual_size_x;
                        for (int index = index_start; index < index_end; ++index)
                        {
                            ROS_ASSERT(index < static_cast<int>(probability_grid_->cells().size()));
                            grid.data[roi_index] =
                                static_cast<int8_t>(probability(probability_grid_->cells()[index]) * 100.0);
                            ++roi_index;
                        }
                    }

                    const int max_occ = 100.0 * probability_grid_->ocupancyThres();
                    for (size_t i = 0; i < grid.data.size(); ++i)
                    {
                        if (grid.data[i] >= max_occ)
                            grid.data[i] = 101;
                    }

                    debug_viz_pub_.publish(grid);
                }
                catch (const tf2::TransformException& e)
                {
                    ROS_WARN("Failed to publish debug. Unknown robot pose");
                }
            }
        }
        rate.sleep();
    }
}

void ObstacleLayer::timeDecayThread(const double frequency, const double alpha_decay)
{
    ros::Rate rate(frequency);
    const std::chrono::milliseconds period(static_cast<long>(rate.expectedCycleTime().toSec()));

    while (time_decay_running_ && ros::ok())
    {
        {
            std::unique_lock<std::timed_mutex> _lock(map_mutex_, period);
            if (_lock.owns_lock() && probability_grid_)
            {
                auto pg_lock = probability_grid_->getLock();
                const int cells = probability_grid_->dimensions().cells();
                for (int i = 0; i < cells; ++i)
                {
                    if (std::abs(probability_grid_->cell(i)) > 0.1)
                        probability_grid_->cell(i) -= probability_grid_->cell(i) * alpha_decay;
                }
            }
        }

        rate.sleep();
    }
}
}  // namespace gridmap
