#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <gridmap/layers/obstacle_layer.h>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/msg/marker.hpp>

// For logging reasons
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/asserts.hpp"

#include <chrono>

PLUGINLIB_EXPORT_CLASS(gridmap::ObstacleLayer, gridmap::Layer)

namespace gridmap
{

namespace
{

std::unordered_map<std::string, std::shared_ptr<gridmap::DataSource>>
    loadDataSources(const YAML::Node& parameters, pluginlib::ClassLoader<gridmap::DataSource>& loader,
                    const std::vector<Eigen::Vector2d>& robot_footprint,
                    const std::shared_ptr<RobotTracker>& robot_tracker, const std::shared_ptr<URDFTree>& urdf_tree)
{
    std::unordered_map<std::string, std::shared_ptr<gridmap::DataSource>> plugin_ptrs;
    const std::string param_name = "data_sources";
    if (parameters[param_name])
    {
        const YAML::Node& value = parameters[param_name];
        if (value.Type() != YAML::NodeType::Sequence)
        {
            throw std::runtime_error(param_name + " has incorrect type, expects a Sequence");
        }

        for (YAML::const_iterator it = value.begin(); it != value.end(); ++it)
        {
            rcpputils::assert_true(it->IsMap());

            std::string pname = (*it)["name"].as<std::string>();
            std::string type = (*it)["type"].as<std::string>();

            try
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger(""), "Loading plugin: " << pname << " type: " << type);
                const YAML::Node params = parameters[pname];
                std::shared_ptr<gridmap::DataSource> plugin_ptr =
                    std::shared_ptr<gridmap::DataSource>(loader.createUnmanagedInstance(type));
                plugin_ptr->initialize(pname, params, robot_footprint, robot_tracker, urdf_tree);
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
    : ds_loader_("gridmap", "gridmap::DataSource"), debug_viz_running_(false), clear_footprint_running_(false),
      time_decay_running_(false)
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

bool ObstacleLayer::draw(OccupancyGrid& grid) const
{
    // cppcheck-suppress unreadVariable
    const auto layer_lock = getReadLock();

    if (!probability_grid_)
        return false;

    if (!isDataOk())
        return false;

    // cppcheck-suppress unreadVariable
    const auto src_lock = probability_grid_->getReadLock();
    // cppcheck-suppress unreadVariable
    const auto dst_lock = grid.getWriteLock();
    const int size = dimensions().cells();
    for (int index = 0; index < size; ++index)
    {
        grid.cells()[index] = probability_grid_->occupied(index) ? OccupancyGrid::OCCUPIED : OccupancyGrid::FREE;
    }
    return true;
}

bool ObstacleLayer::draw(OccupancyGrid& grid, const AABB& bb) const
{
    // cppcheck-suppress unreadVariable
    const auto layer_lock = getReadLock();

    if (!probability_grid_)
        return false;

    if (!isDataOk())
        return false;

    // cppcheck-suppress unreadVariable
    const auto src_lock = probability_grid_->getReadLock();
    // cppcheck-suppress unreadVariable
    const auto dst_lock = grid.getWriteLock();

    rcpputils::assert_true(((bb.roi_start + bb.roi_size) <= grid.dimensions().size()).all());

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

bool ObstacleLayer::update(OccupancyGrid& grid) const
{
    // cppcheck-suppress unreadVariable
    const auto layer_lock = getReadLock();

    if (!probability_grid_)
        return false;

    if (!isDataOk())
        return false;

    // cppcheck-suppress unreadVariable
    const auto src_lock = probability_grid_->getReadLock();
    // cppcheck-suppress unreadVariable
    const auto dst_lock = grid.getWriteLock();
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

bool ObstacleLayer::update(OccupancyGrid& grid, const AABB& bb) const
{
    // cppcheck-suppress unreadVariable
    const auto layer_lock = getReadLock();

    if (!probability_grid_)
        return false;

    if (!isDataOk())
        return false;

    // cppcheck-suppress unreadVariable
    const auto src_lock = probability_grid_->getReadLock();
    // cppcheck-suppress unreadVariable
    const auto dst_lock = grid.getWriteLock();

    rcpputils::assert_true(((bb.roi_start + bb.roi_size) <= grid.dimensions().size()).all());

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

void ObstacleLayer::onInitialize(const YAML::Node& parameters)
{
    clamping_thres_min_ = parameters["clamping_thres_min"].as<double>(clamping_thres_min_);
    clamping_thres_max_ = parameters["clamping_thres_max"].as<double>(clamping_thres_max_);
    occ_prob_thres_ = parameters["occ_prob_thres"].as<double>(occ_prob_thres_);

    data_sources_ = loadDataSources(parameters, ds_loader_, robot_footprint_, robot_tracker_, urdf_tree_);

    time_decay_ = parameters["time_decay"].as<bool>(time_decay_);
    if (time_decay_)
    {
        time_decay_frequency_ = parameters["time_decay_frequency"].as<double>(time_decay_frequency_);
        alpha_decay_ = parameters["alpha_decay"].as<double>(alpha_decay_);
        rcpputils::assert_true(time_decay_frequency_ > 0);
    }

    debug_viz_ = parameters["debug_viz"].as<bool>(debug_viz_);
    if (debug_viz_)
    {
        debug_viz_frequency_ = parameters["debug_viz_frequency"].as<double>(debug_viz_frequency_);
        rcpputils::assert_true(debug_viz_frequency_ > 0);
    }
}

void ObstacleLayer::onMapChanged(const nav_msgs::msg::OccupancyGrid&)
{
    probability_grid_ =
        std::make_shared<ProbabilityGrid>(dimensions(), clamping_thres_min_, clamping_thres_max_, occ_prob_thres_);

    // cppcheck-suppress unreadVariable
    const auto grid_lock = probability_grid_->getWriteLock();
    for (auto plugin : data_sources_)
    {
        plugin.second->setMapData(probability_grid_);
    }

    if (debug_viz_)
    {
        //ros::NodeHandle nh(name());
        auto node = rclcpp::Node::make_shared(name());

        //debug_viz_pub_ = nh.advertise<nav_msgs::msg::OccupancyGrid>("costmap", 1);
        debug_viz_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 1);

        if (debug_viz_running_)
        {
            debug_viz_running_ = false;
            debug_viz_thread_.join();
        }
        debug_viz_running_ = true;
        debug_viz_thread_ = std::thread(&ObstacleLayer::debugVizThread, this, debug_viz_frequency_);
    }

    {
        if (clear_footprint_running_)
        {
            clear_footprint_running_ = false;
            clear_footprint_thread_.join();
        }
        clear_footprint_running_ = true;
        clear_footprint_thread_ = std::thread(&ObstacleLayer::clearFootprintThread, this, clear_footprint_frequency_);
    }

    if (time_decay_)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger(""),name() << ": enabling time decay freq: " << time_decay_frequency_
                               << " alpha: " << alpha_decay_);
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
    // cppcheck-suppress unreadVariable
    const auto layer_lock = getReadLock();

    if (!probability_grid_)
        return false;

    // cppcheck-suppress unreadVariable
    const auto grid_lock = probability_grid_->getWriteLock();
    std::fill(probability_grid_->cells().begin(), probability_grid_->cells().end(), 0.0);

    return true;
}

bool ObstacleLayer::clearRadius(const Eigen::Vector2i& cell_index, const int cell_radius)
{
    // cppcheck-suppress unreadVariable
    const auto layer_lock = getReadLock();

    if (!probability_grid_)
        return false;

    // cppcheck-suppress unreadVariable
    const auto grid_lock = probability_grid_->getWriteLock();
    // Write directly to probability_grid_ by reinterpreting the grid as a cv::Mat
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
            RCLCPP_WARN_STREAM(rclcpp::get_logger(""), "'" << ds.first << "' has stale data");
        ok &= ds_ok;
    }
    return ok;
}

void ObstacleLayer::debugVizThread(const double frequency)
{
    nav_msgs::msg::OccupancyGrid grid;
    //ros::Rate rate(frequency);
    rclcpp::Rate rate(frequency);
    //Changes in expectedCycleTime explained:
    //https://answers.ros.org/question/350222/expcectedcycletime-in-ros2/
    //const boost::chrono::milliseconds period(static_cast<long>(rate.expectedCycleTime().toSec() * 1000));
    const boost::chrono::milliseconds period(static_cast<long>(rate.period<double>() * (1.0/1e6)));

    while (debug_viz_running_ && rclcpp::ok())
    {
        {
            boost::shared_lock<boost::shared_timed_mutex> _lock(layer_mutex_, period);
            const RobotState robot_state = robot_tracker_->robotState();
            //if (_lock.owns_lock() && debug_viz_pub_.getNumSubscribers() != 0 && probability_grid_ &&
            if (_lock.owns_lock() && debug_viz_pub_->get_subscription_count() != 0 && probability_grid_ &&
                robot_state.localised)
            {
                grid.header.frame_id = "map";
                grid.info.resolution = static_cast<float>(probability_grid_->dimensions().resolution());
                grid.info.origin.orientation.w = 1.0;

                const int size_x = static_cast<int>(8.0 / probability_grid_->dimensions().resolution());
                const int size_y = static_cast<int>(8.0 / probability_grid_->dimensions().resolution());

                const Eigen::Isometry2d robot_pose = robot_state.map_to_odom * robot_state.odom.pose;
                const Eigen::Array2i robot_map = probability_grid_->dimensions().getCellIndex(robot_pose.translation());

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

                grid.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();

                {
                    // cppcheck-suppress unreadVariable
                    const auto lock = probability_grid_->getReadLock();

                    rcpputils::assert_true((top_left_x + actual_size_x) <= probability_grid_->dimensions().size().x());
                    rcpputils::assert_true((top_left_y + actual_size_y) <= probability_grid_->dimensions().size().y());

                    int roi_index = 0;
                    const int y_size = top_left_y + actual_size_y;
                    for (int y = top_left_y; y < y_size; y++)
                    {
                        const int index_start = probability_grid_->dimensions().size().x() * y + top_left_x;
                        const int index_end = index_start + actual_size_x;
                        for (int index = index_start; index < index_end; ++index)
                        {
                            rcpputils::assert_true(index < static_cast<int>(probability_grid_->cells().size()));
                            grid.data[roi_index] =
                                static_cast<int8_t>(probability(probability_grid_->cells()[index]) * 100.0);
                            ++roi_index;
                        }
                    }
                }

                const int max_occ = 100.0 * probability_grid_->ocupancyThres();
                for (size_t i = 0; i < grid.data.size(); ++i)
                {
                    if (grid.data[i] >= max_occ)
                        grid.data[i] = 101;
                }

                debug_viz_pub_->publish(grid);
            }
        }
        rate.sleep();
    }
}

void ObstacleLayer::clearFootprintThread(const double frequency)
{
    //ros::Rate rate(frequency);
    rclcpp::Rate rate(frequency);
    //const boost::chrono::milliseconds period(static_cast<long>(rate.expectedCycleTime().toSec() * 1000));
    const boost::chrono::milliseconds period(static_cast<long>(rate.period() * (1.0/1e6)));

    while (clear_footprint_running_ && rclcpp::ok())
    {
        {
            boost::shared_lock<boost::shared_timed_mutex> _lock(layer_mutex_, period);
            const RobotState robot_state = robot_tracker_->robotState();
            if (_lock.owns_lock() && probability_grid_ && robot_state.localised)
            {
                const Eigen::Isometry2d robot_pose = robot_state.map_to_odom * robot_state.odom.pose;
                const auto footprint =
                    buildFootprintSet(probability_grid_->dimensions(), robot_pose, robot_footprint_, 0.95);

                // cppcheck-suppress unreadVariable
                const auto pg_lock = probability_grid_->getWriteLock();
                for (const auto& elem : footprint)
                {
                    const Eigen::Array2i index = KeyToIndex(elem);
                    if (probability_grid_->dimensions().contains(index))
                        probability_grid_->setMinThres(index);
                }
            }
        }

        rate.sleep();
    }
}

void ObstacleLayer::timeDecayThread(const double frequency, const double alpha_decay)
{
    //ros::Rate rate(frequency);
    rclcpp::Rate rate(frequency);
    //const boost::chrono::milliseconds period(static_cast<long>(rate.expectedCycleTime().toSec() * 1000));
    const boost::chrono::milliseconds period(static_cast<long>(rate.period() * (1.0/1e6)));

    // divide the grid into a set of blocks which we can mark as dirty
    // only update dirty blocks
    // clear the blocks when the robot moves away from them
    const int block_size = 256;
    const Eigen::Array2i block_dims = probability_grid_->dimensions().size() / block_size;

    auto get_block_id = [block_size](const Eigen::Array2i& xy) { return xy.y() * block_size + xy.x(); };
    auto get_block_xy = [block_size](const int index) {
        return Eigen::Array2i(index % block_size, index / block_size);
    };

    auto& pg = probability_grid_;

    auto update_block = [&pg, alpha_decay](const Eigen::Array2i& block_xy) {
        const int top_left_x = block_size * block_xy.x();
        const int top_left_y = block_size * block_xy.y();
        const int size_x = std::min(pg->dimensions().size().x() - 1, top_left_x + block_size) - top_left_x;
        const int size_y = std::min(pg->dimensions().size().y() - 1, top_left_y + block_size) - top_left_y;
        const auto pg_lock = pg->getWriteLock();
        const int y_size = top_left_y + size_y;
        for (int y = top_left_y; y < y_size; y++)
        {
            const int index_start = pg->dimensions().size().x() * y + top_left_x;
            const int index_end = index_start + size_x;
            for (int index = index_start; index < index_end; ++index)
            {
                if (std::abs(pg->cell(index)) > 0.1)
                    pg->cell(index) -= pg->cell(index) * alpha_decay;
            }
        }
    };

    auto clear_block = [&pg](const Eigen::Array2i& block_xy) {
        const int top_left_x = block_size * block_xy.x();
        const int top_left_y = block_size * block_xy.y();
        const int size_x = std::min(pg->dimensions().size().x() - 1, top_left_x + block_size) - top_left_x;
        const int size_y = std::min(pg->dimensions().size().y() - 1, top_left_y + block_size) - top_left_y;
        const auto pg_lock = pg->getWriteLock();
        const int y_size = top_left_y + size_y;
        for (int y = top_left_y; y < y_size; y++)
        {
            const int index_start = pg->dimensions().size().x() * y + top_left_x;
            const int index_end = index_start + size_x;
            for (int index = index_start; index < index_end; ++index)
                pg->cell(index) = 0;
        }
    };

    const std::vector<Eigen::Array2i> neighbours = {{-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {0, 0},
                                                    {1, 0},   {-1, 1}, {0, 1},  {1, 1}};

    std::set<int> last_update;
    while (time_decay_running_ && rclcpp::ok())
    {
        {
            boost::shared_lock<boost::shared_timed_mutex> _lock(layer_mutex_, period);
            const RobotState robot_state = robot_tracker_->robotState();
            if (_lock.owns_lock() && probability_grid_ && robot_state.localised)
            {
                const Eigen::Isometry2d robot_pose = robot_state.map_to_odom * robot_state.odom.pose;
                const Eigen::Array2i robot_map = probability_grid_->dimensions().getCellIndex(robot_pose.translation());

                // determine block
                const Eigen::Array2i robot_map_block = robot_map / block_size;

                // collect 3x3 grid of neighbours
                std::set<int> dirty;
                for (const auto& n : neighbours)
                {
                    const Eigen::Array2i block = robot_map_block + n;
                    if ((Eigen::Array2i(0, 0) <= block).all() && (block < block_dims).all())
                    {
                        dirty.insert(get_block_id(block));
                    }
                }

                // update dirty
                for (const int block_id : dirty)
                    update_block(get_block_xy(block_id));

                // clear those that are no longer being updated
                for (const int block_id : last_update)
                    if (dirty.find(block_id) == dirty.end())
                        clear_block(get_block_xy(block_id));

                last_update = dirty;
            }
        }

        rate.sleep();
    }
}
}  // namespace gridmap
