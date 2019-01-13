#include <clear_costmap_recovery/clear_costmap_recovery.h>
#include <costmap_2d/cost_values.h>
#include <pluginlib/class_list_macros.h>
#include <vector>

PLUGINLIB_EXPORT_CLASS(clear_costmap_recovery::ClearCostmapRecovery, nav_core::RecoveryBehavior)

namespace clear_costmap_recovery
{
ClearCostmapRecovery::ClearCostmapRecovery() : tf_buffer_(nullptr), local_costmap_(nullptr), global_costmap_(nullptr)
{
}

void ClearCostmapRecovery::initialize(const std::string& name, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                      const std::shared_ptr<costmap_2d::Costmap2DROS>& global_costmap,
                                      const std::shared_ptr<costmap_2d::Costmap2DROS>& local_costmap)
{
    tf_buffer_ = tf_buffer;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    ros::NodeHandle private_nh("~/" + name);

    std::vector<std::string> vec_layers;
    private_nh.param("layer_names", vec_layers, {std::string("obstacle_layer")});

    for (const std::string& name : vec_layers)
    {
        ROS_INFO_STREAM("ClearCostmapRecovery will clear layer: " << name);
        clearable_layers_.insert(name);
    }
}

void ClearCostmapRecovery::runBehavior()
{
    if (global_costmap_ == nullptr || local_costmap_ == nullptr)
    {
        ROS_ERROR("The costmaps passed to the ClearCostmapRecovery object cannot be NULL. Doing nothing.");
        return;
    }
    ROS_INFO("Clearing costmaps");
    clear(*global_costmap_);
    clear(*local_costmap_);
}

void ClearCostmapRecovery::clear(costmap_2d::Costmap2DROS& costmap)
{
    std::vector<boost::shared_ptr<costmap_2d::Layer>>* plugins = costmap.getLayeredCostmap()->getPlugins();

    for (std::vector<boost::shared_ptr<costmap_2d::Layer>>::iterator pluginp = plugins->begin();
         pluginp != plugins->end(); ++pluginp)
    {
        boost::shared_ptr<costmap_2d::Layer> plugin = *pluginp;
        std::string name = plugin->getName();

        const std::size_t slash = name.rfind('/');
        if (slash != std::string::npos)
        {
            name = name.substr(slash + 1);
        }

        if (clearable_layers_.count(name) != 0)
        {
            ROS_INFO_STREAM("Clearing '" << costmap.getName() << "' layer '" << name << "'");

            boost::shared_ptr<costmap_2d::CostmapLayer> costmap =
                boost::static_pointer_cast<costmap_2d::CostmapLayer>(plugin);
            boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
            std::memset(costmap->getCharMap(), costmap_2d::NO_INFORMATION,
                        costmap->getSizeInCellsX() * costmap->getSizeInCellsY());
        }
    }
}
};
