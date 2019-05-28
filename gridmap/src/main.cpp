#include <cstdlib>
#include <ros/ros.h>

#include <gridmap/grids/grid_2d.h>
#include <gridmap/grids/occupancy_grid.h>
#include <gridmap/grids/probability_grid.h>
#include <gridmap/layers/layer.h>

#include <gridmap/layered_map.h>

#include <gridmap/layers/obstacle_layer.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <vector>


void updateThread(const double frequency, const std::shared_ptr<gridmap::LayeredMap>& layered_map)
{
    ros::NodeHandle nh;
    ros::Publisher costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("costmap", 1, true);
    nav_msgs::OccupancyGrid grid;

    ros::Rate rate(frequency);
    while (ros::ok())
    {
        if (costmap_pub.getNumSubscribers() != 0)
        {
            layered_map->update();
            auto map_data = layered_map->map();

            {
                auto lock = map_data->grid.getLock();
                const double resolution = map_data->grid.dimensions().resolution();
                const int down_sample = 4;

                grid.header.frame_id = "map";
                grid.header.stamp = ros::Time::now();
                grid.info.resolution = resolution * down_sample;

                grid.info.width = map_data->grid.dimensions().size().x() / down_sample;
                grid.info.height = map_data->grid.dimensions().size().y() / down_sample;

                grid.info.origin.position.x = map_data->grid.dimensions().origin().x();
                grid.info.origin.position.y = map_data->grid.dimensions().origin().y();
                grid.info.origin.orientation.w = 1.0;

                grid.data.resize(grid.info.width * grid.info.height);
                std::fill(grid.data.begin(), grid.data.end(), 0);
                int index = 0;
                for (int i = 0; i < map_data->grid.dimensions().size().x(); ++i)
                {
                    for (int j = 0; j < map_data->grid.dimensions().size().y(); ++j)
                    {
                        const int sub_i = i / down_sample;
                        const int sub_j = j / down_sample;
                        const int sub_index = sub_i * grid.info.width + sub_j;
                        if (map_data->grid.cells()[index] > 0)
                            grid.data[sub_index] = 100;
                        ++index;
                    }
                }
            }
            costmap_pub.publish(grid);
        }

        rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gridmap_test");

    const std::string global_frame = "map";

    auto tf_buffer = std::make_shared<tf2_ros::Buffer>();
    tf2_ros::TransformListener tf_listener(*tf_buffer);

    /*
        gridmap::LaserData laser_data;
        laser_data.initialize("laser_data", global_frame, parameters, map_data, tf_buffer);

        std::vector<std::shared_ptr<gridmap::RangeData>> rds;
        for (int i = 1; i <= 16; ++i)
        {
            auto rd = std::make_shared<gridmap::RangeData>();
            rd->initialize("sonar_" + std::to_string(i) + "_link", global_frame, parameters, map_data, tf_buffer);
            rds.push_back(rd);
        }

        const double map_publish_frequency = 1.0;
        gridmap::MapPublisher map_publisher(map_publish_frequency, map_data, global_frame);
        */

    std::vector<std::shared_ptr<gridmap::Layer>> layers;

    // add laser data
    if (false)
    {
        auto laser_layer = std::make_shared<gridmap::ObstacleLayer>();
        XmlRpc::XmlRpcValue obs_parameters;
        XmlRpc::XmlRpcValue laser_source;
        laser_source["name"] = "laser_data";
        laser_source["type"] = "gridmap::LaserData";
        obs_parameters["data_sources"][0] = laser_source;
        laser_layer->initialize("laser_layer", global_frame, obs_parameters, tf_buffer);
        layers.push_back(laser_layer);
    }

    // add sonar data
    {
        auto sonar_layer = std::make_shared<gridmap::ObstacleLayer>();
        XmlRpc::XmlRpcValue obs_parameters;

        for (int i = 0; i <= 12; ++i)
        {
            XmlRpc::XmlRpcValue source;
            source["name"] = "sonar_" + std::to_string(i);
            source["type"] = "gridmap::RangeData";
            obs_parameters["data_sources"][i] = source;
        }

        sonar_layer->initialize("sonar_layer", global_frame, obs_parameters, tf_buffer);
        layers.push_back(sonar_layer);
    }

    auto base_map_layer = std::make_shared<gridmap::BaseMapLayer>();
    XmlRpc::XmlRpcValue base_map_params;
    base_map_layer->initialize("base_map_layer", global_frame, base_map_params, tf_buffer);

    auto layered_map = std::make_shared<gridmap::LayeredMap>(base_map_layer, layers);

    auto map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", ros::Duration(10.0));
    if (!map)
        return EXIT_FAILURE;

    hd_map::Map hd_map;
    hd_map.map_info = map->info;

    layered_map->setMap(hd_map, *map);

    ROS_INFO_STREAM("map size: " << layered_map->map()->grid.dimensions().size().transpose());

    //    std::thread update_thread(updateThread, 1.0, layered_map);

    ros::spin();

    //    update_thread.join();

    return EXIT_SUCCESS;
}
