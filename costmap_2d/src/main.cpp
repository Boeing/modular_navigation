#include <cstdlib>
#include <ros/ros.h>

#include <costmap_2d/map_data.h>
#include <costmap_2d/map_publisher.h>
#include <costmap_2d/plugins/base_map_data.h>
#include <costmap_2d/plugins/laser_data.h>
#include <costmap_2d/map_publisher.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <vector>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_base");

    const std::string global_frame = "map";

    auto tf_buffer = std::make_shared<tf2_ros::Buffer>();
    tf2_ros::TransformListener tf_listener(*tf_buffer);

    const double clamping_thres_min = 0.1192;
    const double clamping_thres_max = 0.971;
    const double occ_prob_thres = 0.8;

    auto map_data = std::make_shared<costmap_2d::MapData>(clamping_thres_min, clamping_thres_max, occ_prob_thres);

    costmap_2d::BaseMapData base_map;
    XmlRpc::XmlRpcValue parameters;
    base_map.initialize("base_map", global_frame, parameters, map_data, tf_buffer);

    costmap_2d::LaserData laser_data;
    laser_data.initialize("laser_data", global_frame, parameters, map_data, tf_buffer);

    const double map_publish_frequency = 1.0;
    costmap_2d::MapPublisher map_publisher(map_publish_frequency, map_data, global_frame);

    ros::spin();

    return EXIT_SUCCESS;
}
