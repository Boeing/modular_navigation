#include <cstdlib>
#include <ros/ros.h>

#include <gridmap/map_data.h>
#include <gridmap/map_publisher.h>
#include <gridmap/plugins/base_map_data.h>
#include <gridmap/plugins/laser_data.h>
#include <gridmap/plugins/range_data.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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

    auto map_data = std::make_shared<gridmap::MapData>(clamping_thres_min, clamping_thres_max, occ_prob_thres);

    gridmap::BaseMapData base_map;
    XmlRpc::XmlRpcValue parameters;
    base_map.initialize("base_map", global_frame, parameters, map_data, tf_buffer);

    gridmap::LaserData laser_data;
    laser_data.initialize("laser_data", global_frame, parameters, map_data, tf_buffer);

//    gridmap::RangeData range_data_0;
//    range_data_0.initialize("range_0", global_frame, parameters, map_data, tf_buffer);

//    gridmap::RangeData range_data_1;
//    range_data_1.initialize("range_1", global_frame, parameters, map_data, tf_buffer);

//    gridmap::RangeData range_data_2;
//    range_data_2.initialize("range_2", global_frame, parameters, map_data, tf_buffer);

//    gridmap::RangeData range_data_3;
//    range_data_3.initialize("range_3", global_frame, parameters, map_data, tf_buffer);

//    gridmap::RangeData range_data_4;
//    range_data_4.initialize("range_4", global_frame, parameters, map_data, tf_buffer);

//    gridmap::RangeData range_data_5;
//    range_data_5.initialize("range_5", global_frame, parameters, map_data, tf_buffer);

//    gridmap::RangeData range_data_6;
//    range_data_6.initialize("range_6", global_frame, parameters, map_data, tf_buffer);

//    gridmap::RangeData range_data_7;
//    range_data_7.initialize("range_7", global_frame, parameters, map_data, tf_buffer);

//    gridmap::RangeData range_data_8;
//    range_data_8.initialize("range_8", global_frame, parameters, map_data, tf_buffer);

//    gridmap::RangeData range_data_9;
//    range_data_9.initialize("range_9", global_frame, parameters, map_data, tf_buffer);

//    gridmap::RangeData range_data_10;
//    range_data_10.initialize("range_10", global_frame, parameters, map_data, tf_buffer);

//    gridmap::RangeData range_data_11;
//    range_data_11.initialize("range_11", global_frame, parameters, map_data, tf_buffer);

//    gridmap::RangeData range_data_12;
//    range_data_12.initialize("range_12", global_frame, parameters, map_data, tf_buffer);

    const double map_publish_frequency = 1.0;
    gridmap::MapPublisher map_publisher(map_publish_frequency, map_data, global_frame);

    ros::spin();

    return EXIT_SUCCESS;
}
