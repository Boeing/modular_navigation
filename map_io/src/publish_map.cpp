#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <libgen.h>

#include <boost/filesystem/path.hpp>

#include <map_io/image_loader.h>

#include <nav_msgs/MapMetaData.h>

#include <ros/console.h>
#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

template <typename T> void operator>>(const YAML::Node& node, T& i)
{
    i = node.as<T>();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_map");
    if (argc != 2)
    {
        ROS_ERROR("%s", "\nUSAGE: map_publisher <map.yaml>\n");
        return EXIT_FAILURE;
    }

    const std::string file_name(argv[1]);

    try
    {
        std::ifstream fin(file_name.c_str());
        if (fin.fail())
        {
            ROS_ERROR_STREAM("Failed to open: " << file_name);
            return EXIT_FAILURE;
        }

        YAML::Node doc = YAML::Load(fin);

        double resolution;
        try
        {
            doc["resolution"] >> resolution;
        }
        catch (YAML::InvalidScalar)
        {
            ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
            return EXIT_FAILURE;
        }

        int negate;
        try
        {
            doc["negate"] >> negate;
        }
        catch (YAML::InvalidScalar)
        {
            ROS_ERROR("The map does not contain a negate tag or it is invalid.");
            return EXIT_FAILURE;
        }

        double occ_th;
        try
        {
            doc["occupied_thresh"] >> occ_th;
        }
        catch (YAML::InvalidScalar)
        {
            ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
            return EXIT_FAILURE;
        }

        double free_th;
        try
        {
            doc["free_thresh"] >> free_th;
        }
        catch (YAML::InvalidScalar)
        {
            ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
            return EXIT_FAILURE;
        }

        MapMode mode = TRINARY;
        try
        {
            std::string modeS = "";
            doc["mode"] >> modeS;

            if (modeS == "trinary")
                mode = TRINARY;
            else if (modeS == "scale")
                mode = SCALE;
            else if (modeS == "raw")
                mode = RAW;
            else
            {
                ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
                return EXIT_FAILURE;
            }
        }
        catch (YAML::Exception)
        {
            ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
            mode = TRINARY;
        }

        double origin[3];
        try
        {
            doc["origin"][0] >> origin[0];
            doc["origin"][1] >> origin[1];
            doc["origin"][2] >> origin[2];
        }
        catch (YAML::InvalidScalar)
        {
            ROS_ERROR("The map does not contain an origin tag or it is invalid.");
            return EXIT_FAILURE;
        }

        std::string map_file_name;
        try
        {
            doc["image"] >> map_file_name;
            if (map_file_name.size() == 0)
            {
                ROS_ERROR("The image tag cannot be an empty string");
                exit(-1);
            }
            if (map_file_name[0] != '/')
            {
                boost::filesystem::path p(file_name);
                map_file_name = p.parent_path().string() + "/" + map_file_name;
            }
        }
        catch (YAML::InvalidScalar)
        {
            ROS_ERROR("The map does not contain an image tag or it is invalid.");
            return EXIT_FAILURE;
        }

        ROS_INFO_STREAM("Loading map from image: " << map_file_name);
        nav_msgs::OccupancyGrid map = map_io::loadMapFromFile(map_file_name, resolution, negate, occ_th, free_th,
                                                              origin[0], origin[1], origin[2], mode);

        std::string frame_id = "map";
        ros::NodeHandle nh("/");

        ros::Time::waitForValid();

        map.info.map_load_time = ros::Time::now();
        map.header.frame_id = frame_id;
        map.header.stamp = ros::Time::now();

        ROS_INFO_STREAM("Read map: " << map_file_name);
        ROS_INFO_STREAM("size_x: " << map.info.width);
        ROS_INFO_STREAM("size_y: " << map.info.height);
        ROS_INFO_STREAM("resolution: " << map.info.resolution);

        ros::Publisher metadata_pub = nh.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
        metadata_pub.publish(map.info);

        ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
        map_pub.publish(map);
    }
    catch (std::runtime_error& e)
    {
        ROS_ERROR_STREAM("Exception: " << e.what());
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
