#include <autonomy/autonomy.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "autonomy");

    try
    {
        autonomy::Autonomy an;
        ros::spin();
    }
    catch (const std::exception& e)
    {
        ROS_FATAL_STREAM("Exception: " << e.what());
        std::cout << "Exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
