#include <modular_move_base/move_base.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_base");

    try
    {
        move_base::MoveBase move_base;
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
