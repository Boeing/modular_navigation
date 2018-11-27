#include <modular_move_base/move_base.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_base");

    move_base::MoveBase move_base;

    ros::spin();

    return EXIT_SUCCESS;
}
