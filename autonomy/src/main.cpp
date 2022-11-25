#include <autonomy/autonomy.h>

int main(int argc, char** argv)
{
    // Now the "autonomy" name is given in the Autonomy constructor
    rclcpp::init(argc, argv);  //, "autonomy");
    try
    {
        // autonomy::Autonomy an;
        rclcpp::spin(std::make_shared<autonomy::Autonomy>());
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger(""), "Exception: " << e.what());
        std::cout << "Exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
