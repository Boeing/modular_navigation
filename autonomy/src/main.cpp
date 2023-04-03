#include <autonomy/autonomy.h>

int main(int argc, char** argv)
{
    // Now the "autonomy" name is given in the Autonomy constructor
    rclcpp::init(argc, argv);  //, "autonomy");
    try
    {
        // rclcpp::spin defaults to SingleThreadedExecution, meaning
        // only one callback can be executed at a time.
        auto autonomy_node = std::make_shared<autonomy::Autonomy>("autonomy");

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(autonomy_node);
        executor.spin();
    }
    catch (const std::exception& e)
    {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger(""), "Exception: " << e.what());
        std::cout << "Exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
