#include <autonomy/autonomy.h>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    // Create ROS node
    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.start_parameter_services(false);

    auto node = std::make_shared<autonomy::Autonomy>("autonomy", node_options);
    node->init(); // create services

    // Create an executor to spin the node
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    executor.spin();

    // Shutdown
    rclcpp::shutdown();
  } catch (const std::exception &e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger(""), "Exception: " << e.what());
    std::cout << "Exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
