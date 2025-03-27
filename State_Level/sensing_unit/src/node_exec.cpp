#include "sensing_unit/sensing_unit.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto node = std::make_shared<SensingUnit>(rclcpp::NodeOptions());
    exec->add_node(node);
    exec->spin();
    rclcpp::shutdown();
    return 0;
}
