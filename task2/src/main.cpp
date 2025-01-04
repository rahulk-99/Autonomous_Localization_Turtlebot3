#include <part_subscriber_node.hpp>
#include <camera_broadcaster.hpp>
#include <bot_navigate.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Creating nodes
    auto camera_subscriber = std::make_shared<CameraSubscriber>("camera_subscriber");
    auto part_subscriber = std::make_shared<PartSubscriber>("part_subscriber");
    auto bot_navigate = std::make_shared<BotNavigate>();



    // Creating a multi-threaded executor
    rclcpp::executors::SingleThreadedExecutor executor;

    // Adding both nodes to the executor
    executor.add_node(part_subscriber);
    executor.add_node(camera_subscriber);
    executor.add_node(bot_navigate);

    // Spin the executor
    executor.spin();
    rclcpp::shutdown();
}