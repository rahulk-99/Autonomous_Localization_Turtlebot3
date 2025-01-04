#include "rclcpp/rclcpp.hpp"
#include "waypoint_publisher.hpp"
#include "waypoint_reacher.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Creating both nodes
    auto waypoint_publisher = std::make_shared<WaypointPublisher>();
    auto waypoint_reacher = std::make_shared<WaypointReacher>();

    // Creating a multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;

    // Adding both nodes to the executor
    executor.add_node(waypoint_reacher);
    executor.add_node(waypoint_publisher);

    // Spin the executor
    executor.spin();
    rclcpp::shutdown();
}