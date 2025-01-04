
#include "waypoint_publisher.hpp"

// Method to publish waypoints
void WaypointPublisher::publish_waypoints() {
    // Check if there are still waypoints to publish and if it's ready to publish
    if (index_ < 3 && ready_to_publish_) {
        // Log the current waypoint being published
        RCLCPP_INFO(this->get_logger(), "Publishing waypoint %u: x=%.2f, y=%.2f, θ=%.2f, Tolerance: %d",
                    index_,
                    waypoints_[index_].waypoint.x,
                    waypoints_[index_].waypoint.y,
                    waypoints_[index_].waypoint.theta,
                    waypoints_[index_].tolerance);
        // Publish the current waypoint to the bot_waypoint topic
        publisher_->publish(waypoints_[index_]);            //it will publish 1st waypoint whihc is starting state because ready_to_publish_ is initialised with true
        ready_to_publish_ = false;                          // Making ready to publish false so that it will check for true condition 
                                                            // for publishing next waypoints through callback_waypoint
    }
}

// Callback method for subscribing to next_waypoint messages
void WaypointPublisher::callback_waypoint_publisher(const std_msgs::msg::Bool::SharedPtr msg) {
    // Check if the message published by reacher to next_waybot is true and there are still waypoints to publish
    if (msg->data && index_ < 3) {
        ++index_;
        ready_to_publish_ = true;
        // Call the method to publish the next waypoint
        publish_waypoints();
    } else if (index_ >= 3) {
         // Log that all waypoints have been published and shutdown the node
        RCLCPP_INFO(this->get_logger(), "All waypoints published. Shutting down.");
        rclcpp::shutdown();
    }
}




