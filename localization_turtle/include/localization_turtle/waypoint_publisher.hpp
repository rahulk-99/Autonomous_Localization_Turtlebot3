/**
 * @file date.hpp
 * @author2 Rahul Kumar(rahulk99@umd.edu)
 * 
 * @brief ROS2 node responsible for publishing waypoints and guiding bots accordingly.
 * @version 0.1
 * @date 2024-12-03
 * 
 * 
 * @copyright Copyright (c) 2024
 */

#pragma once
#include <rclcpp/rclcpp.hpp>
#include "bot_waypoint_msgs/msg/bot_waypoint.hpp"
#include <std_msgs/msg/bool.hpp>
#include <array>

/**
 * @class WaypointPublisher
 * @brief A ROS2 node which publishes and manages the waypoints for turtlebot.
 * 
 * The WaypointPublisher node is responsible for:
 * Publishing predefined waypoints to the "bot_waypoint" topic.
 * Subscribing to "next_waypoint" notifications to guide the bot to the next waypoint in sequence.
 */

class WaypointPublisher : public rclcpp::Node {
public:

    /**
     * @brief Constructor for the WaypointPublisher class.
     * 
     * Initializes the ROS2 node, sets up publisher and subscriber, and preloads the waypoints with 
     * predefined positions. It is immediately publishing the first waypoint upon startup.
     */
    WaypointPublisher()
        : Node("waypoint_publisher"), index_{0}, ready_to_publish_{true} {
        
        rclcpp::QoS qos(10);                    // keep last 10 messages
        qos.reliable();                         // reliability
        qos.transient_local();                  // Enable transient local durability for late subscriptions

        // Publisher initialization
        publisher_ = this->create_publisher<bot_waypoint_msgs::msg::BotWaypoint>("bot_waypoint", qos);
        
        // Publisher initialization
        subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "next_waypoint", qos, std::bind(&WaypointPublisher::callback_waypoint_publisher, this, std::placeholders::_1));

        // Predfined waypoints with position, orientation and tolerance
        waypoints_[0].waypoint.x = 4.0;
        waypoints_[0].waypoint.y = 4.0;
        waypoints_[0].waypoint.theta= 1.57;
        waypoints_[0].tolerance = bot_waypoint_msgs::msg::BotWaypoint::SMALL; // Starting point
        waypoints_[1].waypoint.x = 4.0;
        waypoints_[1].waypoint.y = -4.0;
        waypoints_[1].waypoint.theta = 3.14;
        waypoints_[1].tolerance = bot_waypoint_msgs::msg::BotWaypoint::MEDIUM;
        waypoints_[2].waypoint.x = -4.0;
        waypoints_[2].waypoint.y = 4.0;
        waypoints_[2].waypoint.theta = -3.14;
        waypoints_[2].tolerance = bot_waypoint_msgs::msg::BotWaypoint::LARGE;
    
        // Initially publishing the first waypoint
        publish_waypoints();
    }

private:
    /**
     * @brief Publishes the current waypoint to the "bot_waypoint" topic.
     * 
     * Ensures that the waypoint is only published if the system is ready, and transitions to the
     * next waypoint after true from the reacher.
     */
    void publish_waypoints();

    /**
     * @brief Callback function for the "next_waypoint" topic.
     * 
     * @param msg Shared pointer to a Bool message telling if the turtlebot is ready for the next waypoint.
     * 
     */
    void callback_waypoint_publisher(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief Publisher for BotWaypoint messages.
     * 
     * This publisher sends `BotWaypoint` messages on the `bot_waypoint` topic. These messages
     * contain the waypoint data (position, orientation, and tolerance) that the robot needs to
     * navigate towards.
     */
    rclcpp::Publisher<bot_waypoint_msgs::msg::BotWaypoint>::SharedPtr publisher_;

    /**
     * @brief Subscribing to next_waypoint.
     * 
     * This subscription listens to the `next_waypoint` topic, receiving a `Bool` message to
     * signal when the robot has completed a waypoint and is ready to proceed to the next one.
     * The subscriber will trigger actions to move to the next waypoint when a new signal is received.
     */
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_;

    /**
     * @brief Array storing the predefined waypoints.
     * 
     * Each waypoint contains position (x, y), orientation (theta), and tolerance levels
     * for bot navigation.
     */
    std::array<bot_waypoint_msgs::msg::BotWaypoint, 3> waypoints_; // Updated to match 4 waypoints, including home state
    
    /**
     * @brief Index of the current waypoint in the sequence.
     * 
     * Incremented after the bot reaches each waypoint.
     */
    unsigned int index_;
    
    /**
     * @brief Flag indicating readiness to publish the next waypoint.
     * 
     * Set to `false` after publishing a waypoint and reset to `true` upon receiving a notification.
     */
    bool ready_to_publish_;
};