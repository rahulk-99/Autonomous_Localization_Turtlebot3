/**
 * @file date.hpp
 * @author2 Rahul Kumar(rahulk99@umd.edu)
 * 
 * @brief ROS2 node responsible for publishing waypoints and guiding bots accordingly.
 * This class subscribes to odometry and waypoint topics, guiding the turtlebot
 * @version 0.1
 * @date 2024-12-03
 * 
 * 
 * @copyright Copyright (c) 2024
 */


#pragma once
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include "bot_waypoint_msgs/msg/bot_waypoint.hpp"
#include <std_msgs/msg/bool.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

/**
 * @class WaypointReacher
 * @brief A ROS2 node for reaching waypoints and managing bot motion.
 * 
 * This class handles waypoint navigation by subscribing to bot waypoint messages and
 * odometry data.
 */
class WaypointReacher : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the WaypointReacher class.
     * 
     * Initializes subscribers and publishers.
     */
    WaypointReacher()
        : Node("waypoint_reacher"), waypoint_reached_{false}, index__waypoint_{0} {
        
        rclcpp::QoS qos(10);            // keep last 10 messages
        qos.reliable();                 // reliability
        qos.transient_local();          // Enable transient local durability for late subscriptions

        // Subscribing bot_waypoint topic to get waypoint information
        reacher_subscription_ = this->create_subscription<bot_waypoint_msgs::msg::BotWaypoint>(
            "bot_waypoint", qos, std::bind(&WaypointReacher::waypoint_callback_reacher, this, std::placeholders::_1));
        
        // Publisher for velocity commands
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscriber for odometry
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&WaypointReacher::odom_callback, this, std::placeholders::_1));

        // Publishing to next_waypoint topic for next waypoint
        next_waypoint_publisher_ = this->create_publisher<std_msgs::msg::Bool>("next_waypoint", qos);

        // Controller gains and tolerances
        kp_distance_ = 0.3;
        kp_angle_ = 0.3;

        RCLCPP_INFO_STREAM(this->get_logger(), "== waypoint_reacher node Started ==");
    }

private:
    /**
     * @brief Callback function for odometry data.
     * 
     * Updates the robot's current position and orientation.
     * 
     * @param msg Shared pointer to the odometry message.
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Callback function for receiving waypoint information.
     * 
     * Sets the goal position, orientation, and tolerance values.
     * 
     * @param msg BotWaypoint message containing the waypoint details.
     */
    void waypoint_callback_reacher(const bot_waypoint_msgs::msg::BotWaypoint msg);

    /**
     * @brief Executes the control loop for guiding the robot.
     * 
     * Computes the linear and angular velocities needed to reach the waypoint
     * and determines if the waypoint has been reached.
     */
    void control_loop();

    /**
     * @brief Publishes velocity commands to the robot.
     * 
     * @param linear The linear velocity in m/s.
     * @param angular The angular velocity in rad/s.
     */
    void publish_velocity(double linear, double angular);

    /**
     * @brief Normalizes an angle to the range [-pi, pi].
     * 
     * @param angle The input angle in radians.
     * @return double The normalized angle in radians.
     */
    double normalize_angle(double angle);

    /**
     * @brief Subscription for waypoint data.
     * 
     * This subscription listens to the `bot_waypoint` topic, which provides
     * the waypoint data (position and orientation) that the robot needs to navigate to.
     */
    rclcpp::Subscription<bot_waypoint_msgs::msg::BotWaypoint>::SharedPtr reacher_subscription_;

    /**
     * @brief Publisher for velocity commands.
     * 
     * This publisher sends velocity commands (`Twist` messages) to the robot to control its
     * linear and angular velocities in order to reach the specified waypoint.
     */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

    /**
     * @brief Subscription for odometry data.
     * 
     * This subscription listens to the `odom` topic and receives odometry information to
     * track the robot's current position and orientation in the environment.
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    /**
     * @brief Publisher to signal the next waypoint.
     * 
     * This publisher sends a `Bool` message on the `next_waypoint` topic to indicate when the
     * robot has reached a waypoint and is ready to proceed to the next one.
     */
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr next_waypoint_publisher_;


    double goal_x_;                 // Goal x position in meters.
    double goal_y_;                 // Goal y position in meters.
    double goal_theta_;             // Goal orientation (theta) in radians.
    double kp_distance_;            // Proportional gain for distance control.
    double kp_angle_;               // Proportional gain for angle control
    double tolerance_value_;        // Tolerance value to determine if waypoint is reached.
    bool waypoint_reached_;         // Flag to indicate if the waypoint has been reached.
    // std::array<bot_waypoint_msgs::msg::BotWaypoint, 3> waypoints_;   // Array to hold waypoint data.
    unsigned int index__waypoint_;  // Index of the current waypoint.


    // Current state of the robot
    double current_x_;              // Current x position in meters.
    double current_y_;              // Current y position in meters.
    double current_theta_;          // Current orientation (theta) in radians.
    double roll_;                   // Current roll orientation in radians.
    double pitch_;                  // Current pitch orientation in radians.
};