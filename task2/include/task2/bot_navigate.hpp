/**
 * @file date.hpp
 * @author1 Rahul Kumar(rahulk99@umd.edu)
 * 
 * @brief ROS2 node responsible for extracting part's pose wrt camera frame and then transforming it into the world frame.
 * @version 0.1
 * @date 2024-12-10
 * 
 * @copyright Copyright (c) 2024
 */
#pragma once
#include <geometry_msgs/msg/twist.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @class BotNavigate
 * @brief This class is for navigating a robot by subscribing to target locations
 *        and odometry data, and publishing velocity commands using a proportional control loop.
 */
class BotNavigate : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the BotNavigate class. 
     */
    BotNavigate()
        : Node("bot_navigate") {
        
        // Setting QoS for both the publisher and subscriber
        rclcpp::QoS qos(10);               // Keep last 10 messages
        qos.reliable();                    // Reliable communication
        qos.transient_local();             // Transient local durability
        
        // Initialize the target publisher to publish to target_part topic
        target_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("target_parts", qos);

        // Hard-code a test target location (for testing)
        geometry_msgs::msg::Twist target_msg;
        target_msg.linear.x = -4.63;  // x-coordinate
        target_msg.linear.y = -0.37;  // y-coordinate
        target_msg.angular.z = 1.00;  // orientation in radians

        // Publish the target location
        target_publisher_->publish(target_msg);
        RCLCPP_INFO(this->get_logger(), "Publishing target: x=%.2f, y=%.2f, theta=%.2f", 
                target_msg.linear.x, target_msg.linear.y, target_msg.angular.z);

        // Subscribe to target location topic named "target_parts"
        navigate_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "target_parts", qos, std::bind(&BotNavigate::bot_navigate_callback, this, std::placeholders::_1));
        
        // Publisher for velocity commands
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscriber for odometry
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&BotNavigate::odom_callback, this, std::placeholders::_1));

        // Controller gains and tolerances
        kp_distance_ = 0.3;
        kp_angle_ = 0.3;

        RCLCPP_INFO_STREAM(this->get_logger(), "== bot_navigate node Started ==");
    }

private:
    /**
     * @brief Callback function to process odometry messages.
     * @param msg Odometry message containing current position and orientation of the robot.
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Callback function to process target locations.
     * @param msg Twist message containing target linear and angular velocities.
     */
    void bot_navigate_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief Proportional control loop for navigation. This function calculates the required velocity commands.
     */
    void control_loop();

    /**
     * @brief Publishes velocity commands to the robot.
     * @param linear Linear velocity in meters per second.
     * @param angular Angular velocity in radians per second.
     */
    void publish_velocity(double linear, double angular);

    /**
     * @brief Normalizes angles to the range [-pi, pi].
     * @param angle The angle in radians to be normalized.
     * @return The normalized angle.
     */
    double normalize_angle(double angle);


    /**
     * @brief Subscription to target location updates.
     */
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr navigate_subscription_;

    /**
     * @brief Publisher for velocity commands to the robot.
     */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

    /**
     * @brief Subscription to odometry data.
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    /**
     * @brief Publisher for target location information.
     */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr target_publisher_;

    /**
     * @brief Proportional gain for distance control.
     */
    double kp_distance_; 

    /**
     * @brief Proportional gain for angle control.
     */
    double kp_angle_;

    /**
     * @brief Current x position in meters.
     */
    double current_x_;

    /**
     * @brief Current y position in meters.
     */
    double current_y_;

    /**
     * @brief Current orientation (theta) in radians.
     */
    double current_theta_;

    /**
     * @brief Goal x position in meters.
     */
    double goal_x_;

    /**
     * @brief Goal y position in meters.
     */
    double goal_y_;

    /**
     * @brief Goal orientation (theta) in radians.
     */
    double goal_theta_;

    /**
     * @brief Index of the current waypoint.
     */
    unsigned int index_waypoint_;

    /**
     * @brief Current roll orientation in radians.
     */
    double roll_;

    /**
     * @brief Current pitch orientation in radians.
     */
    double pitch_;
};
