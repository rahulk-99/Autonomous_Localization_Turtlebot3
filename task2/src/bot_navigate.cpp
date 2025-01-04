#include "bot_navigate.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

// Callback to process target locations
void BotNavigate::bot_navigate_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    goal_x_ = msg->linear.x;
    goal_y_ = msg->linear.y;
    goal_theta_ = msg->angular.z;

    RCLCPP_INFO(this->get_logger(), "Received target: x=%.2f, y=%.2f, theta=%.2f", goal_x_, goal_y_, goal_theta_);
}

// Callback to process odometry messages
void BotNavigate::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    tf2::Quaternion quaternion;
    tf2::fromMsg(msg->pose.pose.orientation, quaternion);
    tf2::Matrix3x3(quaternion).getRPY(roll_, pitch_, current_theta_);

    control_loop();
}

// Proportional control loop for navigation
void BotNavigate::control_loop() {
    // Calculate the distance error from the current position to the goal position
    double distance_error = std::sqrt(std::pow(goal_x_ - current_x_, 2) + std::pow(goal_y_ - current_y_, 2));
    
    // Calculate the desired angle to the goal position
    double desired_angle = std::atan2(goal_y_ - current_y_, goal_x_ - current_x_);
    
    // Calculate the angle error between the robot's current orientation and the desired angle
    double angle_error = normalize_angle(desired_angle - current_theta_);

    // Calculate the error in final orientation
    double final_orientation_error = normalize_angle(goal_theta_ - current_theta_);

    // Phase 1: Navigate to goal position (move towards goal)
    if (distance_error > 0.1 || std::abs(final_orientation_error) > 0.017) {  // Move until the robot is within tolerance
        // Calculate the linear velocity based on the distance error
        double linear_velocity = kp_distance_ * distance_error;
        
        // Calculate the angular velocity based on the angle error
        double angular_velocity = kp_angle_ * angle_error;

        // Publish the velocity commands to the robot
        publish_velocity(linear_velocity, angular_velocity);
        
        // Log the current movement towards the target
        RCLCPP_INFO(this->get_logger(), "Heading towards target with linear velocity %.2f, angular velocity %.2f, and distance error %.2f",
                    linear_velocity, angular_velocity, distance_error);
    }
 
    else {
        publish_velocity(0.0, 0.0);  // Stop all movement
        RCLCPP_INFO(this->get_logger(), "Goal reached with correct orientation!");

    }
}



// Publish velocity commands
void BotNavigate::publish_velocity(double linear, double angular) {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear;
    msg.angular.z = angular;
    velocity_publisher_->publish(msg);
}

// Normalize angles to the range [-pi, pi]
double BotNavigate::normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}


