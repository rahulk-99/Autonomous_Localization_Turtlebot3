#include "waypoint_reacher.hpp"
// Callback method for receiving waypoint data
void WaypointReacher::waypoint_callback_reacher(const bot_waypoint_msgs::msg::BotWaypoint msg) {
    // Extract the goal position and orientation x, y and theta from the received message
    goal_x_ = msg.waypoint.x;
    goal_y_ = msg.waypoint.y;
    goal_theta_ = msg.waypoint.theta;
    tolerance_value_ = msg.tolerance/10.0;              // Convert tolerance to correct scale
    index__waypoint_ = index__waypoint_ + 1;            // Increment the waypoint index

    // ogging next waypoint to be published
    RCLCPP_INFO(this->get_logger(), "next_waypoint message arrived: x=%.2f, y=%.2f, theta=%.2f",
                msg.waypoint.x, msg.waypoint.y, msg.waypoint.theta);

}

// Callback method for receiving odometry data
void WaypointReacher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract the current position from the odometry message
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    // Extract current orientation as yaw (theta)
    tf2::Quaternion quaternion;
    tf2::fromMsg(msg->pose.pose.orientation, quaternion);
    tf2::Matrix3x3(quaternion).getRPY(roll_, pitch_, current_theta_);

    // Call the control loop to update the robot's movement
    control_loop();

}

// Control loop to navigate the robot towards the goal waypoint
void WaypointReacher::control_loop() {
    // Calculate the distance error from the current position to the goal position
    double distance_error = std::sqrt(std::pow(goal_x_ - current_x_, 2) + std::pow(goal_y_ - current_y_, 2));
    // Calculate the desired angle to the goal position
    double desired_angle = std::atan2(goal_y_ - current_y_, goal_x_ - current_x_);
    // Calculate the angle error between the robot's current orientation and the desired angle
    double angle_error = normalize_angle(desired_angle - current_theta_);
    // Calculate the error in final orientatio
    double final_orientation_error = normalize_angle(goal_theta_ - current_theta_);

    // Phase 1: Navigate to goal positionwaypoints_[index_].t
    if (distance_error > tolerance_value_) {
        // Calculate the linear and angular velocities based on the errors
        double linear_velocity = kp_distance_ * distance_error;
        double angular_velocity = kp_angle_ * angle_error;

        // Publish the velocity commands to the robot
        publish_velocity(linear_velocity, angular_velocity);
        // Log the current movement towards the waypoint
        RCLCPP_INFO(this->get_logger(), "Heading towards waypoint no: %u with linear velocit %.2f and distance error %.2f",
                index__waypoint_, linear_velocity, distance_error);
    }
    // Phase 2: Adjust final orientation
    else if (std::abs(final_orientation_error) > 0.017) {
        // Stop linear movement and adjust angular velocity
        double angular_velocity = kp_angle_ * final_orientation_error;
        publish_velocity(0.0, angular_velocity);
        // Log the final orientation correction process
        RCLCPP_INFO(this->get_logger(), "Final orientation correction with angular velocity %.2f and final orientation error %.2f",
                angular_velocity, final_orientation_error);
    }
    // Goal reached with correct orientation
    else {
        publish_velocity(0.0, 0.0);         // Stop all movement 
        RCLCPP_INFO(this->get_logger(), "Goal reached with correct orientation!");
        waypoint_reached_ = true;           // Waypoint reached at this time and flag is chaged to true which will be published to topic as follow

        auto msg = std_msgs::msg::Bool();
        msg.data = waypoint_reached_;
        next_waypoint_publisher_->publish(msg);

    }
}

// Method to publish velocity commands to the robot
void WaypointReacher::publish_velocity(double linear, double angular) {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear;
    msg.angular.z = angular;
    velocity_publisher_->publish(msg);
}

// Method to normalize an angle to the range [-pi, pi]
double WaypointReacher::normalize_angle(double angle) {
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}
