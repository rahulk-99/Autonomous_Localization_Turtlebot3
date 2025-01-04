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
#include "rclcpp/rclcpp.hpp"
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
#include <mage_msgs/msg/parts.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <mage_msgs/msg/part.hpp>

/**
 * @class CameraSubscriber
 * @brief This class subscribes to all the cameras available in our Gazebo world through the advanced logical camera image.
 */
class CameraSubscriber : public rclcpp::Node {
   public:
    /**
     * @brief Constructor for the CameraSubscriber class which initializes part_received flags to false.
     * @param camera_subscriber The name of the node.
     */
    CameraSubscriber(std::string camera_subscriber)
        : rclcpp::Node(camera_subscriber),
          parts_received1_{false}, parts_received2_{false}, parts_received3_{false},
          parts_received4_{false}, parts_received5_{false}, parts_received6_{false},
          parts_received7_{false}, parts_received8_{false} {
         
        // Subscriptions for each camera topic
        camera1_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "/mage/camera1/image", rclcpp::SensorDataQoS(),
            std::bind(&CameraSubscriber::camera1_callback, this, std::placeholders::_1));

        camera2_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "/mage/camera2/image", rclcpp::SensorDataQoS(),
            std::bind(&CameraSubscriber::camera2_callback, this, std::placeholders::_1));

        camera3_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "/mage/camera3/image", rclcpp::SensorDataQoS(),
            std::bind(&CameraSubscriber::camera3_callback, this, std::placeholders::_1));

        camera4_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "/mage/camera4/image", rclcpp::SensorDataQoS(),
            std::bind(&CameraSubscriber::camera4_callback, this, std::placeholders::_1));

        camera5_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "/mage/camera5/image", rclcpp::SensorDataQoS(),
            std::bind(&CameraSubscriber::camera5_callback, this, std::placeholders::_1));

        camera6_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "/mage/camera6/image", rclcpp::SensorDataQoS(),
            std::bind(&CameraSubscriber::camera6_callback, this, std::placeholders::_1));

        camera7_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "/mage/camera7/image", rclcpp::SensorDataQoS(),
            std::bind(&CameraSubscriber::camera7_callback, this, std::placeholders::_1));

        camera8_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "/mage/camera8/image", rclcpp::SensorDataQoS(),
            std::bind(&CameraSubscriber::camera8_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Broadcaster started");
    }

   private:
       private:

    /**
     * @brief Flag indicating for 1st camera.
     */
    bool parts_received1_;

    /**
     * @brief Flag indicating for 2nd camera.
     */
    bool parts_received2_;

    /**
     * @brief Flag indicating for 3rd camera.
     */
    bool parts_received3_;

    /**
     * @brief Flag indicating for 4th camera
     */
    bool parts_received4_;

    /**
     * @brief Flag indicating for 5th camera.
     */
    bool parts_received5_;

    /**
     * @brief Flag indicating for 6th camera.
     */
    bool parts_received6_;

    /**
     * @brief Flag indicating for 7th camera.
     */
    bool parts_received7_;

    /**
     * @brief Flag indicating for 8th camera.
     */
    bool parts_received8_;

    
    /**
     * @brief       Callback function for receiving camera1 messages.
     * @param msg The message from the camera1 topic.
     */
    void camera1_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Shared pointer for the camera1 subscription.
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera1_sub_;
    
    /**
     * @brief   Callback function for receiving camera2 messages.
     * @param msg The message from the camera2 topic.
     */
    void camera2_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Shared pointer for the camera2 subscription.
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera2_sub_;
    
    /**
     * @brief       Callback function for receiving camera3 messages.
     * @param msg   The message from the camera3 topic.
     */
    void camera3_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Shared pointer for the camera3 subscription.
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera3_sub_;
    
    /**
     * @brief   Callback function for receiving camera4 messages.
     * @param msg The message from the camera4 topic.
     */
    void camera4_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Shared pointer for the camera4 subscription.
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera4_sub_;
    
    /**
     * @brief       Callback function for receiving camera5 messages.
     * @param msg The message from the camera5 topic.
     */
    void camera5_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Shared pointer for the camera5 subscription.
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera5_sub_;
    
    /**
     * @brief       Callback function for receiving camera6 messages.
     * @param msg The message from the camera6 topic.
     */
    void camera6_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Shared pointer for the camera6 subscription.
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera6_sub_;
    
    /**
     * @brief Callback function for receiving camera7 messages.
     * @param msg The message from the camera7 topic.
     */
    void camera7_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Shared pointer for the camera7 subscription.
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera7_sub_;
    
    /**
     * @brief Callback function for receiving camera8 messages.
     * @param msg The message from the camera8 topic.
     */
    void camera8_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Shared pointer for the camera8 subscription.
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera8_sub_;
};
