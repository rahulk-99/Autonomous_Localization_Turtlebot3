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
#include "rclcpp/rclcpp.hpp"
#include <mage_msgs/msg/parts.hpp>

/**
 * @class PartSubscriber
 * @brief this node subscribes to the "parts" topic and processes the received messages.
 */

class PartSubscriber : public rclcpp::Node {

   public:
   /**
     * @brief constructor for the PartSubscriber class
     *
     */
    PartSubscriber(std::string part_subscriber)
        : rclcpp::Node(part_subscriber) ,parts_published_{false}{

      /**
         * @brief  subscription to the "parts" topic.
         *        The callback function `parts_callback`invoked when receiving the message
         */
      part_subscription_ = this->create_subscription<mage_msgs::msg::Parts>(
      "parts", 10, std::bind(&PartSubscriber::parts_callback, this, std::placeholders::_1));
    
    }

   private:
   /**
     * @brief Flag to check the parts have been published and ignore the data coming after one loop
     */
    bool parts_published_;
    /**
     * @brief Callback function for processing messages from the parts topic.
     *
     */
    void parts_callback(const mage_msgs::msg::Parts::SharedPtr msg);
    /**
     * @brief Subscription to the parts topic.
     */
    rclcpp::Subscription<mage_msgs::msg::Parts>::SharedPtr part_subscription_;
    
};

