#include "rclcpp/rclcpp.hpp"
#include <mage_msgs/msg/part.hpp>
#include <mage_msgs/msg/parts.hpp>
#include <part_subscriber_node.hpp>
#include <part_string.hpp>


void PartSubscriber::parts_callback(const mage_msgs::msg::Parts::SharedPtr msg){

    // Check if parts have already been processed to avoid rmultiple iteration of same data
    if(!parts_published_){
        for(auto &part : msg->parts){
            
          //the string representations of the part's color and type
          std::string color_ = part_color(part.color);
          std::string type_ = part_type(part.type);

          //condition for unkown parts that are not available in gazebo world
          if(color_ == "unkown" || type_ =="unkown" ){

            RCLCPP_INFO(this->get_logger(),"part is not available in the environment. It will be skipped: %s, %s",color_.c_str(), type_.c_str() );

          }

        RCLCPP_INFO(this->get_logger(),"Received part color: %s, part type: %s",color_.c_str(), type_.c_str());
         // Set the flag as parts have been published one time
        parts_published_ = true;
      }
      RCLCPP_INFO(this->get_logger(),"Necessary parts published --ignoring the rest from now");
      
    }
     
        
}




