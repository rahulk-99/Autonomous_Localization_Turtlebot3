#include "rclcpp/rclcpp.hpp"
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>
#include <camera_broadcaster.hpp>
#include <mage_msgs/msg/part.hpp>
#include <mage_msgs/msg/parts.hpp>
#include <part_string.hpp>
#include <vector>
#include <string>

struct part{
   std::string color;
   std::string type;
   geometry_msgs::msg::Pose world_pose;

};

std::vector<part> parts;


// callback function for the subscription to /mage/camera1/image topic
void CameraSubscriber::camera1_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
if (!parts_received1_){
        
        // getting the sensor data for camera's coordinate in world
        tf2::Transform camera_to_world;
        tf2::fromMsg(msg->sensor_pose, camera_to_world);
        for (const auto& part_pose : msg->part_poses) {

        // getting the part pose data for part's coordinate in camera
        tf2::Transform part_in_camera;
        tf2::fromMsg(part_pose.pose, part_in_camera);
        // multiplication to get part's coordinate in world
        tf2::Transform part_in_world = camera_to_world * part_in_camera;

        geometry_msgs::msg::Pose world_pose;
        tf2::toMsg(part_in_world, world_pose);

        std::string color_ = part_color(part_pose.part.color);
        std::string type_ = part_type(part_pose.part.type);
        
        //condition to store published parts in the vector 
        if (color_ == "Red" && type_ == "Battery") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Blue" && type_ == "Battery") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Green" && type_ == "Pump") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Red" && type_ == "Regulator") {
                parts.push_back({color_, type_, world_pose});
            }

        RCLCPP_INFO(this->get_logger(),"Received part color: %s, part type: %s",color_.c_str(), type_.c_str());
        RCLCPP_INFO(this->get_logger(),"Part Pose:");
        RCLCPP_INFO(this->get_logger(),"Position: x=%.2f, y=%.2f, z=%.2f",
                    part_pose.pose.position.x, part_pose.pose.position.y, part_pose.pose.position.z);
        // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
        //             part_pose.pose.orientation.x, part_pose.pose.orientation.y,
        //             part_pose.pose.orientation.z, part_pose.pose.orientation.w);
        RCLCPP_INFO(this->get_logger(), "World Position and orientation of %s %s:", color_.c_str(), type_.c_str());
        RCLCPP_INFO(this->get_logger(), "Position: x=%.2f, y=%.2f, z=%.2f",
                        world_pose.position.x, world_pose.position.y, world_pose.position.z);
        // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
        //             world_pose.orientation.x, world_pose.orientation.y,
        //             world_pose.orientation.z, world_pose.orientation.w);
       }

  
    // RCLCPP_INFO(this->get_logger(),"Sensor Pose:");
    // RCLCPP_INFO(this->get_logger(),"Position: x=%.2f, y=%.2f, z=%.2f",
    //             msg->sensor_pose.position.x, msg->sensor_pose.position.y, msg->sensor_pose.position.z);
    // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
    //             msg->sensor_pose.orientation.x, msg->sensor_pose.orientation.y,
    //             msg->sensor_pose.orientation.z, msg->sensor_pose.orientation.w);

    parts_received1_ = true;
   for (const auto& part : parts) {
    RCLCPP_INFO(this->get_logger(), 
                "Part: Color = %s, Type = %s, Position = (x: %.2f, y: %.2f, z: %.2f)",
                part.color.c_str(), part.type.c_str(),
                part.world_pose.position.x, part.world_pose.position.y, part.world_pose.position.z);
    }
                
}

}

void CameraSubscriber::camera2_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
if (!parts_received2_){

        tf2::Transform camera_to_world;
        tf2::fromMsg(msg->sensor_pose, camera_to_world);
        for (const auto& part_pose : msg->part_poses) {

        tf2::Transform part_in_camera;
        tf2::fromMsg(part_pose.pose, part_in_camera);
        tf2::Transform part_in_world = camera_to_world * part_in_camera;

        geometry_msgs::msg::Pose world_pose;
        tf2::toMsg(part_in_world, world_pose);
        std::string color_ = part_color(part_pose.part.color);
        std::string type_ = part_type(part_pose.part.type);
        if (color_ == "Red" && type_ == "Battery") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Blue" && type_ == "Battery") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Green" && type_ == "Pump") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Red" && type_ == "Regulator") {
                parts.push_back({color_, type_, world_pose});
            }
        RCLCPP_INFO(this->get_logger(),"Received part color: %s, part type: %s",color_.c_str(), type_.c_str());
        RCLCPP_INFO(this->get_logger(),"Part Pose:");
        RCLCPP_INFO(this->get_logger(),"Position: x=%.2f, y=%.2f, z=%.2f",
                    part_pose.pose.position.x, part_pose.pose.position.y, part_pose.pose.position.z);
        // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
        //             part_pose.pose.orientation.x, part_pose.pose.orientation.y,
        //             part_pose.pose.orientation.z, part_pose.pose.orientation.w);
        RCLCPP_INFO(this->get_logger(), "World Position and orientation of %s %s:", color_.c_str(), type_.c_str());
        RCLCPP_INFO(this->get_logger(), "Position: x=%.2f, y=%.2f, z=%.2f",
                        world_pose.position.x, world_pose.position.y, world_pose.position.z);
        // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
        //             world_pose.orientation.x, world_pose.orientation.y,
        //             world_pose.orientation.z, world_pose.orientation.w);
       }

  
    // RCLCPP_INFO(this->get_logger(),"Sensor Pose:");
    // RCLCPP_INFO(this->get_logger(),"Position: x=%.2f, y=%.2f, z=%.2f",
    //             msg->sensor_pose.position.x, msg->sensor_pose.position.y, msg->sensor_pose.position.z);
    // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
    //             msg->sensor_pose.orientation.x, msg->sensor_pose.orientation.y,
    //             msg->sensor_pose.orientation.z, msg->sensor_pose.orientation.w);

    parts_received2_ = true;
     for (const auto& part : parts) {
    RCLCPP_INFO(this->get_logger(), 
                "Part: Color = %s, Type = %s, Position = (x: %.2f, y: %.2f, z: %.2f)",
                part.color.c_str(), part.type.c_str(),
                part.world_pose.position.x, part.world_pose.position.y, part.world_pose.position.z);
    }
                
}

}

void CameraSubscriber::camera3_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
if (!parts_received3_){

        tf2::Transform camera_to_world;
        tf2::fromMsg(msg->sensor_pose, camera_to_world);
        for (const auto& part_pose : msg->part_poses) {

        tf2::Transform part_in_camera;
        tf2::fromMsg(part_pose.pose, part_in_camera);
        tf2::Transform part_in_world = camera_to_world * part_in_camera;

        geometry_msgs::msg::Pose world_pose;
        tf2::toMsg(part_in_world, world_pose);
        std::string color_ = part_color(part_pose.part.color);
        std::string type_ = part_type(part_pose.part.type);

        if (color_ == "Red" && type_ == "Battery") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Blue" && type_ == "Battery") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Green" && type_ == "Pump") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Red" && type_ == "Regulator") {
                parts.push_back({color_, type_, world_pose});
            }
        RCLCPP_INFO(this->get_logger(),"Received part color: %s, part type: %s",color_.c_str(), type_.c_str());
        RCLCPP_INFO(this->get_logger(),"Part Pose:");
        RCLCPP_INFO(this->get_logger(),"Position: x=%.2f, y=%.2f, z=%.2f",
                    part_pose.pose.position.x, part_pose.pose.position.y, part_pose.pose.position.z);
        // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
        //             part_pose.pose.orientation.x, part_pose.pose.orientation.y,
        //             part_pose.pose.orientation.z, part_pose.pose.orientation.w);
        RCLCPP_INFO(this->get_logger(), "World Position and orientation of %s %s:", color_.c_str(), type_.c_str());
        RCLCPP_INFO(this->get_logger(), "Position: x=%.2f, y=%.2f, z=%.2f",
                        world_pose.position.x, world_pose.position.y, world_pose.position.z);
        // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
        //             world_pose.orientation.x, world_pose.orientation.y,
        //             world_pose.orientation.z, world_pose.orientation.w);
       }

  
    // RCLCPP_INFO(this->get_logger(),"Sensor Pose:");
    // RCLCPP_INFO(this->get_logger(),"Position: x=%.2f, y=%.2f, z=%.2f",
    //             msg->sensor_pose.position.x, msg->sensor_pose.position.y, msg->sensor_pose.position.z);
    // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
    //             msg->sensor_pose.orientation.x, msg->sensor_pose.orientation.y,
    //             msg->sensor_pose.orientation.z, msg->sensor_pose.orientation.w);
    parts_received3_ = true;

     for (const auto& part : parts) {
    RCLCPP_INFO(this->get_logger(), 
                "Part: Color = %s, Type = %s, Position = (x: %.2f, y: %.2f, z: %.2f)",
                part.color.c_str(), part.type.c_str(),
                part.world_pose.position.x, part.world_pose.position.y, part.world_pose.position.z);
    }
                
}

}

void CameraSubscriber::camera4_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
if (!parts_received4_){

        tf2::Transform camera_to_world;
        tf2::fromMsg(msg->sensor_pose, camera_to_world);
        for (const auto& part_pose : msg->part_poses) {

        tf2::Transform part_in_camera;
        tf2::fromMsg(part_pose.pose, part_in_camera);
        tf2::Transform part_in_world = camera_to_world * part_in_camera;

        geometry_msgs::msg::Pose world_pose;
        tf2::toMsg(part_in_world, world_pose);
        std::string color_ = part_color(part_pose.part.color);
        std::string type_ = part_type(part_pose.part.type);

        if (color_ == "Red" && type_ == "Battery") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Blue" && type_ == "Battery") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Green" && type_ == "Pump") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Red" && type_ == "Regulator") {
                parts.push_back({color_, type_, world_pose});
            }
        RCLCPP_INFO(this->get_logger(),"Received part color: %s, part type: %s",color_.c_str(), type_.c_str());
        RCLCPP_INFO(this->get_logger(),"Part Pose:");
        RCLCPP_INFO(this->get_logger(),"Position: x=%.2f, y=%.2f, z=%.2f",
                    part_pose.pose.position.x, part_pose.pose.position.y, part_pose.pose.position.z);
        // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
        //             part_pose.pose.orientation.x, part_pose.pose.orientation.y,
        //             part_pose.pose.orientation.z, part_pose.pose.orientation.w);
        RCLCPP_INFO(this->get_logger(), "World Position and orientation of %s %s:", color_.c_str(), type_.c_str());
        RCLCPP_INFO(this->get_logger(), "Position: x=%.2f, y=%.2f, z=%.2f",
                        world_pose.position.x, world_pose.position.y, world_pose.position.z);
        // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
        //             world_pose.orientation.x, world_pose.orientation.y,
        //             world_pose.orientation.z, world_pose.orientation.w);
       }

  
    // RCLCPP_INFO(this->get_logger(),"Sensor Pose:");
    // RCLCPP_INFO(this->get_logger(),"Position: x=%.2f, y=%.2f, z=%.2f",
    //             msg->sensor_pose.position.x, msg->sensor_pose.position.y, msg->sensor_pose.position.z);
    // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
    //             msg->sensor_pose.orientation.x, msg->sensor_pose.orientation.y,
    //             msg->sensor_pose.orientation.z, msg->sensor_pose.orientation.w);
    parts_received4_ = true;
     for (const auto& part : parts) {
    RCLCPP_INFO(this->get_logger(), 
                "Part: Color = %s, Type = %s, Position = (x: %.2f, y: %.2f, z: %.2f)",
                part.color.c_str(), part.type.c_str(),
                part.world_pose.position.x, part.world_pose.position.y, part.world_pose.position.z);
    }
                
}

}

void CameraSubscriber::camera5_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
if (!parts_received5_){

        tf2::Transform camera_to_world;
        tf2::fromMsg(msg->sensor_pose, camera_to_world);
        for (const auto& part_pose : msg->part_poses) {

        tf2::Transform part_in_camera;
        tf2::fromMsg(part_pose.pose, part_in_camera);
        tf2::Transform part_in_world = camera_to_world * part_in_camera;

        geometry_msgs::msg::Pose world_pose;
        tf2::toMsg(part_in_world, world_pose);
        std::string color_ = part_color(part_pose.part.color);
        std::string type_ = part_type(part_pose.part.type);
        if (color_ == "Red" && type_ == "Battery") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Blue" && type_ == "Battery") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Green" && type_ == "Pump") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Red" && type_ == "Regulator") {
                parts.push_back({color_, type_, world_pose});
            }
        RCLCPP_INFO(this->get_logger(),"Received part color: %s, part type: %s",color_.c_str(), type_.c_str());
        RCLCPP_INFO(this->get_logger(),"Part Pose:");
        RCLCPP_INFO(this->get_logger(),"Position: x=%.2f, y=%.2f, z=%.2f",
                    part_pose.pose.position.x, part_pose.pose.position.y, part_pose.pose.position.z);
        // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
        //             part_pose.pose.orientation.x, part_pose.pose.orientation.y,
        //             part_pose.pose.orientation.z, part_pose.pose.orientation.w);
        RCLCPP_INFO(this->get_logger(), "World Position and orientation of %s %s:", color_.c_str(), type_.c_str());
        RCLCPP_INFO(this->get_logger(), "Position: x=%.2f, y=%.2f, z=%.2f",
                        world_pose.position.x, world_pose.position.y, world_pose.position.z);
        // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
        //             world_pose.orientation.x, world_pose.orientation.y,
        //             world_pose.orientation.z, world_pose.orientation.w);
       }

  
    // RCLCPP_INFO(this->get_logger(),"Sensor Pose:");
    // RCLCPP_INFO(this->get_logger(),"Position: x=%.2f, y=%.2f, z=%.2f",
    //             msg->sensor_pose.position.x, msg->sensor_pose.position.y, msg->sensor_pose.position.z);
    // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
    //             msg->sensor_pose.orientation.x, msg->sensor_pose.orientation.y,
    //             msg->sensor_pose.orientation.z, msg->sensor_pose.orientation.w);
    parts_received5_ = true;

     for (const auto& part : parts) {
    RCLCPP_INFO(this->get_logger(), 
                "Part: Color = %s, Type = %s, Position = (x: %.2f, y: %.2f, z: %.2f)",
                part.color.c_str(), part.type.c_str(),
                part.world_pose.position.x, part.world_pose.position.y, part.world_pose.position.z);
    }
                
}

}

void CameraSubscriber::camera6_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
if (!parts_received6_){

        tf2::Transform camera_to_world;
        tf2::fromMsg(msg->sensor_pose, camera_to_world);
        for (const auto& part_pose : msg->part_poses) {

        tf2::Transform part_in_camera;
        tf2::fromMsg(part_pose.pose, part_in_camera);
        tf2::Transform part_in_world = camera_to_world * part_in_camera;

        geometry_msgs::msg::Pose world_pose;
        tf2::toMsg(part_in_world, world_pose);
        std::string color_ = part_color(part_pose.part.color);
        std::string type_ = part_type(part_pose.part.type);
        if (color_ == "Red" && type_ == "Battery") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Blue" && type_ == "Battery") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Green" && type_ == "Pump") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Red" && type_ == "Regulator") {
                parts.push_back({color_, type_, world_pose});
            }
        RCLCPP_INFO(this->get_logger(),"Received part color: %s, part type: %s",color_.c_str(), type_.c_str());
        RCLCPP_INFO(this->get_logger(),"Part Pose:");
        RCLCPP_INFO(this->get_logger(),"Position: x=%.2f, y=%.2f, z=%.2f",
                    part_pose.pose.position.x, part_pose.pose.position.y, part_pose.pose.position.z);
        // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
        //             part_pose.pose.orientation.x, part_pose.pose.orientation.y,
        //             part_pose.pose.orientation.z, part_pose.pose.orientation.w);
        RCLCPP_INFO(this->get_logger(), "World Position and orientation of %s %s:", color_.c_str(), type_.c_str());
        RCLCPP_INFO(this->get_logger(), "Position: x=%.2f, y=%.2f, z=%.2f",
                        world_pose.position.x, world_pose.position.y, world_pose.position.z);
        // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
        //             world_pose.orientation.x, world_pose.orientation.y,
        //             world_pose.orientation.z, world_pose.orientation.w);
       }

  
    // RCLCPP_INFO(this->get_logger(),"Sensor Pose:");
    // RCLCPP_INFO(this->get_logger(),"Position: x=%.2f, y=%.2f, z=%.2f",
    //             msg->sensor_pose.position.x, msg->sensor_pose.position.y, msg->sensor_pose.position.z);
    // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
    //             msg->sensor_pose.orientation.x, msg->sensor_pose.orientation.y,
    //             msg->sensor_pose.orientation.z, msg->sensor_pose.orientation.w);

    parts_received6_ = true;
     for (const auto& part : parts) {
    RCLCPP_INFO(this->get_logger(), 
                "Part: Color = %s, Type = %s, Position = (x: %.2f, y: %.2f, z: %.2f)",
                part.color.c_str(), part.type.c_str(),
                part.world_pose.position.x, part.world_pose.position.y, part.world_pose.position.z);
    }
                
}

}

void CameraSubscriber::camera7_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
if (!parts_received7_){

        tf2::Transform camera_to_world;
        tf2::fromMsg(msg->sensor_pose, camera_to_world);
        for (const auto& part_pose : msg->part_poses) {

        tf2::Transform part_in_camera;
        tf2::fromMsg(part_pose.pose, part_in_camera);
        tf2::Transform part_in_world = camera_to_world * part_in_camera;

        geometry_msgs::msg::Pose world_pose;
        tf2::toMsg(part_in_world, world_pose);
        std::string color_ = part_color(part_pose.part.color);
        std::string type_ = part_type(part_pose.part.type);
        if (color_ == "Red" && type_ == "Battery") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Blue" && type_ == "Battery") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Green" && type_ == "Pump") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Red" && type_ == "Regulator") {
                parts.push_back({color_, type_, world_pose});
            }
        RCLCPP_INFO(this->get_logger(),"Received part color: %s, part type: %s",color_.c_str(), type_.c_str());
        RCLCPP_INFO(this->get_logger(),"Part Pose:");
        RCLCPP_INFO(this->get_logger(),"Position: x=%.2f, y=%.2f, z=%.2f",
                    part_pose.pose.position.x, part_pose.pose.position.y, part_pose.pose.position.z);
        // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
        //             part_pose.pose.orientation.x, part_pose.pose.orientation.y,
        //             part_pose.pose.orientation.z, part_pose.pose.orientation.w);
        RCLCPP_INFO(this->get_logger(), "World Position and orientation of %s %s:", color_.c_str(), type_.c_str());
        RCLCPP_INFO(this->get_logger(), "Position: x=%.2f, y=%.2f, z=%.2f",
                        world_pose.position.x, world_pose.position.y, world_pose.position.z);
        // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
        //             world_pose.orientation.x, world_pose.orientation.y,
        //             world_pose.orientation.z, world_pose.orientation.w);
       }

  
    // RCLCPP_INFO(this->get_logger(),"Sensor Pose:");
    // RCLCPP_INFO(this->get_logger(),"Position: x=%.2f, y=%.2f, z=%.2f",
    //             msg->sensor_pose.position.x, msg->sensor_pose.position.y, msg->sensor_pose.position.z);
    // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
    //             msg->sensor_pose.orientation.x, msg->sensor_pose.orientation.y,
    //             msg->sensor_pose.orientation.z, msg->sensor_pose.orientation.w);
    parts_received7_ = true;

    for (const auto& part : parts) {
    RCLCPP_INFO(this->get_logger(), 
                "Part: Color = %s, Type = %s, Position = (x: %.2f, y: %.2f, z: %.2f)",
                part.color.c_str(), part.type.c_str(),
                part.world_pose.position.x, part.world_pose.position.y, part.world_pose.position.z);
    }
                
}

}

void CameraSubscriber::camera8_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
if (!parts_received8_){

        tf2::Transform camera_to_world;
        tf2::fromMsg(msg->sensor_pose, camera_to_world);
        for (const auto& part_pose : msg->part_poses) {

        tf2::Transform part_in_camera;
        tf2::fromMsg(part_pose.pose, part_in_camera);
        tf2::Transform part_in_world = camera_to_world * part_in_camera;

        geometry_msgs::msg::Pose world_pose;
        tf2::toMsg(part_in_world, world_pose);
        std::string color_ = part_color(part_pose.part.color);
        std::string type_ = part_type(part_pose.part.type);
        if (color_ == "Red" && type_ == "Battery") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Blue" && type_ == "Battery") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Green" && type_ == "Pump") {
                parts.push_back({color_, type_, world_pose});
            } else if (color_ == "Red" && type_ == "Regulator") {
                parts.push_back({color_, type_, world_pose});
            }
        RCLCPP_INFO(this->get_logger(),"Received part color: %s, part type: %s",color_.c_str(), type_.c_str());
        RCLCPP_INFO(this->get_logger(),"Part Pose:");
        RCLCPP_INFO(this->get_logger(),"Position: x=%.2f, y=%.2f, z=%.2f",
                    part_pose.pose.position.x, part_pose.pose.position.y, part_pose.pose.position.z);
        // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
        //             part_pose.pose.orientation.x, part_pose.pose.orientation.y,
        //             part_pose.pose.orientation.z, part_pose.pose.orientation.w);
        RCLCPP_INFO(this->get_logger(), "World Position and orientation of %s %s:", color_.c_str(), type_.c_str());
        RCLCPP_INFO(this->get_logger(), "Position: x=%.2f, y=%.2f, z=%.2f",
                        world_pose.position.x, world_pose.position.y, world_pose.position.z);
        // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
        //             world_pose.orientation.x, world_pose.orientation.y,
        //             world_pose.orientation.z, world_pose.orientation.w);
       }

  
    // RCLCPP_INFO(this->get_logger(),"Sensor Pose:");
    // RCLCPP_INFO(this->get_logger(),"Position: x=%.2f, y=%.2f, z=%.2f",
    //             msg->sensor_pose.position.x, msg->sensor_pose.position.y, msg->sensor_pose.position.z);
    // RCLCPP_INFO(this->get_logger(),"Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
    //             msg->sensor_pose.orientation.x, msg->sensor_pose.orientation.y,
    //             msg->sensor_pose.orientation.z, msg->sensor_pose.orientation.w);

    parts_received8_ = true;

    for (const auto& part : parts) {
    RCLCPP_INFO(this->get_logger(), 
                "Part: Color = %s, Type = %s, Position = (x: %.2f, y: %.2f, z: %.2f)",
                part.color.c_str(), part.type.c_str(),
                part.world_pose.position.x, part.world_pose.position.y, part.world_pose.position.z);
    }
                
}

}








    


 
