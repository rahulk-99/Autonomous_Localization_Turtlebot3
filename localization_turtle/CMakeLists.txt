cmake_minimum_required(VERSION 3.8)
project(localization_turtle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bot_waypoint_msgs REQUIRED)

include_directories(include/localization_turtle)

add_executable(localization_turtle src/main.cpp src/waypoint_publisher.cpp src/waypoint_reacher.cpp)
# add_executable(localization_turtle_pub src/waypoint_publisher.cpp)
# add_executable(localization_turtle_sub src/waypoint_reacher.cpp)

ament_target_dependencies(localization_turtle rclcpp bot_waypoint_msgs std_msgs geometry_msgs nav_msgs tf2_geometry_msgs)
# ament_target_dependencies(localization_turtle_pub rclcpp bot_waypoint_msgs std_msgs)
# ament_target_dependencies(localization_turtle_sub rclcpp bot_waypoint_msgs std_msgs geometry_msgs nav_msgs tf2_geometry_msgs)


install(TARGETS localization_turtle DESTINATION lib/${PROJECT_NAME})
# install(TARGETS localization_turtle_pub DESTINATION lib/${PROJECT_NAME})
# install(TARGETS localization_turtle_sub DESTINATION lib/${PROJECT_NAME})



ament_package()

