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
#include <string>

/**
 * @brief Converts a color code to its corresponding string representation.
 * 
 * @param color A uint8_t value representing the color code.
 * @return A string corresponding to the color code (e.g., "Red", "Blue"....etc.).
 */
std::string part_color(uint8_t color);

/**
 * @brief Converts a part type code to its corresponding string representation.
 * 
 * @param type A uint8_t value representing the part type code.
 * @return A string corresponding to the part type code (e.g., "Battery", "Pump"....etc.).
 */
std::string part_type(uint8_t type);
