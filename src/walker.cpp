/**
 * @file walker.cpp
 * @author Grayson Gilbert (ggilbert@umd.edu)
 * @brief ROS2 walker node implemenation
 * @version 0.1
 * @date 2025-11-23
 * 
 * @copyright Copyright (c) 2025
 * 
 */

// Copyright 2025 Grayson Gilbert
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "my_webots_tutorials/walker.hpp"
#include "my_webots_tutorials/walker_states.hpp"
#include <memory>
#include <rclcpp/logging.hpp>

#define MAX_RANGE 0.15

bool WalkerStateMachine::turned_clockwise_ = true;

Walker::Walker() : Node("obstacle_avoider") {
  // Publisher used to send velocity commands to the robot
  publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  // Subscribe to left range sensor
  left_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>("/left_sensor", 1, std::bind(&Walker::left_sensor_callback, this, std::placeholders::_1));

  // Subscribe to right range sensor
  right_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>("/right_sensor", 1, std::bind(&Walker::right_sensor_callback, this, std::placeholders::_1));

  // Create the state machine and start in the MoveForward state
  state_machine_ = std::make_unique<WalkerStateMachine>();

  // Log initial state
  RCLCPP_INFO_STREAM(get_logger(), "Current Walker State: " << state_machine_->state_name());

}

// Update stored left sensor value
void Walker::left_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
  left_sensor_value = msg->range;
}

// Right sensor callback also triggers the main control loop
void Walker::right_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) { 
  right_sensor_value = msg->range;

  // Ask the state machine what movement command should be sent
  auto command_message = state_machine_->movement_state(left_sensor_value, right_sensor_value);
  
  // Publish the Twist command to /cmd_vel
  publisher_->publish(std::move(command_message));

  // Check if the state machine wants to transition to a different state
  auto new_state = state_machine_->transition_state(left_sensor_value, right_sensor_value);

  // Log the current state
  if (new_state) {
    RCLCPP_INFO_STREAM(get_logger(), "Current Walker State: " << state_machine_->state_name());
  }
}
  

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // Create and spin the walker node
  auto walker = std::make_shared<Walker>();
  rclcpp::spin(walker);
  rclcpp::shutdown();
  return 0;
}
