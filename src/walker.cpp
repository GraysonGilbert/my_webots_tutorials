/**
 * @file walker.cpp
 * @author Grayson Gilbert (ggilbert@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2025-11-23
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#include "my_webots_tutorials/walker.hpp"
//#include "my_webots_tutorials/walker_states.hpp"
#include <memory>
#include <rclcpp/logging.hpp>

#define MAX_RANGE 0.15

Walker::Walker() : Node("obstacle_avoider") {
  publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  left_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>("/left_sensor", 1, std::bind(&Walker::left_sensor_callback, this, std::placeholders::_1));

  right_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>("/right_sensor", 1, std::bind(&Walker::right_sensor_callback, this, std::placeholders::_1));

  // Code to start in move state
  state_machine_ = std::make_unique<WalkerStateMachine>();
  RCLCPP_INFO_STREAM(get_logger(), "Current Walker State: " << state_machine_->state_name());

}

void Walker::left_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
  left_sensor_value = msg->range;
}

void Walker::right_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
  right_sensor_value = msg->range;

  // Begin new code for state machine
  auto command_message = state_machine_->movement_state(left_sensor_value, right_sensor_value);
  
  publisher_->publish(std::move(command_message));

  auto new_state = state_machine_->transition_state(left_sensor_value, right_sensor_value);

  // Log the current state
  RCLCPP_INFO_STREAM(get_logger(), "Current Walker State: " << state_machine_->state_name());

}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto walker = std::make_shared<Walker>();
  rclcpp::spin(walker);
  rclcpp::shutdown();
  return 0;
}
