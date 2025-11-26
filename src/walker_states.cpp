/**
 * @file walker_states.cpp
 * @author Grayson Gilbert (ggilbert@umd.edu)
 * @brief Implementation of the Walker finite state machine for robot navigation
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


#include "my_webots_tutorials/walker_states.hpp"

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logging.hpp>


// Delegate movement logic to whatever the current state is
std::unique_ptr<geometry_msgs::msg::Twist> WalkerStateMachine::movement_state(double left_sensor, double right_sensor)
{
   return current_state_->movement_state(left_sensor, right_sensor);
}

// Ask the current state whether we should transition
std::unique_ptr<WalkerStateInterface> WalkerStateMachine::transition_state(double left_sensor, double right_sensor)
{
    auto next = current_state_->transition_state(left_sensor, right_sensor);

    // If a new state was returned, switch to it
    if (next) {
        current_state_.swap(next);
        return next;
    }
    return nullptr; // State machine itself never returns a new state
}

// Move Forward State:
// Drive straight forward
 std::unique_ptr<geometry_msgs::msg::Twist> WalkerStateMachine::state_MoveForward::movement_state(double left_sensor, double right_sensor){

    auto command_message = std::make_unique<geometry_msgs::msg::Twist>();
    command_message->linear.x = 0.1; // Forward speed
    command_message->angular.z = 0.0; // No turning

    return command_message;

 }

 // Switch to Turn state if either sensor is too close to an obstacle
 std::unique_ptr<WalkerStateInterface> WalkerStateMachine::state_MoveForward::transition_state(double left_sensor, double right_sensor){

   if (left_sensor <= WalkerStateInterface::MIN_TURN_THRESHOLD || right_sensor <= WalkerStateInterface::MIN_TURN_THRESHOLD){
      return  std::make_unique<WalkerStateMachine::state_Turn>(); // Move to turn state
    }
    return nullptr; // Remain in MoveForward
 }


// Turning State:
// Turn in place to avoid obstacle
std::unique_ptr<geometry_msgs::msg::Twist> WalkerStateMachine::state_Turn::movement_state(double left_sensor, double right_sensor){
      auto command_message = std::make_unique<geometry_msgs::msg::Twist>();
      command_message->linear.x = 0.0; // No forward motion while turning
      command_message->angular.z = WalkerStateMachine::turned_clockwise_ ? -2.0 : 2.0; // Rotate

      return command_message;
   
 }

// Transition back to MoveForward once both sensors are clear
 std::unique_ptr<WalkerStateInterface> WalkerStateMachine::state_Turn::transition_state(double left_sensor, double right_sensor){

   if (left_sensor > WalkerStateInterface::MIN_TURN_THRESHOLD && right_sensor > WalkerStateInterface::MIN_TURN_THRESHOLD){

      WalkerStateMachine::turned_clockwise_ = !WalkerStateMachine::turned_clockwise_;
      return std::make_unique<WalkerStateMachine::state_MoveForward>(); // Move to forward state
    }
   return nullptr; // Stay in Turn state
 }