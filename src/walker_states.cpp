/**
 * @file walker_states.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-11-23
 * 
 * @copyright Copyright (c) 2025
 * 
 */


 #include "my_webots_tutorials/walker_states.hpp"

 #include <memory>

#include "my_webots_tutorials/walker_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"


// Move Forward State:

 std::unique_ptr<geometry_msgs::msg::Twist> WalkerStateMachine::state_MoveForward::movement_state(double left_sensor, double right_sensor){

    auto command_message = std::make_unique<geometry_msgs::msg::Twist>();
    command_message->linear.x = 0.1;
    command_message->angular.z = 0.0;

    return command_message;

 }

 std::unique_ptr<WalkerStateInterface> WalkerStateMachine::state_MoveForward::transition_state(double left_sensor, double right_sensor){

    if (left_sensor <= WalkerStateInterface::MIN_TURN_THRESHOLD || right_sensor <= WalkerStateInterface::MIN_TURN_THRESHOLD){
        return  std::make_unique<WalkerStateMachine::state_Turn>();
    }
    return nullptr;
 }


 // Turning State:

std::unique_ptr<geometry_msgs::msg::Twist> WalkerStateMachine::state_Turn::movement_state(double left_sensor, double right_sensor){

    auto command_message = std::make_unique<geometry_msgs::msg::Twist>();
    command_message->linear.x = 0.0;
    command_message->angular.z = -2.0;

    return command_message;

 }

 std::unique_ptr<WalkerStateInterface> WalkerStateMachine::state_Turn::transition_state(double left_sensor, double right_sensor){

    if (left_sensor > WalkerStateInterface::MIN_TURN_THRESHOLD && right_sensor > WalkerStateInterface::MIN_TURN_THRESHOLD){
        return std::make_unique<WalkerStateMachine::state_MoveForward>();
    }
   return nullptr;
 }