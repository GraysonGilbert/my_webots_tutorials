/**
 * @file walker_state_interface.hpp
 * @author Grayson Gilbert (ggilbert@umd.edu)
 * @brief Abstract Walker State Interface header file declaring Walker State Machine states
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


#ifndef WALKER_STATE_INTERFACE_HPP_
#define WALKER_STATE_INTERFACE_HPP_

#include <memory> 
#include "geometry_msgs/msg/twist.hpp"

class Walker;

class WalkerStateInterface {
    public:
        /**
         * @brief Destroy the Walker State Interface object
         * 
         */
        virtual ~WalkerStateInterface() = default;

        /**
         * @brief Compute the robot's velocity command for this state
         * 
         * @param left_sensor - current sensor reading from the left distance sensor
         * @param right_sensor - current sensor reading from the right distance sensor
         * @return std::unique_ptr<geometry_msgs::msg::Twist> - A Twist message that will be published by the node to the 
         *         /cmd_vel topic
         */
        virtual std::unique_ptr<geometry_msgs::msg::Twist> movement_state(double left_sensor, double right_sensor) = 0;

        /**
         * @brief Determine whether the robot should transition to a new state
         * 
         * @param left_sensor - current sensor reading from the left distance sensor
         * @param right_sensor - current sensor reading from the right distance sensor
         * @return std::unique_ptr<WalkerStateInterface> - a new state object if a transition is required
         *                                               - returns nullptr if no transition is required
         */
        virtual std::unique_ptr<WalkerStateInterface> transition_state(double left_sensor, double right_sensor) = 0;

        /**
         * @brief Get a human-readable name for the current state
         * 
         * @return const char* - string idenfitier for the state
         */
        virtual const char* state_name() const = 0;
        
        static constexpr double MAX_RANGE = 0.15;   // Maximum valid range reading for the distance sensor
        static constexpr double MIN_TURN_THRESHOLD = 0.9 * MAX_RANGE;   // Minimum threshold to determine if robot must turn
};


#endif // WALKER_STATE_INTERFACE_HPP_
