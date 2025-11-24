/**
 * @file walker_state_machine.hpp
 * @author Grayson Gilbert (ggilbert@umd.edu)
 * @brief Header file for the Walker State Machine
 * @version 0.1
 * @date 2025-11-21
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


#ifndef MY_WEBOTS_TUTORIALS_WALKER_HPP_
#define MY_WEBOTS_TUTORIALS_WALKER_HPP_


#include <algorithm>
#include <memory>

#include "my_webots_tutorials/walker_state_interface.hpp"


class WalkerStateMachine {
    public:

    // Base state should be move forward
        WalkerStateMachine(){

            current_state_ = std::make_unique<state_MoveForward>();
        }

        std::unique_ptr<geometry_msgs::msg::Twist> movement_state(double left_sensor, double right_sensor);

        std::unique_ptr<WalkerStateInterface> transition_state(double left_sensor, double right_sensor);
        
        const char* state_name() const { return current_state_ ? current_state_->state_name() : "(no state)"; }

    protected:

    // Include all states here
    // Move State
        class state_MoveForward : public WalkerStateInterface
        {
            public: 

                virtual std::unique_ptr<geometry_msgs::msg::Twist> movement_state(double left_sensor, double right_sensor) override;


                virtual std::unique_ptr<WalkerStateInterface> transition_state(double left_sensor, double right_sensor) override;

                const char* state_name() const override { return "Forward State"; }
        };

    // Turn State

        class state_Turn : public WalkerStateInterface
        {
            public: 

                virtual std::unique_ptr<geometry_msgs::msg::Twist> movement_state(double left_sensor, double right_sensor) override;


                virtual std::unique_ptr<WalkerStateInterface> transition_state(double left_sensor, double right_sensor) override;

                const char* state_name() const override { return "Turn State"; }

            // private:

            //     bool turn_clockwise_;
            // Bool here to distinguish if turning clockwise or counter closkcwise

        };


    private:

        std::unique_ptr<WalkerStateInterface> current_state_;
    
};

#endif // MY_WEBOTS_TUTORIALS_WALKER_HPP_
