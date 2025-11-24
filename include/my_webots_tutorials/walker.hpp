/**
 * @file walker.hpp
 * @author Grayson Gilbert (ggilbert@umd.edu)
 * @brief ROS2 Walker node declaration
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


#ifndef WALKER_HPP_
#define WALKER_HPP_


#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "my_webots_tutorials/walker_states.hpp"
#include "my_webots_tutorials/walker_state_interface.hpp"
//#include "walker_states.hpp"

class Walker: public rclcpp::Node {
    public:
        explicit Walker();

    private:
        void left_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg);

        void right_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg);
    
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr left_sensor_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr right_sensor_sub_;

        double left_sensor_value{0.0};
        double right_sensor_value{0.0};

        std::unique_ptr<WalkerStateMachine> state_machine_;
    
};

#endif // WALKER_HPP_