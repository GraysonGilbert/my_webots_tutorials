/**
 * @file walker_robot_driver.hpp
 * @author Grayson Gilbert (ggilbert@umd.edu)
 * @brief Header file for the WalkerRobotDriver class
 * @version 0.1
 * @date 2025-11-22
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

#ifndef WALKER_ROBOT_DRIVER_HPP_
#define WALKER_ROBOT_DRIVER_HPP_

#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace walker_robot_driver {
class WalkerRobotDriver : public webots_ros2_driver::PluginInterface {
public:
  /**
   * @brief Called at every time step of the simulation. Retrieves the desrived forward and
   *        angular speeds from the cmd_vel message. Converts the message contents into the
   *        appropriate left and right wheel angular velocity commands.
   * 
   */
  void step() override;

  /**
   * @brief Initializes the plugin by setting up the robot motors, setting their positions and 
   *        velocities, and subscribing to the /cmd_vel topic.
   * 
   * @param node - Pointer to the Webots ROS2 driver node, used to create subscriptions.
   * @param parameters - A map of string key-value pairs for additional initialization parameters.
   */                   
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;

private:
  /**
   * @brief Callback function for each Twist message received on the /cmd_vel topic
   * 
   * @param msg - /cmd_vel message received that will be parsed
   */
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_subscription_;
  geometry_msgs::msg::Twist cmd_vel_msg;

  WbDeviceTag right_motor;
  WbDeviceTag left_motor;
};
} // namespace walker_robot_driver
#endif // WALKER_ROBOT_DRIVER_HPP_