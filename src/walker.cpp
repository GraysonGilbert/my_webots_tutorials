#include "my_webots_tutorials/walker.hpp"
#include "my_webots_tutorials/walker_states.hpp"
#include <memory>
#include <rclcpp/logging.hpp>

#define MAX_RANGE 0.15

Walker::Walker() : Node("obstacle_avoider") {
  publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  left_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/left_sensor", 1,
      std::bind(&Walker::left_sensor_callback, this,
                std::placeholders::_1));

  right_sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/right_sensor", 1,
      std::bind(&Walker::right_sensor_callback, this,
                std::placeholders::_1));

        // Code to start in move state
        current_state_ = std::make_unique<WalkerStateMachine>();
}

void Walker::left_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
  left_sensor_value = msg->range;
}

void Walker::right_sensor_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
  right_sensor_value = msg->range;

  // Begin new code for state machine
  auto command_message = current_state_->movement_state(left_sensor_value, right_sensor_value);

  publisher_->publish(std::move(command_message));
  
  // Log the current state
  RCLCPP_INFO_STREAM(get_logger(), "Current Walker State: " << current_state_->state_name());

}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto walker = std::make_shared<Walker>();
  rclcpp::spin(walker);
  rclcpp::shutdown();
  return 0;
}