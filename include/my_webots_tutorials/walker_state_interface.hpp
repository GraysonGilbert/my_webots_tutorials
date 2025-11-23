/**
 * @file walker_state_interface.hpp
 * @author Grayson Gilbert (ggilbert@umd.edu)
 * @brief Walker State Interface header file declaration
 * @version 0.1
 * @date 2025-11-23
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef WALKER_STATE_INTERFACE_HPP_
#define WALKER_STATE_INTERFACE_HPP_

#include <memory> 
#include "geometry_msgs/msg/twist.hpp"

class Walker;

class WalkerStateInterface {
    public:
        virtual ~WalkerStateInterface() = default;

        virtual std::unique_ptr<geometry_msgs::msg::Twist> movement_state(double left_sensor, double right_sensor) = 0;

        virtual std::unique_ptr<WalkerStateInterface> transition_state(double left_sensor, double right_sensor) = 0;

        virtual const char* state_name() const = 0;
        
        static constexpr double MAX_RANGE = 0.15;
        static constexpr double MIN_TURN_THRESHOLD = 0.9 * MAX_RANGE;
};


#endif // WALKER_STATE_INTERFACE_HPP_
