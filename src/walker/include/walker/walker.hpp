// Apache License
// Version 2.0, January 2004
// http://www.apache.org/licenses/LICENSE-2.0
//
// Copyright (c) 2024 Nazrin Gurbanova
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

/**
 * @file walker.hpp
 * @author Nazrin Gurbanova (nazrin@umd.edu)
 * @brief Header file for the Walker robot navigation system.
 * @version 0.1
 * @date 2024-11-23
 * @copyright Copyright (c) 2024
 */

#ifndef WALKER_HPP
#define WALKER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>

/**
 * @brief Abstract base class representing a state in the Walker state machine.
 */
class WalkerStateBase {
public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~WalkerStateBase() = default;

    /**
     * @brief Pure virtual method to process the current state.
     * 
     * @param node The WalkerNode instance that processes the state.
     */
    virtual void processState(class WalkerNode& node) = 0;
};

/**
 * @brief Node class for controlling the Walker robot, utilizing a state machine.
 */
class WalkerNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor that initializes the WalkerNode.
     */
    WalkerNode();

    /**
     * @brief Changes the current state of the WalkerNode.
     * 
     * @param new_state The new state to transition to.
     */
    void changeState(std::unique_ptr<WalkerStateBase> new_state);

    /**
     * @brief Callback for processing incoming laser scan data.
     * 
     * @param msg Shared pointer to the LaserScan message containing sensor data.
     */
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    /**
     * @brief Publishes a velocity command to control the robot's movement.
     * 
     * @param linear The linear velocity component.
     * @param angular The angular velocity component.
     */
    void publishVelocityCommand(float linear, float angular);

    /**
     * @brief Checks if an obstacle is detected in the robot's path.
     * 
     * @return True if an obstacle is present, false otherwise.
     */
    bool isObstaclePresent() const { return obstacle_present_; }

    /**
     * @brief Checks if the robot should turn clockwise.
     * 
     * @return True if the robot should turn clockwise, false otherwise.
     */
    bool shouldTurnClockwise() const { return turn_clockwise_; }

    /**
     * @brief Sets the clockwise turn direction for the robot.
     * 
     * @param clockwise Boolean value indicating the turn direction.
     */
    void setTurnClockwise(bool clockwise) { turn_clockwise_ = clockwise; }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_; ///< Publisher for velocity commands.
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_; ///< Subscription to laser scan data.
    std::unique_ptr<WalkerStateBase> current_state_; ///< The current state of the Walker node.
    bool obstacle_present_; ///< Flag indicating if an obstacle is detected.
    bool turn_clockwise_; ///< Flag indicating if the robot should turn clockwise.
    float detection_limit_; ///< The distance limit for obstacle detection.
};

/**
 * @brief State class for controlling the forward movement of the Walker.
 */
class ForwardMovement : public WalkerStateBase {
public:
    /**
     * @brief Processes the forward movement state.
     * 
     * @param node The WalkerNode instance to control during this state.
     */
    void processState(WalkerNode& node) override;
};

/**
 * @brief State class for controlling a clockwise turn of the Walker.
 */
class ClockwiseTurn : public WalkerStateBase {
public:
    /**
     * @brief Processes the clockwise turn state.
     * 
     * @param node The WalkerNode instance to control during this state.
     */
    void processState(WalkerNode& node) override;
};

/**
 * @brief State class for controlling a counterclockwise turn of the Walker.
 */
class CounterclockwiseTurn : public WalkerStateBase {
public:
    /**
     * @brief Processes the counterclockwise turn state.
     * 
     * @param node The WalkerNode instance to control during this state.
     */
    void processState(WalkerNode& node) override;
};

#endif // WALKER_HPP
