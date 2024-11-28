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
 * @file walker.cpp
 * @author Nazrin Gurbanova (nazrin@umd.edu)
 * @brief Implementation of a robot navigation system using laser scan data for obstacle detection.
 * @version 0.1
 * @date 2024-11-23
 * @copyright Copyright (c) 2024
 */
#include "walker/walker.hpp"

/**
 * @brief Constructor for the WalkerNode class, initializes the node, publishers, subscribers, and sets the initial state.
 */
WalkerNode::WalkerNode()
    : Node("walker_node"),
      obstacle_present_(false),
      turn_clockwise_(true),
      detection_limit_(0.5) {  // Detection limit: can adjust this for sensitivity
    // Publisher for velocity commands to "/cmd_vel"
    velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);

    // Subscription to laser scan data on the "/scan" topic
    laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&WalkerNode::laserScanCallback,
            this, std::placeholders::_1));

    // Initially set state to ForwardMovement
    changeState(std::make_unique<ForwardMovement>());
    RCLCPP_INFO(this->get_logger(), "Walker node initialized.");
}

/**
 * @brief Changes the current state of the WalkerNode to the new provided state.
 * 
 * @param new_state A unique pointer to the new state to switch to.
 */
void WalkerNode::changeState(std::unique_ptr<WalkerStateBase> new_state) {
    current_state_ = std::move(new_state);
    RCLCPP_INFO(this->get_logger(), "State switched.");
}

/**
 * @brief Callback function to process the laser scan data and detect obstacles.
 * 
 * @param msg Shared pointer to the LaserScan message containing the scan data.
 */
void WalkerNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    obstacle_present_ = false;

    // Adjusted indices for detecting obstacles (broader range for obstacle detection)
    int left_start_index = 0;
    int left_end_index = 50;  // Adjusted for a more reasonable range
    int right_start_index = 310;  // Adjusted start index for right range
    int right_end_index = 359;

    /**
     * @brief Detects obstacles by checking laser scan ranges in a given index range.
     * 
     * @param start Starting index of the laser scan range to check.
     * @param end Ending index of the laser scan range to check.
     * @return True if an obstacle is detected, false otherwise.
     */
    auto detectObstacle = [&](int start, int end) {
        for (int i = start; i <= end; ++i) {
            if (msg->ranges[i] < detection_limit_) {
                obstacle_present_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "Obstacle detected. Range: %f at index %d", msg->ranges[i], i);
                return true;
            }
        }
        return false;
    };

    // Check both left and right ranges for obstacles
    if (!detectObstacle(left_start_index, left_end_index)) {
        detectObstacle(right_start_index, right_end_index);
    }

    // Process the current state of the node
    current_state_->processState(*this);
}

/**
 * @brief Publishes a velocity command to the robot's /cmd_vel topic.
 * 
 * @param linear Linear velocity (m/s) to publish.
 * @param angular Angular velocity (rad/s) to publish.
 */
void WalkerNode::publishVelocityCommand(float linear, float angular) {
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    velocity_publisher_->publish(cmd);
    RCLCPP_INFO(this->get_logger(),
        "Published velocity - Linear: %f, Angular: %f", linear, angular);
}

/**
 * @brief Processes the forward movement state of the Walker robot.
 * 
 * If an obstacle is detected, the robot will change its state to either clockwise or counterclockwise turn.
 * If no obstacle is detected, the robot moves forward at a moderate speed.
 * 
 * @param node The WalkerNode instance to control during this state.
 */
void ForwardMovement::processState(WalkerNode& node) {
    if (node.isObstaclePresent()) {
        if (node.shouldTurnClockwise()) {
            node.changeState(std::make_unique<ClockwiseTurn>());
        } else {
            node.changeState(std::make_unique<CounterclockwiseTurn>());
        }
    } else {
        // Increase forward speed to 0.3, slower than before
        node.publishVelocityCommand(0.3, 0.0);  // Move forward at moderate speed
    }
}

/**
 * @brief Processes the clockwise turn state of the Walker robot.
 * 
 * If no obstacle is detected, the robot switches back to forward movement. Otherwise, the robot continues to turn clockwise.
 * 
 * @param node The WalkerNode instance to control during this state.
 */
void ClockwiseTurn::processState(WalkerNode& node) {
    if (!node.isObstaclePresent()) {
        RCLCPP_INFO(node.get_logger(),
            "Obstacle cleared, switching to forward movement.");
        node.setTurnClockwise(false);
        node.changeState(std::make_unique<ForwardMovement>());
    } else {
        // Decreased turning speed to -0.2 for smoother turns
        node.publishVelocityCommand(0.0, -0.2);  // Turn clockwise slower
        RCLCPP_INFO(node.get_logger(), "Turning clockwise.");
    }
}

/**
 * @brief Processes the counterclockwise turn state of the Walker robot.
 * 
 * If no obstacle is detected, the robot switches back to forward movement. Otherwise, the robot continues to turn counterclockwise.
 * 
 * @param node The WalkerNode instance to control during this state.
 */
void CounterclockwiseTurn::processState(WalkerNode& node) {
    if (!node.isObstaclePresent()) {
        RCLCPP_INFO(node.get_logger(),
            "Obstacle cleared, switching to forward movement.");
        node.setTurnClockwise(true);
        node.changeState(std::make_unique<ForwardMovement>());
    } else {
        // Decreased turning speed to 0.2 for smoother turns
        node.publishVelocityCommand(0.0, 0.2);   // Turn counterclockwise slower
        RCLCPP_INFO(node.get_logger(), "Turning counterclockwise.");
    }
}

/**
 * @brief Main entry point for the Walker node. Initializes the ROS 2 system, creates the WalkerNode, and starts spinning.
 * 
 * @param argc The number of command-line arguments.
 * @param argv The command-line arguments.
 * @return Returns 0 on successful execution.
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto walker_node = std::make_shared<WalkerNode>();

    // Start the node's event loop
    rclcpp::spin(walker_node);

    // Shutdown ROS 2 after spinning
    rclcpp::shutdown();
    return 0;
}
