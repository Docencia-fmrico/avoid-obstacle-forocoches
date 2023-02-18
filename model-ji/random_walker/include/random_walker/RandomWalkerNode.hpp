// Copyright 2023 Intelligent Robotics Lab
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

#ifndef RANDOM_WALKER__RANDOMWALKERNODE_HPP_
#define RANDOM_WALKER__RANDOMWALKERNODE_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "random_walker/DebugNode.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals; // NOLINT
using std::placeholders::_1;

class RandomWalkerNode : public rclcpp::Node
{
public:
  RandomWalkerNode();

private:
  // Subscribtion callbacks
  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);

  // Control cycle
  void control_cycle();

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  DebugNode::DebugPublisher debug_pub_;

  // Subscription
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  // Message
  geometry_msgs::msg::Twist out_vel_;
  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
  DebugNode::DebugMessage debug_msg_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // --------- FSM ------------
  // FSM variables
  int state_;
  int last_state_;
  rclcpp::Time state_timestamp_;
  // FSM states
  static const int STOP = 0;
  static const int FORWARD = 1;
  static const int TURN = 2;
  static const int ROTATION = 3;
  // FSM changes
  /**
   * @brief Change the state
   * @param new_state
   */
  void change_state(int new_state);
  // FSM Check to change state
  /**
   * @brief Checks if it can resume the movement
   * @return true
   * @return false
   */
  bool check_stop_2_last();
  /**
   * @brief Checks if it needs to stop
   * @return true
   * @return false
   */
  bool check_2_stop();
  /**
   * @brief Checks if it can change from turning to rotating
   * @return true
   * @return false
   */
  bool check_turn_2_rotation();
  /**
   * @brief Check if it needs to turn
   * @return true
   * @return false
   */
  bool check_2_turn();
  /**
   * @brief Checks if the rotation has ended
   * @return true
   * @return false
   */
  bool check_rotation_2_forward();

  // Velocity control
  float SPEED_STOP_LINEAR = 0.0f;
  float SPEED_STOP_ANGULAR = 0.0f;
  float SPEED_FORWARD_LINEAR = 0.3f;
  float SPEED_FORWARD_ANGULAR = 0.0f;
  float SPEED_TURN_LINEAR = 0.0f;
  float SPEED_TURN_ANGULAR = 0.3f;
  const rclcpp::Duration TURNING_TIME {6s};
  const rclcpp::Duration ROTATING_TIME {12s};

  // Laser control
  float OBSTACLE_DISTANCE_THRESHOLD = 1.0f;
  int SCAN_RANGE = 10;
  int obstacle_position_ = 0;  // 1 Left / -1 Right
  const rclcpp::Duration LASER_SCAN_TIMEOUT {1s};
};

#endif  // RANDOM_WALKER__RANDOMWALKERNODE_HPP_
