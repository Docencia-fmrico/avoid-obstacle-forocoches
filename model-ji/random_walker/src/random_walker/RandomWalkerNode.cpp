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

#include "random_walker/RandomWalkerNode.hpp"

using namespace std::chrono_literals; // NOLINT
using std::placeholders::_1;

RandomWalkerNode::RandomWalkerNode()
: Node("rand_walker_node")
{
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  debug_pub_ = create_publisher<DebugNode::DebugMessage>(DebugNode::TOPIC_NAME, 10);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(),
    std::bind(&RandomWalkerNode::scan_callback, this, _1));

  timer_ = create_wall_timer(
    500ms, std::bind(&RandomWalkerNode::control_cycle, this));

  // Initialize the last state to stop
  last_state_ = STOP;
}

void RandomWalkerNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

void RandomWalkerNode::control_cycle()
{
  // Do nothing until the first sensor read
  if (last_scan_ == nullptr) {
    debug_msg_.data = DebugNode::ERROR;
    debug_pub_->publish(debug_msg_);
    return;
  }

  debug_msg_.data = DebugNode::OK;
  // FSM
  switch (state_) {
    case STOP:
      debug_msg_.data = DebugNode::READY;
      out_vel_.linear.x = SPEED_STOP_LINEAR;
      out_vel_.angular.z = SPEED_STOP_ANGULAR;
      // Can go to last state, if is null then go forward
      if (check_stop_2_last()) {
        // Check if we have an old state
        if (last_state_ == STOP) {
          // If we don't, go forward
          change_state(FORWARD);
        } else {
          // If we have an old state, go to that one
          change_state(last_state_);
        }
      }
      break;
    case FORWARD:
      out_vel_.linear.x = SPEED_FORWARD_LINEAR;
      out_vel_.angular.z = 0;
      // Can go to stop or to turn
      if (check_2_stop()) {
        change_state(STOP);
      }
      if (check_2_turn()) {
        change_state(TURN);
      }
      break;
    case TURN:
      debug_msg_.data = DebugNode::ERROR;
      out_vel_.linear.x = SPEED_TURN_LINEAR;
      out_vel_.angular.z = SPEED_TURN_ANGULAR * obstacle_position_;
      // Can go to stop or to rotation
      if (check_2_stop()) {
        change_state(STOP);
      }
      if (check_turn_2_rotation()) {
        change_state(ROTATION);
      }
      break;
    case ROTATION:
      debug_msg_.data = DebugNode::NOT_ON_GROUND;
      out_vel_.linear.x = SPEED_FORWARD_LINEAR;
      out_vel_.angular.z = -(SPEED_TURN_ANGULAR * obstacle_position_);
      // Can go to stop, forward or turn
      if (check_2_stop()) {
        change_state(STOP);
      }
      if (check_rotation_2_forward()) {
        change_state(FORWARD);
      }
      if (check_2_turn()) {
        change_state(TURN);
      }
      break;
    default:
      // Something went wrong
      debug_msg_.data = DebugNode::ERROR;
      out_vel_.linear.x = SPEED_STOP_LINEAR;
      out_vel_.angular.z = SPEED_STOP_ANGULAR;
      debug_pub_->publish(debug_msg_);
      vel_pub_->publish(out_vel_);
      exit(EXIT_FAILURE);
      break;
  }

  // Publish data
  vel_pub_->publish(out_vel_);
  debug_pub_->publish(debug_msg_);
}

// FSM states
void RandomWalkerNode::change_state(int new_state)
{
  last_state_ = state_;
  state_ = new_state;
  state_timestamp_ = now();
}

// FSM changes
bool RandomWalkerNode::check_stop_2_last()
{
  // Return if everything works as intended
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed < LASER_SCAN_TIMEOUT;
}

bool RandomWalkerNode::check_2_stop()
{
  // Stop if something is wrong
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed > LASER_SCAN_TIMEOUT;
}

bool RandomWalkerNode::check_turn_2_rotation()
{
  // Start rotation when it finishes turnig
  return (now() - state_timestamp_) > TURNING_TIME;
}

bool RandomWalkerNode::check_2_turn()
{
  // Turn if the laser detects an object near
  int size = last_scan_->ranges.size();

  // Right range
  for (int i = 0; i < SCAN_RANGE; i++) {
    if (last_scan_->ranges[i] < OBSTACLE_DISTANCE_THRESHOLD) {
      obstacle_position_ = -1;  // Obstacle position in the right
      return true;
    }
  }

  // Left range
  for (int i = size - 1; i > (size - SCAN_RANGE); i--) {
    if (last_scan_->ranges[i] < OBSTACLE_DISTANCE_THRESHOLD) {
      obstacle_position_ = 1;  // Obstacle position in the left
      return true;
    }
  }
  return false;
}

bool RandomWalkerNode::check_rotation_2_forward()
{
  // Go forward when it finishes rotating
  return (now() - state_timestamp_) > ROTATING_TIME;
}
