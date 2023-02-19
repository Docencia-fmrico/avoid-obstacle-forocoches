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
    100ms, std::bind(&RandomWalkerNode::control_cycle, this));

  // Declare parameter
  declare_parameter("SPEED_STOP_LINEAR", SPEED_STOP_LINEAR);
  declare_parameter("SPEED_STOP_ANGULAR", SPEED_STOP_ANGULAR);
  declare_parameter("SPEED_FORWARD_LINEAR", SPEED_FORWARD_LINEAR);
  declare_parameter("SPEED_FORWARD_ANGULAR", SPEED_FORWARD_ANGULAR);
  declare_parameter("SPEED_TURN_LINEAR", SPEED_TURN_LINEAR);
  declare_parameter("SPEED_TURN_ANGULAR", SPEED_TURN_ANGULAR);
  declare_parameter("OBSTACLE_DISTANCE_THRESHOLD", OBSTACLE_DISTANCE_THRESHOLD);
  declare_parameter("SCAN_RANGE", SCAN_RANGE);


  // Retrieve parameters
  get_parameter("SPEED_STOP_LINEAR", SPEED_STOP_LINEAR);
  get_parameter("SPEED_STOP_ANGULAR", SPEED_STOP_ANGULAR);
  get_parameter("SPEED_FORWARD_LINEAR", SPEED_FORWARD_LINEAR);
  get_parameter("SPEED_FORWARD_ANGULAR", SPEED_FORWARD_ANGULAR);
  get_parameter("SPEED_TURN_LINEAR", SPEED_TURN_LINEAR);
  get_parameter("SPEED_TURN_ANGULAR", SPEED_TURN_ANGULAR);
  get_parameter("OBSTACLE_DISTANCE_THRESHOLD", OBSTACLE_DISTANCE_THRESHOLD);
  get_parameter("SCAN_RANGE", SCAN_RANGE);

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
      out_vel_.angular.z = SPEED_FORWARD_ANGULAR;
      finished_rotation_ = false;
      // Can go to stop or to turn
      if (check_2_stop()) {
        change_state(STOP);
      }
      if (check_2_turn()) {
        change_state(TURN);
      }
      break;
    case TURN:
      out_vel_.linear.x = SPEED_TURN_LINEAR;
      out_vel_.angular.z = SPEED_TURN_ANGULAR * obstacle_position_;
      // Can go to stop or to rotation(forward if the rotation already ended)
      if (check_2_stop()) {
        change_state(STOP);
      }
      if (check_turn_2_rotation()) {
        if (finished_rotation_) {
          change_state(FORWARD);
        } else {
          change_state(ROTATION);
        }
      }
      break;
    case ROTATION:
      finished_rotation_ = false;
      out_vel_.linear.x = SPEED_FORWARD_LINEAR;
      out_vel_.angular.z = speed_rotation_angular_;
      // Set rotation speed for the next rotation
      set_rotation_time(speed_rotation_angular_);
      // Can go to stop, forward(Turn again in the opposite direction) or turn
      if (check_2_stop()) {
        change_state(STOP);
      }
      if (check_rotation_2_forward()) {
        finished_rotation_ = true;
        change_state(TURN);
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

  // Left range
  for (int i = 0; i < (size / SCAN_RANGE); i++) {
    if (last_scan_->ranges[i] < OBSTACLE_DISTANCE_THRESHOLD) {
      obstacle_position_ = - 1;  // Obstacle position in the left
      // Set rotation speed
      speed_rotation_angular_ = SPEED_TURN_ANGULAR * (1 - (((float) i) / (size / SCAN_RANGE)) / 2);
      return true;
    }
  }

  // Right range
  for (int i = size - 1; i > (size - (size / SCAN_RANGE)); i--) {
    if (last_scan_->ranges[i] < OBSTACLE_DISTANCE_THRESHOLD) {
      obstacle_position_ = 1;  // Obstacle position in the right
      // Set rotation speed
      speed_rotation_angular_ = - SPEED_TURN_ANGULAR * (1 - ((size - ((float) i)) / (size / SCAN_RANGE)) / 2);
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

float RandomWalkerNode::set_rotation_speed()
{
  // New method 
  // lc = l0 * cos (alpha * index);
  // c = l0 * sin (alpha * index);
  // beta = atan2(l-lc, c);
  // delta = atan2(c,lc)
  // gamma = beta + delta;
  // const gamma if it isn't object ended
  // float alpha = last_scan_->angle_increment;  // in rads
  float beta, delta, gamma;  // in rads
  float lc, c; 
  float precission = 0.2f;
  int size = last_scan_->ranges.size();

  if (obstacle_position_ == 1) {  // Object in right size
    // See left range
    // Get the first beta as reference
    lc = last_scan_->ranges[0] * cos(1);
    c = last_scan_->ranges[0] * sin(1);
    beta = atan2(last_scan_->ranges[1] - lc, c);  // In rads
    delta = atan2(c, lc);  // In rads
    float ref_gamma = beta + delta;  // In rads

    for (int i = 1; i < (size / SCAN_RANGE); i++) {
      lc = last_scan_->ranges[0] * cos(i);
      c = last_scan_->ranges[0] * sin(i);
      beta = atan2(last_scan_->ranges[i] - lc, c);  // In rads
      delta = atan2(c, lc);  // In rads
      gamma = beta + delta;  // In rads
      RCLCPP_INFO(get_logger(), "Check left size info : %d", i);
      RCLCPP_INFO(get_logger(), "Check left size info : l0 = %f", last_scan_->ranges[0]);
      RCLCPP_INFO(get_logger(), "Check left size info : %f of %f|%f|%f|%f",last_scan_->ranges[i] ,lc,c,beta,delta);
      RCLCPP_INFO(get_logger(), "Check left size info : %f of %f", gamma, ref_gamma);
      if ( !(ref_gamma + precission > gamma && ref_gamma - precission < gamma)) {
          RCLCPP_INFO(get_logger(), "Check left size out : %d", i);
          RCLCPP_INFO(get_logger(), "Check left size out : %f of %f", gamma, ref_gamma);
          return - SPEED_TURN_ANGULAR * (1 - (((float) i) / (size / SCAN_RANGE)) / 2);
      }

    }
    return - SPEED_TURN_ANGULAR / 2;

  } else if (obstacle_position_ == - 1) {  // Object in left size
    // See right range
    // Get the first beta as reference
    lc = last_scan_->ranges[0] * cos(size - 1);
    c = - last_scan_->ranges[0] * sin(size - 1);
    beta = atan2(last_scan_->ranges[(size -1)] - lc, c);  // In rads
    delta = atan2(c, lc);  // In rads
    float ref_gamma = beta + delta;  // In rads

    for (int i = size - 1; i > (size - (size / SCAN_RANGE)); i--) {

      lc = last_scan_->ranges[0] * cos(i);
      c = - last_scan_->ranges[0] * sin(i);
      beta = atan2(last_scan_->ranges[i] - lc, c);  // In rads
      delta = atan2(c, lc);  // In rads
      gamma = beta + delta;  // In rads

      RCLCPP_INFO(get_logger(), "Check right size info : %d", i);
      RCLCPP_INFO(get_logger(), "Check right size info : l0 = %f", last_scan_->ranges[0]);
      RCLCPP_INFO(get_logger(), "Check right size info : %f of %f|%f|%f|%f",last_scan_->ranges[i] ,lc,c,beta,delta);
      RCLCPP_INFO(get_logger(), "Check right size info : %f of %f", gamma, ref_gamma);

      if ( !(ref_gamma + precission > gamma && ref_gamma - precission < gamma)) {
          RCLCPP_INFO(get_logger(), "Check right size out : %d", i);
          RCLCPP_INFO(get_logger(), "Check right size out : %f of %f", gamma, ref_gamma);
          return SPEED_TURN_ANGULAR * (1 - ((size - ((float) i)) / (size / SCAN_RANGE)) / 2);
      }

    }
    return SPEED_TURN_ANGULAR / 2;
  }
  return 0.0f;  // Something went wrong
}

void RandomWalkerNode::set_rotation_time(float speed)
{
  if (speed < 0.0f) {
    speed *= -1;  // Make the time positive
  }
  RCLCPP_INFO(get_logger(), "Set time: %f", (SPEED_TURN_ANGULAR / speed) * 12);
  rclcpp::Duration ROTATING_TIME {(SPEED_TURN_ANGULAR / speed) * 12s};
}
