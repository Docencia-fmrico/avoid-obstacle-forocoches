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

#include "avoid_obstacle_forocoches/AvoidObstacleNode.hpp"

using namespace std::chrono_literals; // NOLINT
using std::placeholders::_1;

AvoidObstacleNode::AvoidObstacleNode()
: Node("avoid_obstacle_node")
{
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("laser_marker", 1);
  debug_pub_ = create_publisher<DebugNode::DebugMessage>(DebugNode::TOPIC_NAME, 10);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(),
    std::bind(&AvoidObstacleNode::scan_callback, this, _1));

  button_sub_ = create_subscription<kobuki_ros_interfaces::msg::ButtonEvent>(
    "input_button", 10,
    std::bind(&AvoidObstacleNode::button_callback, this, _1));

  wheel_drop_sub_ = create_subscription<kobuki_ros_interfaces::msg::WheelDropEvent>(
    "input_wheel_drop", 10,
    std::bind(&AvoidObstacleNode::wheel_drop_callback, this, _1));

  bumper_sub_ = create_subscription<kobuki_ros_interfaces::msg::BumperEvent>(
    "input_bumper", 10,
    std::bind(&AvoidObstacleNode::bumper_callback, this, _1));

  timer_ = create_wall_timer(
    100ms, std::bind(&AvoidObstacleNode::control_cycle, this));

  // Declare parameter
  declare_parameter("SPEED_STOP_LINEAR", 0.0f);
  declare_parameter("SPEED_STOP_ANGULAR", 0.0f);
  declare_parameter("SPEED_FORWARD_LINEAR", 0.3f);
  declare_parameter("SPEED_FORWARD_ANGULAR", 0.0f);
  declare_parameter("SPEED_TURN_LINEAR", 0.0f);
  declare_parameter("SPEED_TURN_ANGULAR", 0.4f);
  declare_parameter("OBSTACLE_DISTANCE_THRESHOLD", 1.0f);

  // Retrieve parameters
  get_parameter("SPEED_STOP_LINEAR", SPEED_STOP_LINEAR);
  get_parameter("SPEED_STOP_ANGULAR", SPEED_STOP_ANGULAR);
  get_parameter("SPEED_FORWARD_LINEAR", SPEED_FORWARD_LINEAR);
  get_parameter("SPEED_FORWARD_ANGULAR", SPEED_FORWARD_ANGULAR);
  get_parameter("SPEED_TURN_LINEAR", SPEED_TURN_LINEAR);
  get_parameter("SPEED_TURN_ANGULAR", SPEED_TURN_ANGULAR);
  get_parameter("OBSTACLE_DISTANCE_THRESHOLD", OBSTACLE_DISTANCE_THRESHOLD);

  // Initialize the last state to stop
  last_state_ = STOP;
}

void AvoidObstacleNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

void AvoidObstacleNode::button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg)
{
  last_button_pressed_ = std::move(msg);
  if (last_button_pressed_->button == kobuki_ros_interfaces::msg::ButtonEvent::BUTTON1 &&
    last_button_pressed_->state == 1)
  {
    button_pressed_ = !button_pressed_;
  }
}

void AvoidObstacleNode::wheel_drop_callback(
  kobuki_ros_interfaces::msg::WheelDropEvent::UniquePtr msg)
{
  last_wheel_dropped_ = std::move(msg);
  // Not on the ground
  kobuki_not_on_ground_ = last_wheel_dropped_->state == kobuki_ros_interfaces::msg::WheelDropEvent::DROPPED;
}

void AvoidObstacleNode::bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg)
{
  last_bumper_detected_ = std::move(msg);
}

void AvoidObstacleNode::control_cycle()
{
  // Do nothing until the first sensor read
  if (last_scan_ == nullptr) {
    debug_msg_.data = DebugNode::ERROR;
    debug_pub_->publish(debug_msg_);
    return;
  }

  debug_msg_.data = DebugNode::OK;
  print_markers();
  // FSM
  switch (state_) {
    case STOP:
      out_vel_.linear.x = SPEED_STOP_LINEAR;
      out_vel_.angular.z = SPEED_STOP_ANGULAR;
      if (stopped_with_error_) {
        if (kobuki_not_on_ground_) {debug_msg_.data = DebugNode::NOT_ON_GROUND;}
        else {debug_msg_.data = DebugNode::ERROR;}
      } else {
        // Waiting for button press
        debug_msg_.data = DebugNode::READY;
      }
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
      // Extreme case, go backwards
      if (check_2_backward()) {
        change_state(BACKWARD);
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
      // Extreme case, go backwards
      if (check_2_backward()) {
        change_state(BACKWARD);
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
        // Check if the turn happened faster then expected
        if (check_rotation_2_turn_time()) {
          // Lessen the laser range
          current_range++;
        } else {
          // Make the laser range bigger
          current_range--;
          if (current_range < MIN_LASER_RANGE) {current_range = MIN_LASER_RANGE;}
        }
        change_state(TURN);
      }
      // Extreme case, go backwards
      if (check_2_backward()) {
        change_state(BACKWARD);
      }
      break;
    case BACKWARD:
      // Only used in cases where the laser cannot detect the object
      out_vel_.linear.x = -SPEED_FORWARD_LINEAR;
      out_vel_.angular.z = SPEED_FORWARD_ANGULAR;
      // Can go to stop or to turn
      if (check_2_stop()) {
        change_state(STOP);
      }
      if (check_backward_2_turn()) {
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
  marker_pub_->publish(marker_msg_);
  vel_pub_->publish(out_vel_);
  debug_pub_->publish(debug_msg_);
}

// FSM states
void AvoidObstacleNode::change_state(int new_state)
{
  last_state_ = state_;
  state_ = new_state;
  state_timestamp_ = now();
}

// FSM changes
bool AvoidObstacleNode::check_stop_2_last()
{
  // Return if everything works as intended
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  stopped_with_error_ = elapsed > LASER_SCAN_TIMEOUT || kobuki_not_on_ground_;
  return !stopped_with_error_ && button_pressed_;
}

bool AvoidObstacleNode::check_2_stop()
{
  // Stop if something is wrong
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  stopped_with_error_ = elapsed > LASER_SCAN_TIMEOUT || kobuki_not_on_ground_;
  return stopped_with_error_ || !button_pressed_;
}

bool AvoidObstacleNode::check_turn_2_rotation()
{
  // Start rotation when it finishes turning
  return (now() - state_timestamp_) > TURNING_TIME;
}

bool AvoidObstacleNode::check_2_turn()
{
  // Always size/4 to -size/4 Min_threshold
  // The rest is Min_threshold if i == size / current_range
  // i == 0, always 1 meter
  // Turn if the laser detects an object near

  int size = last_scan_->ranges.size();
  reduced_threshold_ = 0.0f;
  reduced_non_detection_threshold_ = MIN_THRESHOLD - 0.03f * (current_range / 5);
  if (reduced_non_detection_threshold_ <= NON_DETECTION_THRESHOLD) {
    reduced_non_detection_threshold_ = NON_DETECTION_THRESHOLD + 0.01f;
  }

  // Check if the bumper has been triggered
  if (last_bumper_detected_ != nullptr &&
    last_bumper_detected_->state == last_bumper_detected_->PRESSED)
  {
    // Set rotation speed
    if (last_bumper_detected_->bumper == last_bumper_detected_->RIGHT) {
      obstacle_position_ = 1;  // Obstacle position in the right
      speed_rotation_angular_ = -SPEED_TURN_ANGULAR;
    } else {
      obstacle_position_ = -1;  // Obstacle position in the left
      speed_rotation_angular_ = SPEED_TURN_ANGULAR;
    }
    return true;
  }

  // Left range
  for (int i = 0; i < (size / 2); i++) {
    reduced_threshold_ = OBSTACLE_DISTANCE_THRESHOLD -
      (OBSTACLE_DISTANCE_THRESHOLD - reduced_non_detection_threshold_) *
      i / (size / current_range);

    if (reduced_threshold_ < reduced_non_detection_threshold_) {
      reduced_threshold_ = reduced_non_detection_threshold_;
    }
    if (last_scan_->ranges[i] < reduced_threshold_ &&
      last_scan_->ranges[i] > NON_DETECTION_THRESHOLD)
    {
      obstacle_position_ = -1;  // Obstacle position in the left
      // Set rotation speed
      speed_rotation_angular_ = SPEED_TURN_ANGULAR * (1 - (static_cast<float>(i) / (size / 2)) / 2);
      return true;
    }
  }

  // Right range
  for (int i = size - 1; i > (size - (size / 2)); i--) {
    reduced_threshold_ = OBSTACLE_DISTANCE_THRESHOLD -
      (OBSTACLE_DISTANCE_THRESHOLD - reduced_non_detection_threshold_) *
      (size - i) / (size / current_range);

    if (reduced_threshold_ < reduced_non_detection_threshold_) {
      reduced_threshold_ = reduced_non_detection_threshold_;
    }
    if (last_scan_->ranges[i] < reduced_threshold_ &&
      last_scan_->ranges[i] > NON_DETECTION_THRESHOLD)
    {
      obstacle_position_ = 1;  // Obstacle position in the right
      // Set rotation speed
      speed_rotation_angular_ = -SPEED_TURN_ANGULAR *
        (1 - ((size - static_cast<float>(i)) / (size / 2)) / 2);
      marker_msg_.markers[i].color.b = 0.0f;
      marker_msg_.markers[i].lifetime = rclcpp::Duration(5s);
      return true;
    }
  }
  return false;
}

bool AvoidObstacleNode::check_rotation_2_forward()
{
  // Go forward when it finishes rotating
  return (now() - state_timestamp_) > ROTATING_TIME;
}

bool AvoidObstacleNode::check_rotation_2_turn_time()
{
  // Go forward when it finishes rotating
  return (now() - state_timestamp_) > MIN_ROTATING_TIME;
}

void AvoidObstacleNode::set_rotation_time(float speed)
{
  if (speed < 0.0f) {
    speed *= -1;  // Make the time positive
  }
  rclcpp::Duration ROTATING_TIME {(SPEED_TURN_ANGULAR / speed) * 12s};
}

bool AvoidObstacleNode::check_2_backward()
{
  // This shold never happend under normal conditions
  // Check if the bumper has been triggered
  if (last_bumper_detected_ != nullptr &&
    last_bumper_detected_->state == last_bumper_detected_->PRESSED)
  {
    // Set rotation speed
    if (last_bumper_detected_->bumper == last_bumper_detected_->RIGHT) {
      obstacle_position_ = 1;  // Obstacle position in the right
      speed_rotation_angular_ = -SPEED_TURN_ANGULAR;
    } else {
      obstacle_position_ = -1;  // Obstacle position in the left
      speed_rotation_angular_ = SPEED_TURN_ANGULAR;
    }
    return true;
  }
  return false;
}

bool AvoidObstacleNode::check_backward_2_turn()
{
  // Start rotation when it finishes turning
  return (now() - state_timestamp_) > BACKWARD_TIME;
}

void AvoidObstacleNode::print_markers(){
  int size = last_scan_->ranges.size();
  float alpha = last_scan_->angle_increment;

  marker_msg_.markers.clear();
  for (int i = 0; i < size; i++) {
    if (i < size / 2) {
      reduced_threshold_ = OBSTACLE_DISTANCE_THRESHOLD -
        (OBSTACLE_DISTANCE_THRESHOLD - reduced_non_detection_threshold_) * i / (size / current_range);
    } else {
      reduced_threshold_ = OBSTACLE_DISTANCE_THRESHOLD -
        (OBSTACLE_DISTANCE_THRESHOLD - reduced_non_detection_threshold_) * (size - i) / (size / current_range);
    }
    if (reduced_threshold_ < reduced_non_detection_threshold_) {reduced_threshold_ = reduced_non_detection_threshold_;}
    // Publish and create the markers
    // First for detection
    marker_msg_.markers.push_back(set_marker(alpha * i, reduced_threshold_, i));
    // Second for range
    visualization_msgs::msg::Marker range_marker = set_marker(alpha * i, OBSTACLE_DISTANCE_THRESHOLD, i + size);
    range_marker.color.r = 0.0f;
    marker_msg_.markers.push_back(range_marker);
    // Third for min range
    visualization_msgs::msg::Marker min_range_marker = set_marker(alpha * i, NON_DETECTION_THRESHOLD, i + size * 2);
    min_range_marker.color.r = 0.0f;
    marker_msg_.markers.push_back(min_range_marker);
  }
}

visualization_msgs::msg::Marker AvoidObstacleNode::set_marker(float alpha, float distance, int id)
{
  // Set the markers for rviz
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = now();

  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration(100ms);
  marker.id = id;

  // Get x and y coordinates from alpha and distance
  double x = cos(alpha) * distance;
  double y = sin(alpha) * distance;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 1;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;

  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0; // Don't forget to set the alpha!

  return marker;
}
