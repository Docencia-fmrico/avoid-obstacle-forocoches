#include "random_walker/RandomWalkerNode.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

RandomWalkerNode::RandomWalkerNode()
: Node("rand_walker_node")
{
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("input_scan", rclcpp::SensorDataQoS(),
    std::bind(&RandomWalkerNode::scan_callback, this, _1));
  timer_ = create_wall_timer(
    500ms, std::bind(&RandomWalkerNode::control_cycle, this));
}

void
RandomWalkerNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}


void 
RandomWalkerNode::control_cycle()
{

  // Do nothing until the first sensor read
  if (last_scan_ == nullptr) {
    return;
  }

  float c = 2;
  float a, b;

  if ( last_scan_->ranges[0] < 1.0f) {
    stopped_ = true;
  }

  if ( stopped_) {
    a = 0;
    b = 2;
  } else { 
    a = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))*c - (c/2);
    b = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))*c - (c/2);
  }
  out_vel_.linear.x = a;
  out_vel_.angular.z = b;
  vel_pub_->publish(out_vel_);
}