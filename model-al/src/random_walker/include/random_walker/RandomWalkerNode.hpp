#ifndef RANDOM_WALKER__RANDOMWALKER_HPP_
#define RANDOM_WALKER__RANDOMWALKER_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class RandomWalkerNode : public rclcpp::Node
{
  public:
    RandomWalkerNode();

  private:
    void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
    void control_cycle();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
    geometry_msgs::msg::Twist out_vel_;

    bool stopped_ = false;
};

#endif