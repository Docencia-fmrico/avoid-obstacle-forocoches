#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
class BaseController : public rclcpp::Node
{

public:

  enum Direction {
    LEFT = 1, 
    RIGHT = -1,
    NULLSIDE
  };

  enum FSM_Status {
    FORWARD,
    ROTATING,
    AVOIDING,
    RELOCATING
  };

  BaseController();
  void subscription_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void timer_callback();
  void timer_callback_radar();
  
private:
  
  std::vector<bool> sectorize(std::vector<float>& data, float sectors, float range);
  bool obstacleAnalize(std::vector<bool>& status,int sensivity);
  Direction calculate_side_to_avoid_obstacle(std::vector<bool>& status);
  void setIntensities(std::vector<bool>& frontal,std::vector<bool>& avoid,std::vector<bool>& danger);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisherRadar_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timerRadar_;
  geometry_msgs::msg::Twist motion_;
  sensor_msgs::msg::LaserScan radar_;
  Direction AVOID_SIDE_BY_DEFAULT_;

  // Kobuki compatiblity
  bool isKobuki = false;
  std::vector<float> translate_kobuki_scan(std::vector<float> ranges);


  // Radar variables
  float angle_min;
  float angle_max;
  std::vector<float> ranges_;
  std::vector<float> intensities_;

  // FSM Variables
  FSM_Status status_ = FORWARD;
  bool obstacleInFront_;
  bool obstacleInSide_;
  bool obstacleInDanger_;
  Direction sideToAvoidObstacle_ = RIGHT;
  Direction sideCompleted_;
  Direction lastSideRotated_ = NULLSIDE;
  rclcpp::Time end_time_;
  int try_to_unlock_;
  int lock_remaining_;

  // Machine Parameters
  float velocity_;
  float recolocation_delay_;
  float rotation_smoothness_;
  float radious_;
  int seconds_rotating_;
  int seconds_recolocating_;
  float velocity_delay_;
  int scan_resolution_;
  float frontal_range_;
  float avoiding_range_;
  float danger_range_;
  int frontal_resolution_;
  int lateral_resolution_; 

};


