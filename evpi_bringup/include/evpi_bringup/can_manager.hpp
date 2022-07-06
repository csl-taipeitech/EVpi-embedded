#ifndef CAN_MANAGER_HPP_
#define CAN_MANAGER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
auto sensor_can_msgs = can_msgs::msg::Frame();

int strip_mode;
int strip_red;
int sync_interval_;
bool is_steering_limit_ = false;

struct can_t {
    int can_id;
    int mode;
    int pixels;
    int red_value;
    int green_value;
    int blue_value;
    int time_interval;
    int dlc;
    bool is_rtr;
    bool is_extended;
    bool is_error;
    int duty;
};

struct can_t eyes_strip; 
struct can_t front_strip_l_; 
struct can_t front_strip_r_; 

struct can_t back_strip; 
struct can_t steering_angle_; 
struct can_t brake_servo_; 
struct can_t sync_can_msgs_; 
struct can_t steering_motor_; 

/////////////////////////////////////////////////////////////////////////////////////

class CanManager : public rclcpp::Node
{
public:
  CanManager();
  virtual ~CanManager();
  
private:
  void get_params();
  void init_can_mode();
  void timer_callback();
  void steering_motor_callback(const std_msgs::msg::Float32::SharedPtr msg);   
  void strip_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);   
  void brake_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void can_callback(const can_msgs::msg::Frame::SharedPtr msg);
  float steering_mapping(float raw);

  rclcpp::TimerBase::SharedPtr sync_timer_;
  
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_motor_subscription_; 
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr strip_subscription_; 
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr brake_subscription_; 
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscription_; 

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_angle_publisher_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
};

#endif  // CAN_MANAGER_HPP_