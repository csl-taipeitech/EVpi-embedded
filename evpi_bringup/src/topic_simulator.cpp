#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

using std::placeholders::_1;

class TopicSimulator : public rclcpp::Node
{
  public:
    TopicSimulator()
    : Node("topic_simulator"), count_(0)
    {
      can_publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", 5);
      led_publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("led_strips", 5);

      joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&TopicSimulator::joy_callback, this, _1));

    }

  private:

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
    {
        static auto last_joy_msgs = sensor_msgs::msg::Joy();
        static bool first_time = true;
        static auto led_msgs = std_msgs::msg::Int16MultiArray();
        led_msgs.data={513,1,10,20,30};


        if(first_time)
        {
            last_joy_msgs.buttons = msg->buttons;
            first_time=false;
        }

        if(msg->buttons[0] != last_joy_msgs.buttons[0])
        {
            RCLCPP_INFO(this->get_logger(), "Blue button");
            led_msgs.data={513,1,10,20,30};
            led_publisher_->publish(led_msgs);
        }
        else if(msg->buttons[1] != last_joy_msgs.buttons[1])
        {
            RCLCPP_INFO(this->get_logger(), "Green button");
            led_msgs.data={513,1,10,60,10};
            led_publisher_->publish(led_msgs);
        }
        else if(msg->buttons[2] != last_joy_msgs.buttons[2])
        {
            RCLCPP_INFO(this->get_logger(), "Red button");
            led_msgs.data={513,1,50,20,10};
            led_publisher_->publish(led_msgs);
        }
        else if(msg->buttons[3] != last_joy_msgs.buttons[3])
        {
            RCLCPP_INFO(this->get_logger(), "Yellow button");
            led_msgs.data={513,1,50,40,0};
            led_publisher_->publish(led_msgs);
        }
        else if(msg->buttons[4] != last_joy_msgs.buttons[4])
        {
            RCLCPP_INFO(this->get_logger(), "LB button");
            led_msgs.data[1] = 4; //breath mode
            led_msgs.data[5] = 1; //period in second 
            led_publisher_->publish(led_msgs);
        }
        else if(msg->buttons[5] != last_joy_msgs.buttons[5])
        {
            RCLCPP_INFO(this->get_logger(), "RB button");
            led_msgs.data[1] = 3; //breath mode
            led_msgs.data[5] = 1; //period in second 
            led_publisher_->publish(led_msgs);
        }
        else if(msg->buttons[6] != last_joy_msgs.buttons[6])
        {
            RCLCPP_INFO(this->get_logger(), "LT button");
            led_msgs.data={513,1,0,0,0};
            led_publisher_->publish(led_msgs);
        }
        else if(msg->buttons[7] != last_joy_msgs.buttons[7])
        {
          RCLCPP_INFO(this->get_logger(), "RT button");
        }


        last_joy_msgs.buttons = msg->buttons;
        // RCLCPP_INFO(this->get_logger(), "I heard: '%d',%d,%d,%d", 
    }
    
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr led_publisher_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicSimulator>());
  rclcpp::shutdown();
  return 0;
}