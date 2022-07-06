#include "evpi_bringup/can_manager.hpp"

CanManager::CanManager()
: Node("can_manager")
{
    get_params();

    steering_angle_publisher_ = this->create_publisher<std_msgs::msg::Float32>("steering_angle", 3);  
    can_publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_rx", 3);  
    
    init_can_mode();

    sync_timer_ = 
      this->create_wall_timer
      (
      100ms, std::bind(&CanManager::timer_callback, this)
      );

    steering_motor_subscription_ = 
      this->create_subscription<std_msgs::msg::Float32>
      (          
      "steering_motor", 3, 
      std::bind(&CanManager::steering_motor_callback,
      this,
      std::placeholders::_1)
      );

    strip_subscription_ = 
      this->create_subscription<std_msgs::msg::Int16MultiArray>
      (          
      "led_strips", 3, 
      std::bind(&CanManager::strip_callback,
      this,
      std::placeholders::_1)
      );

    brake_subscription_ = 
      this->create_subscription<std_msgs::msg::Int32>
      (          
      "brake_servo", 2, 
      std::bind(&CanManager::brake_callback,
      this,
      std::placeholders::_1)
      );

    can_subscription_ = 
      this->create_subscription<can_msgs::msg::Frame>
      (          
      "can_tx", 3, 
      std::bind(&CanManager::can_callback,
      this,
      std::placeholders::_1)
      );



    // sensor_can_msgs.id = 0x30A;
    sensor_can_msgs.dlc = 4;
    sensor_can_msgs.is_rtr = false;
    sensor_can_msgs.is_extended = false;
    sensor_can_msgs.is_error = false;
}

CanManager::~CanManager()
{
}

void CanManager::get_params()
{
    RCLCPP_INFO(this->get_logger(), "Process Params!");

    this->declare_parameter("sync_signal.time_interval", 100);
    this->declare_parameter("sync_signal.can_id", 600);
    this->declare_parameter("sync_signal.dlc", 4);
    this->declare_parameter("sync_signal.is_rtr", false);
    this->declare_parameter("sync_signal.is_extended", false);
    this->declare_parameter("sync_signal.is_error", false);

    this->declare_parameter("brake.can_id", 0);
    this->declare_parameter("brake.mode", 0);
    this->declare_parameter("brake.dlc", 0);
    this->declare_parameter("brake.is_rtr", false);
    this->declare_parameter("brake.is_extended", false);
    this->declare_parameter("brake.is_error", false);

    this->declare_parameter("steering_motor.duty", 0);
    this->declare_parameter("steering_motor.can_id", 0);
    this->declare_parameter("steering_motor.mode", 0);
    this->declare_parameter("steering_motor.dlc", 0);
    this->declare_parameter("steering_motor.is_rtr", false);
    this->declare_parameter("steering_motor.is_extended", false);
    this->declare_parameter("steering_motor.is_error", false);

    this->declare_parameter("steering_angle.can_id", 0);
    this->declare_parameter("steering_angle.mode", 0);
    this->declare_parameter("steering_angle.dlc", 0);
    this->declare_parameter("steering_angle.is_rtr", false);
    this->declare_parameter("steering_angle.is_extended", false);
    this->declare_parameter("steering_angle.is_error", false);

    this->declare_parameter("eyes_strip.can_id", 0);
    this->declare_parameter("eyes_strip.mode", 0);
    this->declare_parameter("eyes_strip.pixels", 10);
    this->declare_parameter("eyes_strip.dlc", 4);
    this->declare_parameter("eyes_strip.is_rtr", false);
    this->declare_parameter("eyes_strip.is_extended", false);
    this->declare_parameter("eyes_strip.is_error", false);
    this->declare_parameter("eyes_strip.time_interval", 69);

    this->declare_parameter("front_strip_l.can_id", 0);
    this->declare_parameter("front_strip_l.mode", 0);
    this->declare_parameter("front_strip_l.pixels", 10);
    this->declare_parameter("front_strip_l.dlc", 4);
    this->declare_parameter("front_strip_l.is_rtr", false);
    this->declare_parameter("front_strip_l.is_extended", false);
    this->declare_parameter("front_strip_l.is_error", false);    
    this->declare_parameter("front_strip_l.time_interval", 69);

    this->declare_parameter("front_strip_r.can_id", 0);
    this->declare_parameter("front_strip_r.mode", 0);
    this->declare_parameter("front_strip_r.pixels", 10);
    this->declare_parameter("front_strip_r.dlc", 4);
    this->declare_parameter("front_strip_r.is_rtr", false);
    this->declare_parameter("front_strip_r.is_extended", false);
    this->declare_parameter("front_strip_r.is_error", false);    
    this->declare_parameter("front_strip_r.time_interval", 69);

    this->declare_parameter("back_strip.can_id", 0);
    this->declare_parameter("back_strip.mode", 0);
    this->declare_parameter("back_strip.pixels", 10);
    this->declare_parameter("back_strip.dlc", 4);
    this->declare_parameter("back_strip.is_rtr", false);
    this->declare_parameter("back_strip.is_extended", false);
    this->declare_parameter("back_strip.is_error", false);   
    this->declare_parameter("back_strip.time_interval", 69);  

    this->get_parameter<int>("sync_signal.time_interval", sync_interval_);
    this->get_parameter<int>("sync_signal.can_id", sync_can_msgs_.can_id);
    this->get_parameter<int>("sync_signal.dlc", sync_can_msgs_.dlc);
    this->get_parameter<bool>("sync_signal.is_rtr", sync_can_msgs_.is_rtr);
    this->get_parameter<bool>("sync_signal.is_extended", sync_can_msgs_.is_extended);
    this->get_parameter<bool>("sync_signal.is_error", sync_can_msgs_.is_error);

    this->get_parameter<int>("brake.can_id", brake_servo_.can_id);
    this->get_parameter<int>("brake.mode", brake_servo_.mode);
    this->get_parameter<int>("brake.dlc", brake_servo_.dlc);
    this->get_parameter<bool>("brake.is_rtr", brake_servo_.is_rtr);
    this->get_parameter<bool>("brake.is_extended", brake_servo_.is_extended);
    this->get_parameter<bool>("brake.is_error", brake_servo_.is_error);

    this->get_parameter<int>("steering_motor.duty", steering_motor_.duty);
    this->get_parameter<int>("steering_motor.can_id", steering_motor_.can_id);
    this->get_parameter<int>("steering_motor.mode", steering_motor_.mode);
    this->get_parameter<int>("steering_motor.dlc", steering_motor_.dlc);
    this->get_parameter<bool>("steering_motor.is_rtr", steering_motor_.is_rtr);
    this->get_parameter<bool>("steering_motor.is_extended", steering_motor_.is_extended);
    this->get_parameter<bool>("steering_motor.is_error", steering_motor_.is_error);

    this->get_parameter<int>("steering_angle.can_id", steering_angle_.can_id);
    this->get_parameter<int>("steering_angle.mode", steering_angle_.mode);
    this->get_parameter<int>("steering_angle.dlc", steering_angle_.dlc);
    this->get_parameter<bool>("steering_angle.is_rtr", steering_angle_.is_rtr);
    this->get_parameter<bool>("steering_angle.is_extended", steering_angle_.is_extended);
    this->get_parameter<bool>("steering_angle.is_error", steering_angle_.is_error);

    this->get_parameter<int>("eyes_strip.can_id", eyes_strip.can_id);
    this->get_parameter<int>("eyes_strip.mode", eyes_strip.mode);
    this->get_parameter<int>("eyes_strip.pixels", eyes_strip.pixels);
    this->get_parameter<int>("eyes_strip.dlc", eyes_strip.dlc);
    this->get_parameter<bool>("eyes_strip.is_rtr", eyes_strip.is_rtr);
    this->get_parameter<bool>("eyes_strip.is_extended", eyes_strip.is_extended);
    this->get_parameter<bool>("eyes_strip.is_error", eyes_strip.is_error);    
    this->get_parameter<int>("eyes_strip.time_interval", eyes_strip.time_interval);

    this->get_parameter<int>("front_strip_l.can_id", front_strip_l_.can_id);
    this->get_parameter<int>("front_strip_l.mode", front_strip_l_.mode);
    this->get_parameter<int>("front_strip_l.pixels", front_strip_l_.pixels);
    this->get_parameter<int>("front_strip_l.dlc", front_strip_l_.dlc);
    this->get_parameter<bool>("front_strip_l.is_rtr", front_strip_l_.is_rtr);
    this->get_parameter<bool>("front_strip_l.is_extended", front_strip_l_.is_extended);
    this->get_parameter<bool>("front_strip_l.is_error", front_strip_l_.is_error);  
    this->get_parameter<int>("front_strip_l.time_interval", front_strip_l_.time_interval);   

    this->get_parameter<int>("front_strip_r.can_id", front_strip_r_.can_id);
    this->get_parameter<int>("front_strip_r.mode", front_strip_r_.mode);
    this->get_parameter<int>("front_strip_r.pixels", front_strip_r_.pixels);
    this->get_parameter<int>("front_strip_r.dlc", front_strip_r_.dlc);
    this->get_parameter<bool>("front_strip_r.is_rtr", front_strip_r_.is_rtr);
    this->get_parameter<bool>("front_strip_r.is_extended", front_strip_r_.is_extended);
    this->get_parameter<bool>("front_strip_r.is_error", front_strip_r_.is_error);  
    this->get_parameter<int>("front_strip_r.time_interval", front_strip_r_.time_interval);   


    this->get_parameter<int>("back_strip.can_id", back_strip.can_id);
    this->get_parameter<int>("back_strip.mode", back_strip.mode);
    this->get_parameter<int>("back_strip.pixels", back_strip.pixels);
    this->get_parameter<int>("back_strip.dlc", back_strip.dlc);
    this->get_parameter<bool>("back_strip.is_rtr", back_strip.is_rtr);
    this->get_parameter<bool>("back_strip.is_extended", back_strip.is_extended);
    this->get_parameter<bool>("back_strip.is_error", back_strip.is_error);  
    this->get_parameter<int>("back_strip.time_interval", back_strip.time_interval);      

    RCLCPP_INFO(this->get_logger(), "Params Prepared!");
}

void CanManager::init_can_mode()
{
    auto init_can_msgs = can_msgs::msg::Frame();
    
    // Initiate Steering Motor
    init_can_msgs.id = steering_motor_.can_id;
    init_can_msgs.dlc = steering_motor_.dlc;
    init_can_msgs.is_rtr = steering_motor_.is_rtr;
    init_can_msgs.is_extended = steering_motor_.is_extended;
    init_can_msgs.is_error = steering_motor_.is_error;
    init_can_msgs.data = {steering_motor_.mode, 0, 0};
    can_publisher_->publish(init_can_msgs);  
    RCLCPP_INFO(this->get_logger(), "Finish: Initiate Steering Motor!");


    // Initiate Brake Servo
    init_can_msgs.id = brake_servo_.can_id;
    init_can_msgs.dlc = brake_servo_.dlc;
    init_can_msgs.is_rtr = brake_servo_.is_rtr;
    init_can_msgs.is_extended = brake_servo_.is_extended;
    init_can_msgs.is_error = brake_servo_.is_error;
    init_can_msgs.data = {brake_servo_.mode};
    can_publisher_->publish(init_can_msgs);  
    RCLCPP_INFO(this->get_logger(), "Finish: Initiate Brake Servo!");

    // Initiate Steering Encoder
    init_can_msgs.id = steering_angle_.can_id;
    init_can_msgs.dlc = steering_angle_.dlc;
    init_can_msgs.is_rtr = steering_angle_.is_rtr;
    init_can_msgs.is_extended = steering_angle_.is_extended;
    init_can_msgs.is_error = steering_angle_.is_error;
    init_can_msgs.data = {steering_angle_.mode};
    can_publisher_->publish(init_can_msgs);  
    RCLCPP_INFO(this->get_logger(), "Finish: Initiate Steering Encoder!");
}

void CanManager::timer_callback()
{
    static auto sync = can_msgs::msg::Frame();

    sync.id = sync_can_msgs_.can_id;
    sync.dlc = sync_can_msgs_.dlc;
    sync.is_error = sync_can_msgs_.is_error;
    sync.is_extended = sync_can_msgs_.is_extended;
    sync.is_rtr = sync_can_msgs_.is_rtr;

    can_publisher_->publish(sync);    
}

void CanManager::steering_motor_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data); 
      sensor_can_msgs.id = steering_motor_.can_id;
      sensor_can_msgs.dlc = steering_motor_.dlc;
      sensor_can_msgs.is_error = steering_motor_.is_error;
      sensor_can_msgs.is_extended = steering_motor_.is_extended;
      sensor_can_msgs.is_rtr = steering_motor_.is_rtr;
      float motor_cmd = msg->data;

      if(motor_cmd == 0.00)
      {
        sensor_can_msgs.data = {steering_motor_.mode, 0, 0};
        RCLCPP_INFO(this->get_logger(), "Nidec motor now stop moving..");
      }
      else if(motor_cmd > 0.00)
      {
        sensor_can_msgs.data = {steering_motor_.mode, motor_cmd*steering_motor_.duty, 0};
        RCLCPP_INFO(this->get_logger(), "Nidec motor is moving at duty= %f ", motor_cmd*steering_motor_.duty);
      }
      else
      {
        sensor_can_msgs.data = {steering_motor_.mode, abs(motor_cmd)*steering_motor_.duty, 1};
        RCLCPP_INFO(this->get_logger(), "Nidec motor is moving at duty= %f ", abs(motor_cmd)*steering_motor_.duty);
      }


    can_publisher_->publish(sensor_can_msgs);  

}

void CanManager::strip_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data[0]); 

    if(eyes_strip.can_id == msg->data[0])
    {
      eyes_strip.mode = msg->data[1];
      eyes_strip.red_value = msg->data[2];
      eyes_strip.green_value = msg->data[3];
      eyes_strip.blue_value = msg->data[4];

      sensor_can_msgs.id = eyes_strip.can_id;
      sensor_can_msgs.dlc = eyes_strip.dlc;
      sensor_can_msgs.is_error = eyes_strip.is_error;
      sensor_can_msgs.is_extended = eyes_strip.is_extended;
      sensor_can_msgs.is_rtr = eyes_strip.is_rtr;
      sensor_can_msgs.data = {eyes_strip.mode, eyes_strip.pixels, eyes_strip.red_value,
                              eyes_strip.green_value, eyes_strip.blue_value, 
                              eyes_strip.time_interval};
    }
    else if(front_strip_l_.can_id == msg->data[0])
    {
      front_strip_l_.mode = msg->data[1];
      front_strip_l_.red_value = msg->data[2];
      front_strip_l_.green_value = msg->data[3];
      front_strip_l_.blue_value = msg->data[4];

      sensor_can_msgs.id = front_strip_l_.can_id;
      sensor_can_msgs.dlc = front_strip_l_.dlc;
      sensor_can_msgs.is_error = front_strip_l_.is_error;
      sensor_can_msgs.is_extended = front_strip_l_.is_extended;
      sensor_can_msgs.is_rtr = front_strip_l_.is_rtr;
      sensor_can_msgs.data = {front_strip_l_.mode, front_strip_l_.pixels, front_strip_l_.red_value,
                              front_strip_l_.green_value, front_strip_l_.blue_value, 
                              front_strip_l_.time_interval};
    }
    else if(front_strip_r_.can_id == msg->data[0])
    {
      front_strip_r_.mode = msg->data[1];
      front_strip_r_.red_value = msg->data[2];
      front_strip_r_.green_value = msg->data[3];
      front_strip_r_.blue_value = msg->data[4];

      sensor_can_msgs.id = front_strip_r_.can_id;
      sensor_can_msgs.dlc = front_strip_r_.dlc;
      sensor_can_msgs.is_error = front_strip_r_.is_error;
      sensor_can_msgs.is_extended = front_strip_r_.is_extended;
      sensor_can_msgs.is_rtr = front_strip_r_.is_rtr;
      sensor_can_msgs.data = {front_strip_r_.mode, front_strip_r_.pixels, front_strip_r_.red_value,
                              front_strip_r_.green_value, front_strip_r_.blue_value, 
                              front_strip_r_.time_interval};
    }
    else if(back_strip.can_id == msg->data[0])
    {
      back_strip.mode = msg->data[1];
      back_strip.red_value = msg->data[2];
      back_strip.green_value = msg->data[3];
      back_strip.blue_value = msg->data[4];

      sensor_can_msgs.id = back_strip.can_id;
      sensor_can_msgs.dlc = back_strip.dlc;
      sensor_can_msgs.is_error = back_strip.is_error;
      sensor_can_msgs.is_extended = back_strip.is_extended;
      sensor_can_msgs.is_rtr = back_strip.is_rtr;
      sensor_can_msgs.data = {back_strip.mode, back_strip.pixels, back_strip.red_value,
                              back_strip.green_value, back_strip.blue_value, 
                              back_strip.time_interval};
    }

    can_publisher_->publish(sensor_can_msgs);    
}

void CanManager::brake_callback(const std_msgs::msg::Int32::SharedPtr msg)       
{
      sensor_can_msgs.id = brake_servo_.can_id;
      sensor_can_msgs.dlc = brake_servo_.dlc;
      sensor_can_msgs.is_error = brake_servo_.is_error;
      sensor_can_msgs.is_extended = brake_servo_.is_extended;
      sensor_can_msgs.is_rtr = brake_servo_.is_rtr;
      sensor_can_msgs.data = {brake_servo_.mode, msg->data};
    can_publisher_->publish(sensor_can_msgs);    
}

void CanManager::can_callback(const can_msgs::msg::Frame::SharedPtr msg)       
{
    if(steering_angle_.can_id == msg->id)
    {
      static uint32_t adc_val;
      static auto st_angle = std_msgs::msg::Float32();
      
      float max_radian = M_PI/4, min_radian = -M_PI/4;
      float max_adc = 4095, min_adc = 0;
      static float str_data;

      adc_val = msg->data[1] + 256*msg->data[2];
      str_data = adc_val*(max_radian-min_radian)/(max_adc-min_adc)+min_radian;

      st_angle.data = steering_mapping(str_data);
      // RCLCPP_INFO(this->get_logger(), "steering_raw= %f, steering_map= %f",str_data, st_angle.data); 

      steering_angle_publisher_->publish(st_angle);

    }
}

float CanManager::steering_mapping(float raw)       
{
  // read value from encoder
  float raw_upper = 0.655;
  float raw_mid= 0.115;
  float raw_lower= -0.425;

  // true radian regarding degree
  float true_upper = 0.698;
  float true_mid= 0.00;
  float true_lower= -0.698;

  return (raw-raw_mid)/(raw_upper-raw_lower)*(true_upper-true_lower)+true_mid;
}
/////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CanManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}