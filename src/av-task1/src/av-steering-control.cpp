#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <random>

using namespace std::chrono_literals;

constexpr auto FREQ = 5;
constexpr auto MSG_PERIOD = 1.0s / FREQ;

// Declare steeringController node class
class steeringController : public rclcpp::Node {
  public:
    // Name the node steering_controller using Node class constructor
  steeringController() : Node("steering_controller") {
    // Initialize publisher with topic steering_command
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("steering_command", 10);

    // Initialize subscriber to vehicle_speed topic and call speed_callback when topic is updated
    speed_subscriber_ = this->create_subscription<std_msgs::msg::Float32>("vehicle_speed", 10,
    std::bind(&steeringController::speed_callback, this, std::placeholders::_1));
    
    // Initialize subscriber to vehicle_position topic and call position_callback when topic is updated
    position_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>("vehicle_position", 10,
    std::bind(&steeringController::position_callback, this, std::placeholders::_1));
      
    // Initialize timer with period 1/5Hz and call timer_callback to publish steering_command when timer ends
    timer_ = this->create_wall_timer(MSG_PERIOD, std::bind(&steeringController::timer_callback, this));
    }
  private:
  bool speed_active = false;
  bool position_active = false;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // When speed message is received,
  // set speed_active to true to indicate vehicle_speed topic is updating
  void speed_callback(const std_msgs::msg::Float32::SharedPtr speed){
    speed_active = true;
  }

  // When position message is received,
  // set position_active to true to indicate vehicle_position topic is updating
  void position_callback(const geometry_msgs::msg::Point::SharedPtr position){ 
    position_active = true;
  }
  
  // Timer callback function to publish steering command
  void timer_callback(){
    // If vehicle_position and vehicle_speed topics have received new messages
    if (position_active && speed_active){
      std_msgs::msg::Float32 steering_command;
      steering_command.data = getRandomFloat(-100,100);
      // Print steering command to log
      RCLCPP_INFO(this->get_logger(), "Steering Command: %.2f",steering_command.data);
      // Publish steering command to steering_command topic
      publisher_->publish(steering_command);
      // Set vehicle_speed and vehicle_position activity to false
      speed_active = false;
      position_active = false;
    } 
    else { 
      // If no new speed or position messages were received since last callback,
      // set activity to false and print warning to log that it is waiting for speed and position updates
      speed_active = false;
      position_active = false;
      RCLCPP_WARN(this->get_logger(), "Waiting for speed and position updates...");
    }
  }
  // Function to seed random number generator and generate numbers
  float getRandomFloat(int min, int max){
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> dist(min, max);
    return dist(gen);
  }
  
};


int main(int argc, char ** argv)
{
  // Initialize ROS2 environment
  rclcpp::init(argc,argv);
  // Launch steeringController node and block program execution until exited
  rclcpp::spin(std::make_shared<steeringController>());
  rclcpp::shutdown();
  return 0;
}
