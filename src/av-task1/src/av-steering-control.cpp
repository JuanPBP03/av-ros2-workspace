#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <random>

using namespace std::chrono_literals;

constexpr auto FREQ = 5;
constexpr auto MSG_PERIOD = 1.0s / FREQ;

class steeringController : public rclcpp::Node {
  public:
    steeringController() : Node("steering_controller") {
      publisher_ = this->create_publisher<std_msgs::msg::Float32>("steering_command", 10);

      speed_subscriber_ = this->create_subscription<std_msgs::msg::Float32>("vehicle_speed", 10,
      std::bind(&steeringController::speed_callback, this, std::placeholders::_1));

      position_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>("vehicle_position", 10,
      std::bind(&steeringController::position_callback, this, std::placeholders::_1));
      
      timer_ = this->create_wall_timer(MSG_PERIOD, std::bind(&steeringController::timer_callback, this));
    }
  private:
  bool speed_active = false;
  bool position_active = false;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  void speed_callback(const std_msgs::msg::Float32::SharedPtr speed){
    speed_active = true;
  }

  void position_callback(const geometry_msgs::msg::Point::SharedPtr position){ 
    position_active = true;
  }
  
  void timer_callback(){
    if (position_active && speed_active){
      std_msgs::msg::Float32 steering_command;
      steering_command.data = getRandomFloat(-100,100);
      RCLCPP_INFO(this->get_logger(), "Steering Command: %.2f",steering_command.data);
      publisher_->publish(steering_command);
      speed_active = false;
      position_active = false;
    }
  }

  float getRandomFloat(int min, int max){
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> dist(min, max);
    return dist(gen);
  }
  
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<steeringController>());
  rclcpp::shutdown();
  return 0;
}
