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
         std::bind(&steeringController::speed_active,this));
      position_subscriber_ = this->create_subscription<ge
    }
  private:
  bool speed_active = false;
  bool position_active = false;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  void speed_callback(){
   speed_active = true;
  }

  void position_callback(){ 
    position_active = true;
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
