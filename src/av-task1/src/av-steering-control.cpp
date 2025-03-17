#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"
#include <random>

using namespace std::chrono_literals;

constexpr auto FREQ = 10;
constexpr auto MSG_PERIOD = 1.0s / FREQ;

class speedPublisher : public rclcpp::Node {
  public:
    speedPublisher() : Node("vehicle_speed_publisher") {
      publisher_ = this->create_publisher<std_msgs::msg::Float32>("vehicle_speed", 10);
      timer_ = this->create_wall_timer(MSG_PERIOD, std::bind(&speedPublisher::publishSpeed, this));
    }
  private:
  
  void publishSpeed(){
    std_msgs::msg::Float32 speed;
    speed.data = getSpeed();
    RCLCPP_INFO(this->get_logger(), "Vehicle Speed: %.2f", speed.data);
    publisher_->publish(speed);
  }

  float getSpeed(){ 
    return getRandomFloat(-100,100);
  }
  
  float getRandomFloat(int min, int max){
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> dist(min, max);
    return dist(gen);
  }
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<speedPublisher>());
  rclcpp::shutdown();
  return 0;
}
