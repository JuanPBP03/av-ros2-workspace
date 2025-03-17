#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point.hpp"
#include <random>

using namespace std::chrono_literals;

constexpr auto FREQ = 10;
constexpr auto MSG_PERIOD = 1.0s / FREQ;

class positionPublisher : public rclcpp::Node {
  public:
    positionPublisher() : Node("vehicle_position_publisher") {
      publisher_ = this->create_publisher<geometry_msgs::msg::Point>("vehicle_position", 10);
      timer_ = this->create_wall_timer(MSG_PERIOD, std::bind(&positionPublisher::publishPosition, this));
  }
  private:
  
  void publishPosition(){
    geometry_msgs::msg::Point position;
    position = getPosition();
    RCLCPP_INFO(this->get_logger(), "Vehicle Position: x=%.2f, y=%.2f, z=%.2f", position.x, position.y, position.z);
    publisher_->publish(position);
  }

  geometry_msgs::msg::Point getPosition(){
    geometry_msgs::msg::Point pos;
    pos.x = getRandomFloat(-100,100);
    pos.y = getRandomFloat(-100,100);
    pos.z = getRandomFloat(-100,100);
    return pos;
  }

  float getRandomFloat(int min, int max){
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> dist(min, max);
    return dist(gen);
  }
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<positionPublisher>());
  rclcpp::shutdown();
  return 0;
}
