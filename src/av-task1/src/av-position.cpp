#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point>
#include <random>

using namespace std::chrono_literals;

constexpr auto FREQ = 10;
constexpr auto MSG_PERIOD = 1s / FREQ;

class positionPublisher : public rclcpp::Node {
  public:
  positionPublisher() : Node("vehicle_position_publisher") {
    publisher_ = this->create_publisher<geometry_msgs::msg::Point>("vehicle_position", 10);
    timer_ = this->create_wall_timer(MSG_PERIOD, std::bind(&positionPublisher::publish_position(), this));
  }
  private:
  
  void  publishPosition(){
    geometry_msgs::msg::Point position;
    position = getPosition();
  }

  geometry_msgs::msg::Point getPosition(){
    float x = getRandomFloat(-100,100);
    float y = getRandomFloat(-100,100);
    float z = getRandomFloat(-100,100);
    return {x, y, z}
  }
  float getRandomFloat(int min, int max){
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> dist(min, max);
    return dist(gen);
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(make_shared<positionPublisher>());
  rclcpp::shutdown();
  return 0;
}
