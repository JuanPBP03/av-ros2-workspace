#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point.hpp"
#include <random>

using namespace std::chrono_literals;
// Set constant for broadcast frequency
constexpr auto FREQ = 10;
constexpr auto MSG_PERIOD = 1.0s / FREQ;

// Node class declaration
class positionPublisher : public rclcpp::Node {
  public:
    // Name the node vehicle_position_publisher using the Node class constructor
    positionPublisher() : Node("vehicle_position_publisher") { 
      // Initialize publisher with topic vehicle_position
      publisher_ = this->create_publisher<geometry_msgs::msg::Point>("vehicle_position", 10); 
      // Initialize timer with period of 1/10Hz and call publishPosition when the timer ends 
      timer_ = this->create_wall_timer(MSG_PERIOD, std::bind(&positionPublisher::publishPosition, this));
  }
  private:
  // Function to publish vehicle position to the vehicle_position topic
  void publishPosition(){
    geometry_msgs::msg::Point position;
    // Get (simulated) position from vehicle
    position = getPosition();
    // Print vehicle coordinates in the log
    RCLCPP_INFO(this->get_logger(),
     "Vehicle Position: x=%.2f, y=%.2f, z=%.2f", position.x, position.y, position.z);
    // Publish position to the topic
    publisher_->publish(position);
  }
  // Function to simulate vehicle position telemetry
  geometry_msgs::msg::Point getPosition(){
    geometry_msgs::msg::Point pos;
    pos.x = getRandomFloat(-100,100);
    pos.y = getRandomFloat(-100,100);
    pos.z = getRandomFloat(-100,100);
    return pos;
  }
  // Function to seed random number generator and generate numbers
  float getRandomFloat(int min, int max){
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> dist(min, max);
    return dist(gen);
  }
  // Declaration of publisher and timer
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char ** argv)
{
  // Initialize ROS2 environment
  rclcpp::init(argc,argv);
  // Launch positionPublisher node and block program execution until exited
  rclcpp::spin(std::make_shared<positionPublisher>());
  rclcpp::shutdown();
  return 0;
}
