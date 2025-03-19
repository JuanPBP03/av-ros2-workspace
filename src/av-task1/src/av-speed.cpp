#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"
#include <random>

using namespace std::chrono_literals;

constexpr auto FREQ = 10;
constexpr auto MSG_PERIOD = 1.0s / FREQ;

// Declaration of speedPublisher class
class speedPublisher : public rclcpp::Node {
  public:
    // Name the node vehicle_speed_publisher using the Node class constructor
    speedPublisher() : Node("vehicle_speed_publisher") {
      // Initialize publisher with topic vehicle_speed
      publisher_ = this->create_publisher<std_msgs::msg::Float32>("vehicle_speed", 10);
      // Initialize timer with period of 1/10Hz and execute publishSpeed when the timer ends
      timer_ = this->create_wall_timer(MSG_PERIOD, std::bind(&speedPublisher::publishSpeed, this));
    }
  private:
  // Declaration of publisher and timer objects
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Function to publish simulated vehicle speed
  void publishSpeed(){
    std_msgs::msg::Float32 speed;
    // Get (simulated) speed from vehicle
    speed.data = getSpeed();
    // Print vehicle speed to the log
    RCLCPP_INFO(this->get_logger(), "Vehicle Speed: %.2f", speed.data);
    // Publish vehicle speed to the topic
    publisher_->publish(speed);
  }
  
  // Function to simulate vehicle speed telemetry
  float getSpeed(){ 
    return getRandomFloat(-100,100);
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
  // Launch speedPublisher node and block program execution until exited
  rclcpp::spin(std::make_shared<speedPublisher>());
  rclcpp::shutdown();
  return 0;
}
