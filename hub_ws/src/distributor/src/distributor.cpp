#include <memory>
#include <chrono>
#include "settings.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "map.h"

#define TIMEOUT 10.0

class Stopwatch
{
public:
  void start()
  {
    start_time = std::chrono::high_resolution_clock::now();
  }
  double elapsedMilliseconds()
  {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
  }
  void reset()
  {
    start_time = std::chrono::high_resolution_clock::now();
  }

private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
};

using std::placeholders::_1;
class Distributor : public rclcpp::Node
{
public:
  Distributor()
      : Node("Distributor_node") // name of the node
  {
    joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>( // Creating the subscriber to the Joy topic
        "/joy", 10, std::bind(&Distributor::joy_callback, this, _1));

    velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10); // creates the publisher to the /joy topic
    // uses the joy_callback to recieve the message from the subscriber and publish it to the /joy topic
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&Distributor::timer_callback, this));
    stopwatch.start();
    this->declare_parameter("TRANSLATION_CONTROL", "LSTICKY");
    this->declare_parameter("ROTATION_CONTROL", "RSTICKX");
    this->declare_parameter("CONVEYOR_FORWARD", "BUTTON_B");
    this->declare_parameter("CONVEYOR_REVERSE", "BUTTON_A");
    this->declare_parameter("LINEAR_SCALE", 0.3);
    this->declare_parameter("ANGULAR_SCALE", 0.3);
    TRANSLATION_CONTROL = this->get_parameter("TRANSLATION_CONTROL").as_string();
    ROTATION_CONTROL = this->get_parameter("ROTATION_CONTROL").as_string();
    CONVEYOR_FORWARD = this->get_parameter("CONVEYOR_FORWARD").as_string();
    CONVEYOR_REVERSE = this->get_parameter("CONVEYOR_REVERSE").as_string();
    linear_scale = this->get_parameter("LINEAR_SCALE").as_double();
    angular_scale = this->get_parameter("ANGULAR_SCALE").as_double();

    RCLCPP_DEBUG(this->get_logger(), "Translation Control parameter: %s", TRANSLATION_CONTROL.c_str());
    RCLCPP_DEBUG(this->get_logger(), "Rotation Control parameter: %s", ROTATION_CONTROL.c_str());
    RCLCPP_DEBUG(this->get_logger(), "Conveyor Forward parameter: %s", CONVEYOR_FORWARD.c_str());
    RCLCPP_DEBUG(this->get_logger(), "Conveyor Reverse parameter: %s", CONVEYOR_REVERSE.c_str());

    RCLCPP_INFO(this->get_logger(), "DISTRIBUTOR ONLINE");
  }

private:
  void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg)
  {
    double lin = msg->axes[controls.at(TRANSLATION_CONTROL)] * linear_scale;
    double ang = msg->axes[controls.at(ROTATION_CONTROL)] * angular_scale;

    geometry_msgs::msg::Twist cmd;    // create a variable of type Twist to hold the velocity
    cmd.linear.x = lin;               // assigning the linear x vlaue to lin
    cmd.angular.z = ang;              // assigning the angular z value to ang
    velocity_publisher->publish(cmd); // publishing the cmd variable to the /cmd_vel topic

    stopwatch.reset();
  }
  void timer_callback()
  {
    if (stopwatch.elapsedMilliseconds() > TIMEOUT * 1000)
    {
      geometry_msgs::msg::Twist cmd;    // create a variable of type Twist to hold the velocity
      cmd.linear.x = 0;                 // assigning the linear x vlaue to lin
      cmd.angular.z = 0;                // assigning the angular z value to ang
      velocity_publisher->publish(cmd); // publishing the cmd variable to the /cmd_vel topic
      RCLCPP_ERROR(this->get_logger(), "CONNECTION LOST %d", 4);
    }
  }

  double linear_scale = 0.6;
  double angular_scale = 3.0;
  Stopwatch stopwatch;

  // this is where you can declare subscribers/publishers.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string TRANSLATION_CONTROL;
  std::string ROTATION_CONTROL;
  std::string CONVEYOR_FORWARD;
  std::string CONVEYOR_REVERSE;
  // rclcpp::Timer timer_
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Distributor>());
  rclcpp::shutdown();
  return 0;
}