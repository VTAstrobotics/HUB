#include <memory>
#include <chrono>
#include "settings.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "map.h"
#include "slew_rate_limiter.hpp"
#include "auto_dig.hpp"
#include "auto_dump.hpp"

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
    auto_dig_ptr = std::make_shared<AutoDig>(this);   // threaded auto dig
    auto_dump_ptr = std::make_shared<AutoDump>(this); // threaded auto dump

    joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>( // Creating the subscriber to the Joy topic
        "/joy", 10, std::bind(&Distributor::joy_callback, this, _1));

    velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10); // creates the publisher to the /joy topic

    dig_publisher = this->create_publisher<std_msgs::msg::Float32>("/dig_teleop", 10);
    dump_actuator_publisher = this->create_publisher<std_msgs::msg::Float32>("/dump_actuator_teleop", 10);
    dump_bucket_publisher = this->create_publisher<std_msgs::msg::Float32>("/dump_bucket_teleop", 10);

    actuator_homing_publisher = this->create_publisher<std_msgs::msg::Int32>("/actuator_homing", 10);

    // uses the joy_callback to recieve the message from the subscriber and publish it to the /joy topic
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&Distributor::timer_callback, this));
    stopwatch.start();
    this->declare_parameter("TRANSLATION_CONTROL", "LSTICKY");
    this->declare_parameter("ROTATION_CONTROL", "RSTICKX");
    this->declare_parameter("DIG_UP", "RTRIGGER");
    this->declare_parameter("DIG_DOWN", "LTRIGGER");
    this->declare_parameter("RAISE_ACTUATOR", "BUTTON_Y");
    this->declare_parameter("LOWER_ACTUATOR", "BUTTON_X");
    this->declare_parameter("OPEN_DOOR", "BUTTON_A");
    this->declare_parameter("CLOSE_DOOR", "BUTTON_B");
    this->declare_parameter("LINEAR_SCALE", 0.3);
    this->declare_parameter("ANGULAR_SCALE", 0.6);
    this->declare_parameter("ACTUATOR_HOMING", "BUTTON_LBUMPER");

    this->declare_parameter("DIG_AUTO", "BUTTON_RSTICK");
    this->declare_parameter("DIG_AUTO_CANCEL", "BUTTON_LSTICK");

    this->declare_parameter("DUMP_DEPOSIT", "BUTTON_RBUMPER");
    this->declare_parameter("DUMP_HOME", "BUTTON_START");
    this->declare_parameter("DUMP_CANCEL", "BUTTON_BACK");

    TRANSLATION_CONTROL = this->get_parameter("TRANSLATION_CONTROL").as_string();
    ROTATION_CONTROL = this->get_parameter("ROTATION_CONTROL").as_string();
    OPEN_DOOR = this->get_parameter("OPEN_DOOR").as_string();
    CLOSE_DOOR = this->get_parameter("CLOSE_DOOR").as_string();
    DIG_UP = this->get_parameter("DIG_UP").as_string();
    DIG_DOWN = this->get_parameter("DIG_DOWN").as_string();
    RAISE_ACTUATOR = this->get_parameter("RAISE_ACTUATOR").as_string();
    LOWER_ACTUATOR = this->get_parameter("LOWER_ACTUATOR").as_string();

    linear_scale = this->get_parameter("LINEAR_SCALE").as_double();
    angular_scale = this->get_parameter("ANGULAR_SCALE").as_double();

    ACTUATOR_HOMING = this->get_parameter("ACTUATOR_HOMING").as_string();

    DIG_AUTO = this->get_parameter("DIG_AUTO").as_string();
    DIG_AUTO_CANCEL = this->get_parameter("DIG_AUTO_CANCEL").as_string();

    // dump buttons declared
    DUMP_DEPOSIT = this->get_parameter("DUMP_DEPOSIT").as_string();
    DUMP_HOME = this->get_parameter("DUMP_HOME").as_string();
    DUMP_CANCEL = this->get_parameter("DUMP_CANCEL").as_string();

    RCLCPP_INFO(this->get_logger(), "DISTRIBUTOR ONLINE");
  }

  void make_slew_rate_limiters(){
    linear_slew_rate_limiter = new slew_rate_limiter{1, this->shared_from_this()};
    dig_slew_rate_limiter = new slew_rate_limiter{0.3, this->shared_from_this()};
  }

private:


  void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg)
  {
    
    double lin = linear_slew_rate_limiter->calculate(msg->axes[controls.at(TRANSLATION_CONTROL)]) * linear_scale;
    double ang = msg->axes[controls.at(ROTATION_CONTROL)] * angular_scale;

    geometry_msgs::msg::Twist cmd; // create a variable of type Twist to hold the velocity
    cmd.linear.x = lin;            // assigning the linear x vlaue to lin
    cmd.angular.z = ang;           // assigning the angular z value to ang

    if (!auto_dig_ptr->is_running())
    {
      velocity_publisher->publish(cmd); // publishing the cmd variable to the /cmd_vel topic
    }

    float dig_up = (-1 * msg->axes[controls.at(DIG_UP)] + 1) * 0.15;
    float dig_down = (-1 * msg->axes[controls.at(DIG_DOWN)] + 1) * 0.15;

    double dig_duty = dig_slew_rate_limiter->calculate((dig_up - dig_down) * 0.5); // limit duty cycle
    std_msgs::msg::Float32 duty_msg;
    duty_msg.data = dig_duty;
    if (!auto_dig_ptr->is_running())
    {
      dig_publisher->publish(duty_msg);
    }

    double dump_actuator_duty = (msg->buttons[controls.at(RAISE_ACTUATOR)] - msg->buttons[controls.at(LOWER_ACTUATOR)]);
    if (!auto_dump_ptr->is_running())
    {
      duty_msg.data = dump_actuator_duty;
      dump_actuator_publisher->publish(duty_msg);
    }

    double dump_door_duty = (msg->buttons[controls.at(OPEN_DOOR)] - msg->buttons[controls.at(CLOSE_DOOR)]) * 0.07; // limit duty cyle;
    duty_msg.data = dump_door_duty;
    dump_bucket_publisher->publish(duty_msg);

    if (msg->buttons[controls.at(DIG_AUTO)])
    {
      if (!auto_dig_ptr->is_running())
        auto_dig_ptr->auto_dig(2.5);
    }

    if (msg->buttons[controls.at(DIG_AUTO_CANCEL)])
    {
      auto_dig_ptr->cancel_dig();
    }

    // For DUMP

    if (msg->buttons[controls.at(DUMP_DEPOSIT)])
    {
      if (!auto_dump_ptr->is_running())
        auto_dump_ptr->auto_dump(1);
    }
    if (msg->buttons[controls.at(DUMP_HOME)])
    {
      if (!auto_dump_ptr->is_running())
        auto_dump_ptr->auto_dump(0);
    }
    if (msg->buttons[controls.at(DUMP_CANCEL)])
    {
      auto_dump_ptr->cancel_dump();
    }

    // std_msgs::msg::Int32 homing_msg;
    // int actuator_homing = msg->buttons[controls.at(ACTUATOR_HOMING)];
    // homing_msg.data = actuator_homing;
    // actuator_homing_publisher->publish(homing_msg);

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
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dig_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dump_bucket_publisher;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dump_actuator_publisher;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr actuator_homing_publisher;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string TRANSLATION_CONTROL;
  std::string ROTATION_CONTROL;
  std::string CONVEYOR_FORWARD;
  std::string CONVEYOR_REVERSE;
  std::string OPEN_DOOR;
  std::string CLOSE_DOOR;
  std::string RAISE_ACTUATOR;
  std::string LOWER_ACTUATOR;
  std::string DIG_UP;
  std::string DIG_DOWN;
  std::string ACTUATOR_HOMING;
  slew_rate_limiter* linear_slew_rate_limiter;
  slew_rate_limiter* dig_slew_rate_limiter;
  
  std::string DIG_AUTO;
  std::string DIG_AUTO_CANCEL;
  std::string DUMP_DEPOSIT;
  std::string DUMP_HOME;
  std::string DUMP_CANCEL;

  std::shared_ptr<AutoDig> auto_dig_ptr;
  std::shared_ptr<AutoDump> auto_dump_ptr;

  // rclcpp::Timer timer_
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto distributor = std::make_shared<Distributor>();
  distributor->make_slew_rate_limiters();
  rclcpp::spin(distributor);
  rclcpp::shutdown();
  return 0;
}
