#include "motor_control/motor_controller_base.hpp"
#include "motor_control/kraken_controller.hpp"
#include <rclcpp/rclcpp.hpp>


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KrakenController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}