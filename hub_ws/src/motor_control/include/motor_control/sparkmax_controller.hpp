#pragma once
#include "motor_control/motor_controller_base.hpp"
#include "motor_control/SparkMax.hpp"
#include "motor_control/SparkBase.hpp"

#include <rclcpp/rclcpp.hpp>

class SparkMaxController : public motor_control::MotorControllerBase
{
public:
  explicit SparkMaxController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : motor_control::MotorControllerBase(options)
  {

    RCLCPP_INFO(this->get_logger(), "SparkMaxController node has been initialized.");

    this->declare_parameter<std::string>("can_interface", "can0");
    this->declare_parameter<int>("can_id", 22);

    control_subscription_ = this->create_subscription<motor_messages::msg::Command>(
        this->get_parameter("control_topic").as_string(),
        10,
        std::bind(&SparkMaxController::control_callback, this, std::placeholders::_1));

    std::string status_topic = this->get_parameter("status_topic").as_string();
    std::string health_topic = this->get_parameter("health_topic").as_string();
    status_publisher = this->create_publisher<motor_messages::msg::Feedback>(status_topic, 4);
    health_publisher = this->create_publisher<motor_messages::msg::Health>(health_topic, 4);

    std::string can_interface = this->get_parameter("can_interface").as_string();
    int can_id = this->get_parameter("can_id").as_int();

    motor = std::make_unique<SparkMax>(can_interface, static_cast<uint8_t>(can_id));

    motor->SetP(0, 0.00008);
    motor->SetI(0, 0.0);
    motor->SetD(0, 0.0001);
    motor->SetF(0, 0.00017);
    motor->BurnFlash();

    float velocityConversion = motor->GetVelocityConversionFactor();
    float positionConversion = motor->GetPositionConversionFactor();

    RCLCPP_INFO(this->get_logger(), "Velocity Conversion: %.6f", velocityConversion);

    RCLCPP_INFO(this->get_logger(), "Position Conversion: %.6f", positionConversion);

  
    

    std::chrono::duration<double> status_period(1 / this->get_parameter("status_publish_frequency").as_double());
    this->status_timer = this->create_wall_timer(status_period, std::bind(&SparkMaxController::publish_status, this));

    std::chrono::duration<double> health_period(1 / this->get_parameter("health_publish_frequency").as_double());
    this->health_timer = this->create_wall_timer(health_period, std::bind(&SparkMaxController::publish_health, this));
  }

  void control_callback(const motor_messages::msg::Command::SharedPtr msg) override;
  void publish_status() override;
  void publish_health() override;

private:
  rclcpp::Subscription<motor_messages::msg::Command>::SharedPtr control_subscription_;
  std::unique_ptr<SparkMax> motor;
};