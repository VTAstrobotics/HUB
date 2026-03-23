#include "motor_control/motor_controller_base.hpp"
#include <rclcpp/rclcpp.hpp>
#include "motor_control/kraken_controller.hpp"
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/unmanaged/Unmanaged.hpp>

using namespace ctre::phoenix6;

void KrakenController::control_callback(const motor_messages::msg::Command::SharedPtr msg)
{

  constexpr float EPS = 1e-6f;

  if (abs(msg->dutycycle.data) > EPS)
  {
    this->outDuty.Output = msg->dutycycle.data;

    auto status = motor->SetControl(this->outDuty);
    ctre::phoenix::unmanaged::FeedEnable(100);

    RCLCPP_INFO(this->get_logger(), "SetControl status: %s", status.GetName());
  }
  else if (abs(msg->current.data) > EPS)
  {
    RCLCPP_ERROR(this->get_logger(), "We have not paid for this feature L");
  }
  else if (abs(msg->position.data) > EPS) // this might cause an issue since we cannot send 0
  {

    // TODO: add position control
    this->outPosition = msg->position.data * 1.0_tr;

    this->outPosition.Slot = 0;

    auto status = motor->SetControl(this->outPosition); // Use default timeout for now
    
    ctre::phoenix::unmanaged::FeedEnable(100);

    RCLCPP_INFO(this->get_logger(), "Set Control status: %s", status.GetName());
  }
  else
  {
    controls::DutyCycleOut stop{0};
    this->motor->SetControl(stop);
  }
}

void KrakenController::publish_status()
{

  motor_messages::msg::Feedback feedback;
  // RCLCPP_INFO(this->get_logger(), "Publishing Kraken motor status");

  feedback.current.data = this->motor->GetStatorCurrent().GetValueAsDouble();
  feedback.is_disabled.data = false;
  feedback.position.data = this->motor->GetPosition().GetValueAsDouble();
  feedback.velocity.data = this->motor->GetVelocity().GetValueAsDouble();

  // Implement the logic to publish the Kraken motor status here
  status_publisher->publish(feedback);
}

void KrakenController::publish_health()
{
  motor_messages::msg::Health health;
  // RCLCPP_INFO(this->get_logger(), "Publishing Kraken motor health");
  // Implement the logic to publish the Kraken motor health here

  health.current.data = this->motor->GetStatorCurrent().GetValueAsDouble();
  health.is_failed.data = false;
  health.temperature.data = this->motor->GetDeviceTemp().GetValueAsDouble();
  health.voltage.data = this->motor->GetSupplyVoltage().GetValueAsDouble();

  health_publisher->publish(health);
}