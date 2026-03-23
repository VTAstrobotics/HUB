#include "motor_control/motor_controller_base.hpp"
#include <rclcpp/rclcpp.hpp>
#include "motor_control/sparkmax_controller.hpp"
#include "motor_control/SparkBase.hpp"
#include "motor_control/SparkMax.hpp"

void SparkMaxController::control_callback(const motor_messages::msg::Command::SharedPtr msg)
{
  constexpr float EPS = 1e-6f;

  if (std::fabs(msg->dutycycle.data) > EPS) {
    float duty = msg->dutycycle.data;
    RCLCPP_INFO(this->get_logger(), "SparkMax set duty cycle: %f", duty);
    if (motor) {
      motor->SetDutyCycle(duty);
      motor->Heartbeat();
    } else {
      RCLCPP_WARN(this->get_logger(), "No SparkMax motor");
    }
  } else if (std::fabs(msg->velocity.data) > EPS) {
    float vel = msg->velocity.data;
    // RCLCPP_INFO(this->get_logger(), "SparkMax set velocity: %f", vel);
    if (motor) {
      motor->SetVelocity(vel);
      motor->Heartbeat();
    } else {
      RCLCPP_WARN(this->get_logger(), "No SparkMax motor");
    }
  } else if (std::fabs(msg->position.data) > EPS) {
    float pos = msg->position.data;
    RCLCPP_INFO(this->get_logger(), "SparkMax set position: %f", pos);
    if (motor) {
      motor->SetPosition(pos);
      motor->Heartbeat();
    } else {
      RCLCPP_WARN(this->get_logger(), "No SparkMax motor");
    }
  } else if (std::fabs(msg->current.data) > EPS) {
    float cur = msg->current.data;
    RCLCPP_INFO(this->get_logger(), "SparkMax set current: %f", cur);
    if (motor) {
      motor->SetCurrent(cur);
      motor->Heartbeat();
    } else {
      RCLCPP_WARN(this->get_logger(), "No SparkMax motor");
    }
  } else {
    RCLCPP_DEBUG(this->get_logger(), "SparkMax no control field set");
    if (motor) {
      motor->SetDutyCycle(0.0f);
    }
  }
}

void SparkMaxController::publish_status()
{
  motor_messages::msg::Feedback feedback;
  if (!motor) {
    RCLCPP_WARN(this->get_logger(), "publish_status: no motor instance");
    feedback.current.data = 0.0f;
    feedback.position.data = 0.0f;
    feedback.velocity.data = 0.0f;
    feedback.is_disabled.data = true;
    status_publisher->publish(feedback);
    return;
  }

  feedback.velocity.data = motor->GetVelocity();
  feedback.current.data = motor->GetCurrent();
  feedback.position.data = motor->GetPosition()/4096.0;
  feedback.is_disabled.data = false;
  status_publisher->publish(feedback);
}

void SparkMaxController::publish_health()
{
  motor_messages::msg::Health health;
  if (!motor) {
    RCLCPP_WARN(this->get_logger(), "publish_health: no motor instance");
    health.current.data = 0.0f;
    health.temperature.data = 0.0f;
    health.voltage.data = 0.0f;
    health.is_failed.data = true;
    health_publisher->publish(health);
    return;
  }

  health.current.data = motor->GetCurrent();
  health.temperature.data = motor->GetTemperature();
  health.voltage.data = motor->GetVoltage();
  health.is_failed.data = false;

  health_publisher->publish(health);
}

