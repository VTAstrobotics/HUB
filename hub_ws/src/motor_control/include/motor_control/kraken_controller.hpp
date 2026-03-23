#pragma once
#include "motor_control/motor_controller_base.hpp"
#include <chrono>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/unmanaged/Unmanaged.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace ctre::phoenix6;
class KrakenController : public motor_control::MotorControllerBase
{
public:
  explicit KrakenController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : motor_control::MotorControllerBase(options)
  {

    RCLCPP_INFO(this->get_logger(), "KrakenController node has been initialized.");
    this->declare_parameter<std::string>("can_interface", "can1");
    this->declare_parameter<int>("can_id", 21);
    this->control_subscription = this->create_subscription<motor_messages::msg::Command>(
        this->get_parameter("control_topic").as_string(),
        10,
        std::bind(&KrakenController::control_callback, this, std::placeholders::_1));

    std::string status_topic = this->get_parameter("status_topic").as_string();
    std::string health_topic = this->get_parameter("health_topic").as_string();
    status_publisher = this->create_publisher<motor_messages::msg::Feedback>(status_topic, 4);
    health_publisher = this->create_publisher<motor_messages::msg::Health>(health_topic, 4);

    std::string can_interface = this->get_parameter("can_interface").as_string();
    this->motor = std::make_unique<hardware::TalonFX>((int)this->get_parameter("can_id").as_int(), can_interface);

    configs::TalonFXConfiguration fx_config{};

    // False is no inversion, positive counterclockwise
    this->declare_parameter<bool>("inverted_value", false);
    this->declare_parameter<float>("kP", 0);
    this->declare_parameter<float>("kI", 0);
    this->declare_parameter<float>("kD", 0);
    this->declare_parameter<float>("kG", 0);
    this->declare_parameter<int>("encoder_canID", 0);

    this->declare_parameter<bool>("arm_cosine", false);
    this->declare_parameter<bool>("brake", false);

    int canID = this->get_parameter("encoder_canID").as_int();
    fx_config.Feedback.FeedbackRemoteSensorID = canID;

    if (canID != 0)
    {
      fx_config.Feedback.FeedbackSensorSource = signals::FeedbackSensorSourceValue::RemoteCANcoder;
    }

    bool kg_type = this->get_parameter("brake").as_bool();
    fx_config.MotorOutput.NeutralMode = (kg_type) ? ctre::phoenix6::signals::NeutralModeValue::Brake : ctre::phoenix6::signals::NeutralModeValue::Coast;

    bool arm_cosine = this->get_parameter("arm_cosine").as_bool();
    fx_config.Slot0.GravityType = (arm_cosine) ? signals::GravityTypeValue::Arm_Cosine : signals::GravityTypeValue::Elevator_Static;
    bool inversion = this->get_parameter("inverted_value").as_bool();
    fx_config.MotorOutput.Inverted = inversion ? signals::InvertedValue::CounterClockwise_Positive : signals::InvertedValue::Clockwise_Positive;

    double kP = this->get_parameter("kP").as_double();
    fx_config.Slot0.kP = kP;
    double kI = this->get_parameter("kI").as_double();
    fx_config.Slot0.kI = kI;
    double kD = this->get_parameter("kD").as_double();
    fx_config.Slot0.kD = kD;
    double kG = this->get_parameter("kG").as_double();
    fx_config.Slot0.kG = kG;
    motor->GetConfigurator().Apply(fx_config);

    // motor->SetNeutralMode(signals::NeutralModeValue::Coast);

    std::chrono::duration<double> status_period(1 / this->get_parameter("status_publish_frequency").as_double()); // Defaults to seconds
    this->status_timer = this->create_wall_timer(status_period, std::bind(&KrakenController::publish_status, this));

    std::chrono::duration<double> health_period(1 / this->get_parameter("health_publish_frequency").as_double()); // Defaults to seconds
    this->health_timer = this->create_wall_timer(health_period, std::bind(&KrakenController::publish_health, this));

    // configs::Slot0Configs slot0Configs{}; // NEED TO TUNE THIS: arbitrary values for now
    // slot0Configs.kP = 2.4;
    // slot0Configs.kI = 0;
    // slot0Configs.kD = 0.1;

    // motor->GetConfigurator().Apply(slot0Configs);
  }

  void control_callback(const motor_messages::msg::Command::SharedPtr msg) override;
  void publish_status() override;
  void publish_health() override;

private:
  std::unique_ptr<hardware::TalonFX> motor; // INIT just so that there are no warnings, this needs to be overridden

  controls::DutyCycleOut outDuty{0.0};
  controls::VelocityVoltage outVelocity{0.0_tps}; // in Turns per second
  controls::PositionVoltage outPosition{0.0_tr};
  // Current Requires a premium payment lmao
};