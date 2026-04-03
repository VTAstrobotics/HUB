#pragma once
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "motor_messages/msg/command.hpp"
#include "motor_messages/msg/feedback.hpp"
#include "motor.hpp"

class Motor
{

public:
    Motor(std::string name,
          rclcpp::Node* owner_node);
    void send_command(motor_messages::msg::Command command);
    motor_messages::msg::Feedback get_motor_state();
    void feedback_callback(motor_messages::msg::Feedback motor_state);

private:
    std::string motor_name;
    rclcpp::Publisher<motor_messages::msg::Command>::SharedPtr command_publisher;
    rclcpp::Subscription<motor_messages::msg::Feedback>::SharedPtr feedback_subscriber;
    motor_messages::msg::Feedback last_motor_state;
    std::mutex read_lock;
    rclcpp::Node* owner_node; 
};