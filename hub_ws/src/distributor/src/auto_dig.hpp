#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "dig/action/motor_control.hpp"

class AutoDig
{
public:
    AutoDig(rclcpp::Node* owner_node);
    void auto_dig(float drive_time_seconds);
    void cancel_dig();
    bool is_running();

private:
    void run_auto_dig(float drive_time_seconds);
    void send_dig_goal(int target_position);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp_action::Client<MotorControl>::SharedPtr dig_client_;
    GoalHandleMotor::SharedPtr active_goal_handle_;
    std::thread driver_thread;
    rclcpp::Node* owner_node; 
    bool running
};