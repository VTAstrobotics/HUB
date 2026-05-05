#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <mutex>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "dig/action/motor_control.hpp"
#include "std_msgs/msg/float32.hpp"

class AutoDig
{
public:
    AutoDig(rclcpp::Node* owner_node);
    ~AutoDig();
    void auto_dig(float drive_time_seconds);
    void cancel_dig();
    bool is_running();

private:
    void run_auto_dig(float drive_time_seconds);
    bool send_dig_goal(int target_position);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp_action::Client<dig::action::MotorControl>::SharedPtr dig_client_;
    rclcpp_action::ClientGoalHandle<dig::action::MotorControl>::SharedPtr active_goal_handle_;
    std::thread driver_thread;
    rclcpp::Node* owner_node; 
    std::atomic_bool running;
    std::atomic_bool cancel_requested;
    std::atomic_bool dig_goal_succeeded;
    std::mutex goal_mutex;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dig_publisher_auto;
};