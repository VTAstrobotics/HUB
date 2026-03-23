#pragma once
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "motor_messages/msg/command.hpp"
#include "motor_control/sparkmax_controller.hpp"
#include "motor_control/motor_controller_base.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_broadcaster.h"

class motor
{

public:
    motor(std::string name,
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