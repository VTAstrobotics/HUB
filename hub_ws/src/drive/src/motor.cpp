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
#include "motor.hpp"



using std::placeholders::_1;


motor::motor(std::string name, rclcpp::Node* owner_node)
{
    this->motor_name = name;
    this->owner_node = owner_node;
    this->command_publisher = owner_node->create_publisher<motor_messages::msg::Command>(name + "/control", 4);
    this->feedback_subscriber = owner_node->create_subscription<motor_messages::msg::Feedback>(name+"/status", 4, std::bind(
        &motor::feedback_callback, this, _1
    ));
}

void motor::send_command(motor_messages::msg::Command command)
{
    this->command_publisher->publish(command);   
}



motor_messages::msg::Feedback motor::get_motor_state()
{
    this->read_lock.lock();
    return this->last_motor_state;
    this->read_lock.unlock();
}

void motor::feedback_callback(motor_messages::msg::Feedback motor_state)
{
    this->read_lock.lock();
    
    this->last_motor_state = motor_state;

    this->read_lock.unlock();

}
