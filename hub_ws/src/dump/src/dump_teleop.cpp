#include "rclcpp/rclcpp.hpp"
#include "motor.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "motor_messages/msg/command.hpp"
#include "motor_messages/msg/feedback.hpp"

using std::placeholders::_1;

class DumpTeleop : public rclcpp::Node
{
public:
    DumpTeleop() : Node("dump_teleop_node")
    {
        // Initialize motors
        bucket_motor = std::make_shared<Motor>("dump_bucket_teleop", this);
        actuator_motor = std::make_shared<Motor>("dump_linear_actuator", this);

        // Subscriber for bucket deposition motor duty cycle
        bucket_subscriber = this->create_subscription<std_msgs::msg::Float32>(
            "/dump_bucket_teleop", 10,
            std::bind(&DumpTeleop::bucket_callback, this, _1));

        // Subscriber for linear actuator (dump angle) duty cycle
        actuator_subscriber = this->create_subscription<std_msgs::msg::Float32>(
            "/dump_actuator_teleop", 10,
            std::bind(&DumpTeleop::actuator_callback, this, _1));

        homing_subscriber = this->create_subscription<std_msgs::msg::Int32>(
            "/actuator_homing", 10,
            std::bind(&DumpTeleop::actuator_homing_callback, this, _1));
        homed = false;

        RCLCPP_INFO(this->get_logger(), "Dump Teleop Node has started.");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr bucket_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr actuator_subscriber;
    rclcpp::Subscription<motor_messages::msg::Feedback>::SharedPtr feedback_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr homing_subscriber;

    std::shared_ptr<Motor> bucket_motor;
    std::shared_ptr<Motor> actuator_motor;

    bool homed;

    void bucket_callback(std_msgs::msg::Float32::SharedPtr msg)
    {
        float duty = msg->data;
        duty = std::clamp(duty, -1.0f, 1.0f);

        motor_messages::msg::Command bucket_cmd;
        bucket_cmd.dutycycle.data = duty;
        bucket_motor->send_command(bucket_cmd);

        RCLCPP_INFO(this->get_logger(), "Bucket duty cycle: %f", duty);
    }

    void actuator_callback(std_msgs::msg::Float32::SharedPtr msg)
    {
        float duty = msg->data;
        duty = std::clamp(duty, -1.0f, 1.0f);

        motor_messages::msg::Command actuator_cmd;
        actuator_cmd.dutycycle.data = duty;
        actuator_motor->send_command(actuator_cmd);

        RCLCPP_INFO(this->get_logger(), "Actuator duty cycle: %f", duty);
    }

    void actuator_homing_callback(std_msgs::msg::Int32::SharedPtr msg)
    {
        int state = msg->data;
        if (state && !homed)
        {
            motor_messages::msg::Command actuator_cmd;
            actuator_cmd.dutycycle.data = -1;
            actuator_motor->send_command(actuator_cmd);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DumpTeleop>());
    rclcpp::shutdown();
    return 0;
}