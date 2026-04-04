#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "motor_messages/msg/command.hpp"
#include "motor.hpp"

enum Control_Signal
{
    TELEOP,
    AUTO
};
//updated dump mux.
using std::placeholders::_1;

class DumpMux : public rclcpp::Node
{
public:
    DumpMux() : Node("dump_mux")
    {
        actuator_autonomy_subscriber = this->create_subscription<motor_messages::msg::Command>(
            "/dump_actuator_motor_auto", 10, std::bind(&DumpMux::dump_actuator_autonomy_callback, this, _1));

        actuator_teleop_subscriber = this->create_subscription<motor_messages::msg::Command>(
            "/dump_actuator_motor_teleop", 10, std::bind(&DumpMux::dump_actuator_teleop_callback, this, _1));

        door_autonomy_subscriber = this->create_subscription<motor_messages::msg::Command>(
            "/dump_door_motor_auto", 10, std::bind(&DumpMux::dump_door_autonomy_callback, this, _1));

        door_teleop_subscriber = this->create_subscription<motor_messages::msg::Command>(
            "/dump_door_motor_teleop", 10, std::bind(&DumpMux::dump_door_teleop_callback, this, _1));

        control_signal = this->create_subscription<std_msgs::msg::Int32>(
            "/dump_control_signal", 10, std::bind(&DumpMux::dump_control_signal_callback, this, _1));

        dump_actuator_motor = std::make_shared<Motor>("dump_actuator_motor", this);
        dump_door_motor     = std::make_shared<Motor>("dump_door_motor", this);

        control_state = TELEOP;
    }

private:
    // Actuator subscribers
    rclcpp::Subscription<motor_messages::msg::Command>::SharedPtr actuator_autonomy_subscriber;
    rclcpp::Subscription<motor_messages::msg::Command>::SharedPtr actuator_teleop_subscriber;

    // Door subscribers
    rclcpp::Subscription<motor_messages::msg::Command>::SharedPtr door_autonomy_subscriber;
    rclcpp::Subscription<motor_messages::msg::Command>::SharedPtr door_teleop_subscriber;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr control_signal;

    std::shared_ptr<Motor> dump_actuator_motor;
    std::shared_ptr<Motor> dump_door_motor;

    Control_Signal control_state;
    // TODO: define dump action client

    void dump_control_signal_callback(std_msgs::msg::Int32::SharedPtr msg)
    {
        if (msg->data < TELEOP || msg->data > AUTO)
            return;
        control_state = static_cast<Control_Signal>(msg->data);
    }

    void dump_actuator_autonomy_callback(motor_messages::msg::Command::SharedPtr msg)
    {
        if (control_state != AUTO)
            return;
        dump_actuator_motor->send_command(*msg);
        RCLCPP_INFO(this->get_logger(), "AUTO COMMAND: DUMP ACTUATOR");
    }

    void dump_actuator_teleop_callback(motor_messages::msg::Command::SharedPtr msg)
    {
        if (control_state != TELEOP)
            return;
        dump_actuator_motor->send_command(*msg);
        RCLCPP_INFO(this->get_logger(), "TELEOP COMMAND: DUMP ACTUATOR");
    }

    void dump_door_autonomy_callback(motor_messages::msg::Command::SharedPtr msg)
    {
        if (control_state != AUTO)
            return;
        dump_door_motor->send_command(*msg);
        RCLCPP_INFO(this->get_logger(), "AUTO COMMAND: DUMP DOOR");
    }

    void dump_door_teleop_callback(motor_messages::msg::Command::SharedPtr msg)
    {
        if (control_state != TELEOP)
            return;
        dump_door_motor->send_command(*msg);
        RCLCPP_INFO(this->get_logger(), "TELEOP COMMAND: DUMP DOOR");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DumpMux>());
    rclcpp::shutdown();
    return 0;
}