#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "motor_messages/msg/command.hpp"
#include "motor.hpp"

enum Autonomy_State
{
    DIG,
    DUMP,
    HOME
};

enum Control_Signal
{
    TELEOP,
    AUTO
};

using std::placeholders::_1;
class DigMux : public rclcpp::Node
{
public:
    DigMux() : Node("dig_mux")
    {
        autonomy_subscriber = this->create_subscription<motor_messages::msg::Command>(
            "/dig_motor_teleop", 10, std::bind(&DigMux::dig_autonomy_callback, this, _1));

        teleop_subscriber = this->create_subscription<motor_messages::msg::Command>(
            "/dig_motor_teleop", 10, std::bind(&DigMux::dig_teleop_callback, this, _1));

        control_signal = this->create_subscription<std_msgs::msg::Int32>(
            "/dig_control_signal", 10, std::bind(&DigMux::dig_control_signal_callback, this, _1));

        dig_motor = std::make_shared<Motor>("dig_motor", this);

        control_state = TELEOP;
    }

private:
    // need to create custom messages for autonomy.
    rclcpp::Subscription<motor_messages::msg::Command>::SharedPtr autonomy_subscriber;
    rclcpp::Subscription<motor_messages::msg::Command>::SharedPtr teleop_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr control_signal;
    std::shared_ptr<Motor> dig_motor;

    Control_Signal control_state;

    // TODO: define dig action client

    void dig_control_signal_callback(std_msgs::msg::Int32::SharedPtr msg)
    {
        control_state = static_cast<Control_Signal>(msg->data);
        if (msg->data < TELEOP || msg->data > AUTO)
            return;
    }

    void dig_autonomy_callback(motor_messages::msg::Command::SharedPtr msg)
    {
        if (control_state != AUTO)
            return;
    }

    void dig_teleop_callback(motor_messages::msg::Command::SharedPtr msg)
    {
        if (control_state != TELEOP)
            return;

        dig_motor->send_command(*msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DigMux>());
    rclcpp::shutdown();
    return 0;
}