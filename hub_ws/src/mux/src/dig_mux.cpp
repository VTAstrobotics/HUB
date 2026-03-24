#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"

enum Autonomy_States
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
        autonomy_subscriber = this->create_subscription<std_msgs::msg::Int32>(
            "/dig_autonomy_mux", 10, std::bind(&DigMux::dig_autonomy_callback, this, _1));

        teleop_subscriber = this->create_subscription<std_msgs::msg::Float32>(
            "/dig_teleop_mux", 10, std::bind(&DigMux::dig_teleop_callback, this, _1));

        control_signal = this->create_subscription<std_msgs::msg::Int32>(
            "/dig_control_signal", 10, std::bind(&DigMux::dig_control_signal_callback, this, _1));

        teleop_publisher = this->create_publisher<std_msgs::msg::Float32>("/dig_teleop", 10);
        control_state = TELEOP;
    }

private:
    // need to create custom messages for autonomy.
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr autonomy_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr teleop_subscriber;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr teleop_publisher;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr control_signal;
    Control_Signal control_state;
    // TODO: define dig action client

    void dig_control_signal_callback(std_msgs::msg::Int32::SharedPtr msg)
    {
        control_state = static_cast<Control_Signal>(msg->data);
    }
    void dig_autonomy_callback(std_msgs::msg::Int32::SharedPtr msg)
    {
        if (control_state != AUTO)
            return;
        // TODO: logic for calling dig server
    }
    void dig_teleop_callback(std_msgs::msg::Float32::SharedPtr msg)
    {
        if (control_state != TELEOP)
            return;
        teleop_publisher->publish(*msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DigMux>());
    rclcpp::shutdown();
    return 0;
}