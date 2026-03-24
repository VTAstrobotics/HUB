#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int_32.hpp"
#include "std_msgs/msg/float_32.hpp"

enum Autonomy_States
{
    DIG,
    DUMP,
    HOME
};

enum Control_Signal{
    TELEOP,
    AUTO
};

using std::placeholders::_1;
class DigMux : public rclcpp::Node
{
public:
    DigMux() : Node("dig_mux")
    {
        autonomy_subscriber = this->create_subscription<std_msgs::msg::Float32>(
        "/dig_teleop", 10, std::bind(&Drive::dig_teleop_callback, this, _1));

        teleop_subscriber = this->create_subscription<std_msgs::msg::Int32>(
        "/dig_autonomy", 10, std::bind(&Drive::dig_autonomy_callback, this, _1));

        control_signal = this->create_subscription<std_msgs::msg::Int32>(
        "/dig_control_signal", 10, std::bind(&Drive::dig_control_signal, this, _1));
    }

private:
    // need to create custom messages for autonomy.
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr autonomy_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr teleop_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr control_signal;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DigMux>("dig_mux");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}