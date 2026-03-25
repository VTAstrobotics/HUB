#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"

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

class DumpMux : public rclcpp::Node
{
public:
    DumpMux() : Node("dump_mux")
    {
        autonomy_subscriber = this->create_subscription<std_msgs::msg::Int32>(
            "/dump_autonomy_mux", 10, std::bind(&DumpMux::dump_autonomy_callback, this, _1));

        actuator_teleop_subscriber = this->create_subscription<std_msgs::msg::Float32>(
            "/dump_actuator_teleop_mux", 10, std::bind(&DumpMux::dump_actuator_teleop_callback, this, _1));

        door_teleop_subscriber = this->create_subscription<std_msgs::msg::Float32>(
            "/dump_door_teleop_mux", 10, std::bind(&DumpMux::dump_door_teleop_callback, this, _1));

        control_signal = this->create_subscription<std_msgs::msg::Int32>(
            "/dump_control_signal", 10, std::bind(&DumpMux::dump_control_signal_callback, this, _1));

        actuator_publisher = this->create_publisher<std_msgs::msg::Float32>("/dump_actuator_teleop", 10);
        door_publisher     = this->create_publisher<std_msgs::msg::Float32>("/dump_door_teleop", 10);

        control_state = TELEOP;
    }

private:
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr autonomy_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr actuator_teleop_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr door_teleop_subscriber;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr control_signal;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr actuator_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr door_publisher;

    Control_Signal control_state;
    //Next TBD: define dump action client

    void dump_control_signal_callback(std_msgs::msg::Int32::SharedPtr msg)
    {
        control_state = static_cast<Control_Signal>(msg->data);
    }

    void dump_autonomy_callback(std_msgs::msg::Int32::SharedPtr msg)
    {
        if (control_state != AUTO)
            return;
        Autonomy_State auto_state = static_cast<Autonomy_State>(msg->data);
        switch (auto_state)
        {
            // TBD: logic for calling dump server based on auto state
        }
    }

    void dump_actuator_teleop_callback(std_msgs::msg::Float32::SharedPtr msg)
    {
        if (control_state != TELEOP)
            return;
        actuator_publisher->publish(*msg);
    }

    void dump_door_teleop_callback(std_msgs::msg::Float32::SharedPtr msg)
    {
        if (control_state != TELEOP)
            return;
        door_publisher->publish(*msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DumpMux>());
    rclcpp::shutdown();
    return 0;
}