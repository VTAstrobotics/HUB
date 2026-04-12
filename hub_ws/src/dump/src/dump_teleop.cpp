#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "motor_messages/msg/command.hpp"

using std::placeholders::_1;

class DumpTeleop : public rclcpp::Node
{
public:
    DumpTeleop() : Node("dump_teleop_node")
    {

        // Subscriber for bucket deposition motor duty cycle
        door_subscriber = this->create_subscription<std_msgs::msg::Float32>(
           "/dump_door_teleop", 10,
           std::bind(&DumpTeleop::door_callback, this, _1));

        // Subscriber for linear actuator (dump angle) duty cycle
        actuator_subscriber = this->create_subscription<std_msgs::msg::Float32>(
            "/dump_actuator_teleop", 10,
            std::bind(&DumpTeleop::actuator_callback, this, _1));

        door_duty_publisher = this->create_publisher<motor_messages::msg::Command>("/dump_door/control", 4);
        actuator_duty_publisher = this->create_publisher<motor_messages::msg::Command>("/dump_linear_actuator/control", 4);
        
        RCLCPP_INFO(this->get_logger(), "Dump Teleop Node has started.");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr door_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr actuator_subscriber;
    rclcpp::Publisher<motor_messages::msg::Command>::SharedPtr door_duty_publisher;
    rclcpp::Publisher<motor_messages::msg::Command>::SharedPtr actuator_duty_publisher;

    void door_callback(std_msgs::msg::Float32::SharedPtr msg)
    {
        float duty = msg->data;
        duty = std::clamp(duty, -1.0f, 1.0f);

        motor_messages::msg::Command door_cmd;
        door_cmd.dutycycle.data = duty;
        door_duty_publisher->publish(door_cmd);

        // RCLCPP_INFO(this->get_logger(), "Door duty cycle: %f", duty);
    }

    void actuator_callback(std_msgs::msg::Float32::SharedPtr msg)
    {
        float duty = msg->data;
        duty = std::clamp(duty, -1.0f, 1.0f);

        motor_messages::msg::Command actuator_cmd;
        actuator_cmd.dutycycle.data = duty;
        actuator_duty_publisher->publish(actuator_cmd);

        // RCLCPP_INFO(this->get_logger(), "Actuator duty cycle: %f", duty);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DumpTeleop>());
    rclcpp::shutdown();
    return 0;
}
