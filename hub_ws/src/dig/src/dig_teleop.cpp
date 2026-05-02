#include "rclcpp/rclcpp.hpp"
#include "motor.hpp"

#include "std_msgs/msg/float32.hpp"
#include "motor_messages/msg/command.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
class DigTeleop : public rclcpp::Node
{
public:
    DigTeleop() : Node("dig_teleop_node")
    {
        linkage_motor = std::make_shared<Motor>("dig_motor", this);
        mux_subscriber = this->create_subscription<std_msgs::msg::Float32>(
            "/dig_teleop", 10, std::bind(&DigTeleop::mux_callback, this, _1));
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr mux_subscriber;
    std::shared_ptr<Motor> linkage_motor;

    void mux_callback(std_msgs::msg::Float32::SharedPtr msg)
    {
        float duty = msg->data;
        duty = std::clamp(duty, -1.0f, 1.0f);
        if (fabs(duty) < 0.01)
        {
            return;
        }
        motor_messages::msg::Command dig_duty_msg;
        dig_duty_msg.dutycycle.data = duty;
        linkage_motor->send_command(dig_duty_msg);
        RCLCPP_INFO(this->get_logger(), "Dig duty cycle: %f", duty);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DigTeleop>());
    rclcpp::shutdown();
    return 0;
}
