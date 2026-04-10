#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"

enum AUTO_STATES
{
    STARTUP,
    HOME,
    DRIVE_TO_DIG,
    DIG,
    DRIVE_TO_DUMP,
    DUMP
};

using std::placeholders::_1;
class AutoFSM : public rclcpp::Node // You will modify the name
{
public:
    AutoFSM() : Node("auto_fsm_node") // You will modify the name
    {

        timer_ptr_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&AutoFSM::fsm_callback, this));
        auto_state = STARTUP;

        position_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered_map", 10, std::bind(&AutoFSM::position_subscriber_callback, this, _1));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_ptr_;
    AUTO_STATES auto_state;

    nav_msgs::msg::Odometry current_position;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr position_subscriber;

    void fsm_callback()
    {
        switch (auto_state)
        {
        case (STARTUP):
            // button press
            break;
        case (HOME):
            // send home goal
            break;
        case (DRIVE_TO_DIG):
            break;
        case (DIG):
            break;
        case (DRIVE_TO_DUMP):
            break;
        case (DUMP):
            break;
        default:
            break;
        }
        // fsm
    }

    void position_subscriber_callback(nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_position = *msg;
        return;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoFSM>());
    rclcpp::shutdown();
    return 0;
}
