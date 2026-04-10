#include "rclcpp/rclcpp.hpp"

#include "action_msgs/msg/goal_status_array.hpp"

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

        subscription_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
            "/navigate_to_pose/_action/status", 10,
            std::bind(&AutoFSM::status_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_ptr_;
    AUTO_STATES auto_state;

    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr subscription_;

    action_msgs::msg::GoalStatusArray::SharedPtr nav2_status;

        void
        fsm_callback()
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

    void status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
    {
        // if (msg->status_list.empty()) {
        //     RCLCPP_INFO(this->get_logger(), "No active navigation goals.");
        //     return;
        // }
        nav2_status = msg;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoFSM>());
    rclcpp::shutdown();
    return 0;
}
