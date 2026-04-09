#include "rclcpp/rclcpp.hpp"

enum AUTO_STATES
{
    STARTUP,
    HOME,
    DRIVE_TO_DIG,
    DIG,
    DRIVE_TO_DUMP,
    DUMP
};

class AutoFSM : public rclcpp::Node // You will modify the name
{
public:
    AutoFSM() : Node("auto_fsm_node") // You will modify the name
    {

        timer_ptr_ = this->create_wall_timer(0.02s, std::bind(&AutoFSM::fsm_callback, this),
                                             timer_cb_group_);
        auto_state = STARTUP
    }

private:
    rclcpp::TimerBase::SharedPtr timer_ptr_;
    AUTO_STATES auto_state;

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
            default() : break;
        }
        // fsm
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoFSM>());
    rclcpp::shutdown();
    return 0;
}
