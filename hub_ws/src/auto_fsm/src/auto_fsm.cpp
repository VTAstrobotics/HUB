#include "rclcpp/rclcpp.hpp"
#include ""

class AutoFSM : public rclcpp::Node // You will modify the name
{
public:
    AutoFSM() : Node("auto_fsm_node") // You will modify the name
    {

        timer_ptr_ = this->create_wall_timer(0.02s, std::bind(&AutoFSM::fsm_callback, this),
                                             timer_cb_group_);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_ptr_;

    void fsm_callback()
    {
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
