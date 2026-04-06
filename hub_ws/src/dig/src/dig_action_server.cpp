#include <memory>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "dig/action/motor_control.hpp"

#include "motor_messages/msg/command.hpp"
#include "motor_messages/msg/feedback.hpp"

#define DIG_DEPOSIT_POSITION 0
#define DIG_COLLECT_POSITION 0
#define DIG_STOW_POSITION 0

#define DIG_SPEED 0.1

using namespace std::chrono_literals;
using MotorControl = dig::action::MotorControl;
using GoalHandleMotor = rclcpp_action::ServerGoalHandle<MotorControl>;

class dig_action_server : public rclcpp::Node
{
public:

    dig_action_server()
    : Node("dig_action_server")
    {
        action_server_ = rclcpp_action::create_server<MotorControl>(
            this,
            "motor_control",
            std::bind(&dig_action_server::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&dig_action_server::handle_cancel, this, std::placeholders::_1),
            std::bind(&dig_action_server::handle_accepted, this, std::placeholders::_1)
        );
    }

private:
    rclcpp_action::Server<MotorControl>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MotorControl::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request");

        // Check if goal is valid
        (void)uuid;

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Handle cancel request
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMotor> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");

        // Cancel motor movement

        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMotor> goal_handle)
    {
        std::thread{std::bind(&dig_action_server::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMotor> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MotorControl::Feedback>();
        auto result = std::make_shared<MotorControl::Result>();

        double target = goal->target_position;

        // Initialize motors

        rclcpp::Rate loop_rate(10);

        while (rclcpp::ok())
        {
            // Cancel check
            if (goal_handle->is_canceling())
            {
                // Stop motor
                result->success = false;
                result->final_position = 0.0; // Change to sensor position

                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // This is all temp and needs to be added

            double current_position = 0.0;  // Change to CANCoder reading

            // Populate feedback
            feedback->current_position = current_position;

            goal_handle->publish_feedback(feedback);

            if (std::abs(current_position - target) < 0.01)
            {
                // Kill motor
                break;
            }

            loop_rate.sleep();
        }

        // Final result
        result->success = true;
        result->final_position = 0.0; // Add final positon here

        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dig_action_server>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}