#include <memory>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "dig/action/motor_control.hpp"

using namespace std::chrono_literals;

class dig_action_server : public rclcpp::Node
{
public:
    using MotorControl = dig::action::MotorControl;
    using GoalHandleMotor = rclcpp_action::ServerGoalHandle<MotorControl>;

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

    // Handle incoming goal
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MotorControl::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request");

        // TODO: Validate goal if needed
        (void)uuid;

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Handle cancel request
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMotor> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");

        // TODO: Stop motor safely here

        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // When goal is accepted
    void handle_accepted(const std::shared_ptr<GoalHandleMotor> goal_handle)
    {
        std::thread{std::bind(&dig_action_server::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    // Execution logic
    void execute(const std::shared_ptr<GoalHandleMotor> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MotorControl::Feedback>();
        auto result = std::make_shared<MotorControl::Result>();

        double target = goal->target_position;
        double max_speed = goal->max_speed;

        // TODO: Initialize motor control here
        // Example: set motor speed limit, enable driver, etc.

        rclcpp::Rate loop_rate(10);

        while (rclcpp::ok())
        {
            // Check if canceled
            if (goal_handle->is_canceling())
            {
                // TODO: Stop motor immediately
                result->success = false;
                result->final_position = 0.0; // TODO: read actual position

                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // ============================
            // TODO: MOTOR CONTROL LOGIC
            // ============================
            // - Read current motor position from sensor
            // - Compute control (PID, etc.)
            // - Send command to motor driver
            // ============================

            double current_position = 0.0;  // TODO: replace with sensor reading
            double current_speed = 0.0;     // TODO: replace with sensor reading

            // Populate feedback
            feedback->current_position = current_position;
            feedback->current_speed = current_speed;

            goal_handle->publish_feedback(feedback);

            // Exit condition (target reached)
            if (std::abs(current_position - target) < 0.01)
            {
                // TODO: Stop motor
                break;
            }

            loop_rate.sleep();
        }

        // Final result
        result->success = true;
        result->final_position = 0.0; // TODO: read final position

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