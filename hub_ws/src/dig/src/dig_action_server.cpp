#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "dig/action/motor_control.hpp"

#include "motor_messages/msg/command.hpp"
#include "motor_messages/msg/feedback.hpp"

#include <thread>
#include <cmath>

// Placeholder, not done yet
#define DIG_DEPOSIT_POSITION 0
#define DIG_COLLECT_POSITION 0
#define DIG_STOW_POSITION 0
#define DIG_THRESHOLD 1

#define DUTY_CYCLE 0.1

using namespace std::placeholders;
using MotorControl = dig::action::MotorControl;
using GoalHandleMotor = rclcpp_action::ServerGoalHandle<MotorControl>;

enum DigPosition
{
    COLLECT,
    DEPOSIT,
    STOW
};

class dig_action_server : public rclcpp::Node
{
public:

    dig_action_server (const rclcpp::NodeOptions & options)
        : Node("dig_action_server")
    {
        dig_action_server = rclcpp_action::create_server<MotorControl>(
            this,
            "motor_control",
            std::bind(&dig_action_server::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&dig_action_server::handle_cancel, this, std::placeholders::_1),
            std::bind(&dig_action_server::handle_accepted, this, std::placeholders::_1)

            door_feedback_subscriber = this->create_subscription<motor_messages::msg::Feedback>("/dig/status", 10, std::bind(&dig_action_server::dig_status_callback, this, _1));

            dig_publisher = this->create_publisher<motor_messages::msg::Command>("/dig/control", 4);
            
        );
    }

private:

    rclcpp_action::Server<MotorControl>::SharedPtr dig_action_server;

    rclcpp::Subscription<motor_messages::msg::Feedback>::SharedPtr dig_feedback_subscriber;
    rclcpp::Publisher<motor_messages::msg::Command>::SharedPtr dig_publisher;

    float dig_position = 0;
    float target_position = 0;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MotorControl::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with goal %d", goal->target_position);

        (void)uuid;

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Handle cancel request
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMotor> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");

        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMotor> goal_handle)
    {
        using namespace std::placeholders;
        std::thread{std::bind(&dig_action_server::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMotor> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        DigPosition commandedPosition = static_cast<DigPosition>(goal_handle->get_goal()->target_position);
        auto feedback = std::make_shared<MotorControl::Feedback>();
        auto result = std::make_shared<MotorControl::Result>();

        auto & positions = feedback->positions;
        positions.push_back(dig_position);
        rclcpp::Rate loop_rate(10);

        while (true)
        {
            // Cancel check
            if (goal_handle->is_canceling())
            {
                result->final_positions = positions;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            positions.clear();
            positions.push_back(dig_position);
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish feedback");

            motor_messages::msg::Command dig_msg;
            bool dig_complete = false;

            switch (commandedPosition)
            {
                case STOW:

                    dig_msg.position.data = DIG_STOW_POSITION;
                    //dig_complete = true;
                    break;
                case COLLECT:
                    dig_msg.position.data = DIG_COLLECT_POSITION;
                    //dig_complete = true;
                    break;
                case DEPOSIT:
                    dig_msg.position.data = DIG_DEPOSIT_POSITION;
                    //dig_complete = true;
                    break;
                
            }

            dig_publisher->publish(dig_msg);
            //if (dig_complete) break;
            loop_rate.sleep();
            
        }

        // Final result
        if (rclcpp::ok()) 
        {
            result->final_positions = positions;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }

    void dig_status_callback(motor_messages::msg::Feedback msg)
    {
        dig_position = msg.position.data;
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(dig_action_server)