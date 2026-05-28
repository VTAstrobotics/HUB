#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "dig/action/motor_control.hpp"
#include "motor_messages/msg/command.hpp"
#include "motor_messages/msg/feedback.hpp"

#include <thread>
#include <memory>
#include <vector>
#include <cmath>

using std::placeholders::_1;
using std::placeholders::_2;

using MotorControl = dig::action::MotorControl;
using GoalHandleMotor = rclcpp_action::ServerGoalHandle<MotorControl>;

#define DIG_DEPOSIT_POSITION 0.374f
#define DIG_COLLECT_POSITION -0.15f
#define DIG_STOW_POSITION 0.1f
#define DIG_THRESHOLD 0.05f

/* TODO
Add can, brake mode, and PID constants to launch file for dig motor, also inversion
*/

enum DigPosition
{
    COLLECT = 0,
    DEPOSIT = 1,
    STOW = 2
};

std::unordered_map<DigPosition, float> positionMap{{COLLECT, DIG_COLLECT_POSITION}, {DEPOSIT, DIG_DEPOSIT_POSITION}, {STOW, DIG_STOW_POSITION}};

class DigActionServer : public rclcpp::Node
{
public:
    explicit DigActionServer(const rclcpp::NodeOptions &options)
        : Node("dig_action_server", options), dig_position_(0.0f)
    {
        action_server_ = rclcpp_action::create_server<MotorControl>(
            this,
            "motor_control",
            std::bind(&DigActionServer::handle_goal, this, _1, _2),
            std::bind(&DigActionServer::handle_cancel, this, _1),
            std::bind(&DigActionServer::handle_accepted, this, _1));

        dig_feedback_subscriber_ =
            this->create_subscription<motor_messages::msg::Feedback>(
                "/dig_motor/status",
                10,
                std::bind(&DigActionServer::dig_status_callback, this, _1));

        dig_publisher_ =
            this->create_publisher<motor_messages::msg::Command>("/dig_motor/control", 10);
        RCLCPP_INFO(this->get_logger(), "Dig action server started");
    }

private:
    rclcpp_action::Server<MotorControl>::SharedPtr action_server_;
    rclcpp::Subscription<motor_messages::msg::Feedback>::SharedPtr dig_feedback_subscriber_;
    rclcpp::Publisher<motor_messages::msg::Command>::SharedPtr dig_publisher_;

    float dig_position_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const MotorControl::Goal> goal)
    {
        (void)uuid;

        RCLCPP_INFO(
            this->get_logger(),
            "Received goal request: target_position=%d",
            goal->target_position);

        if (goal->target_position < COLLECT || goal->target_position > STOW)
        {
            RCLCPP_WARN(this->get_logger(), "Rejecting invalid goal");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMotor> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMotor> goal_handle)
    {
        std::thread{std::bind(&DigActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMotor> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        const auto goal = goal_handle->get_goal();
        const DigPosition commanded_position =
            static_cast<DigPosition>(goal->target_position);

        auto result = std::make_shared<MotorControl::Result>();
        result->final_positions.push_back(dig_position_);

        auto feedback = std::make_shared<MotorControl::Feedback>();
        feedback->positions.push_back(dig_position_);

        float target = 0.0f;
        // target = positionMap.at(commanded_position);

        switch (commanded_position)
        {
        case STOW:
            target = DIG_STOW_POSITION;
            break;
        case COLLECT:
            target = DIG_COLLECT_POSITION;
            break;
        case DEPOSIT:
            target = DIG_DEPOSIT_POSITION;
            break;
        default:
            result->final_positions.push_back(dig_position_);
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Invalid input position");
            return;
        }

        motor_messages::msg::Command dig_msg;
        dig_msg.position.data = target;
        dig_publisher_->publish(dig_msg);

        rclcpp::Rate loop_rate(10);

        while (rclcpp::ok()) // potentially dangerous. May want watchdog timer.
        {
            if (goal_handle->is_canceling())
            {
                result->final_positions.push_back(dig_position_);
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            feedback->positions.clear();
            feedback->positions.push_back(dig_position_);
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Current dig position: %.4f", dig_position_);
            RCLCPP_INFO(this->get_logger(), "Current dig target: %.4f", target);

            if (std::fabs(dig_position_ - target) <= DIG_THRESHOLD) // publish feedback until position is reached, then exit
            {
                result->final_positions.push_back(dig_position_);
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                return;
            }
            
            dig_publisher_->publish(dig_msg);
            loop_rate.sleep();
        }

        result->final_positions.push_back(dig_position_);
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Strange error.");
    }

    void dig_status_callback(const motor_messages::msg::Feedback::SharedPtr msg)
    {
        dig_position_ = msg->position.data;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DigActionServer>(rclcpp::NodeOptions{});
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
