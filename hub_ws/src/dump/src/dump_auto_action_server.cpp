#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <mutex>
#include <cmath>

#include "dump/action/dump.hpp"

#include "motor_messages/msg/command.hpp"
#include "motor_messages/msg/feedback.hpp"

// The vast majority of these are placeholders
// We need to find out these places/speed when testing
#define DOOR_HOME_POSITION 0
#define ACTUATOR_HOME_POSITION 0
#define DOOR_DUMP_POSITION 40.0
#define ACTUATOR_DUMP_POSITION 0.150
#define DOOR_POSITION_THRESHOLD 1.5
#define ACTUATOR_POSITION_THRESHOLD 0.010

#define DOOR_DUTY_CYCLE 0.05
#define ACTUATOR_DUTY_CYCLE 1.0

#define SERVER_LOOP_FREQUENCY 20

using namespace std::placeholders;
using Dump = dump::action::Dump;
using GoalHandleDump = rclcpp_action::ServerGoalHandle<Dump>;

enum DumpPosition
{
    HOME,
    DUMP
};

class DumpAutoActionServer : public rclcpp::Node
{
public:
    DumpAutoActionServer(const rclcpp::NodeOptions &options)
        : Node("dump_auto_action_server")
    {
        dump_action_server = rclcpp_action::create_server<Dump>(
            this,
            "dump_auto",
            std::bind(&DumpAutoActionServer::handle_goal, this, _1, _2),
            std::bind(&DumpAutoActionServer::handle_cancel, this, _1),
            std::bind(&DumpAutoActionServer::handle_accepted, this, _1));

        door_feedback_subscriber = this->create_subscription<motor_messages::msg::Feedback>(
            "/dump_bucket_teleop/status", 10, std::bind(&DumpAutoActionServer::dump_door_status_callback, this, _1));
        linear_actuator_feedback_subscriber = this->create_subscription<motor_messages::msg::Feedback>(
            "/dump_linear_actuator/status", 10, std::bind(&DumpAutoActionServer::dump_actuator_status_callback, this, _1));

        door_duty_publisher = this->create_publisher<motor_messages::msg::Command>("/dump_bucket_teleop/control", 4);
        linear_actuator_duty_publisher = this->create_publisher<motor_messages::msg::Command>("/dump_linear_actuator/control", 4);
    }

private:
    rclcpp_action::Server<Dump>::SharedPtr dump_action_server;
    rclcpp::Subscription<motor_messages::msg::Feedback>::SharedPtr door_feedback_subscriber;
    rclcpp::Subscription<motor_messages::msg::Feedback>::SharedPtr linear_actuator_feedback_subscriber;
    rclcpp::Publisher<motor_messages::msg::Command>::SharedPtr door_duty_publisher;
    rclcpp::Publisher<motor_messages::msg::Command>::SharedPtr linear_actuator_duty_publisher;

    bool startup_sequence = true;
    float door_position = 0;
    float actuator_position = 0;

    rclcpp::Time last_goal_time = this->get_clock()->now();
    rclcpp::Duration min_interval{1, 0};

    std::shared_ptr<GoalHandleDump> active_goal;
    std::mutex goal_mutex;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Dump::Goal> goal)
    {
        (void)uuid;

        std::lock_guard<std::mutex> lock(goal_mutex);

        auto now = this->now();
        if ((now - last_goal_time) < min_interval)
        {
            RCLCPP_WARN(this->get_logger(), "Goal rejected: too soon");
            return rclcpp_action::GoalResponse::REJECT;
        }

        last_goal_time = now;

        RCLCPP_INFO(this->get_logger(), "Goal accepted");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleDump> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleDump> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::lock_guard<std::mutex> lock(goal_mutex);

        // if (active_goal)
        // {
        //     auto result = std::make_shared<Dump::Result>();
        // active_goal->canceled(result);
        // }

        active_goal = goal_handle;

        std::thread{std::bind(&DumpAutoActionServer::execute, this, _1), goal_handle}.detach();
    }

    // Homing system on startup still required
    void execute(const std::shared_ptr<GoalHandleDump> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        DumpPosition commandedPosition = static_cast<DumpPosition>(goal_handle->get_goal()->goal_position);
        auto feedback = std::make_shared<Dump::Feedback>();
        auto result = std::make_shared<Dump::Result>();

        auto &positions = feedback->positions;
        positions.push_back(door_position);
        positions.push_back(actuator_position);
        rclcpp::Rate loop_rate(SERVER_LOOP_FREQUENCY);

        while (rclcpp::ok())
        {
            {
                std::lock_guard<std::mutex> lock(goal_mutex);
                if (goal_handle != active_goal || goal_handle->is_canceling())
                {
                    motor_messages::msg::Command door_stop_msg;
                    motor_messages::msg::Command actuator_stop_msg;
                    door_stop_msg.dutycycle.data = 0;
                    actuator_stop_msg.dutycycle.data = 0;
                    door_duty_publisher->publish(door_stop_msg);
                    linear_actuator_duty_publisher->publish(actuator_stop_msg);
                    result->final_positions = positions;
                    if (goal_handle->is_canceling() && goal_handle->is_active())
                    {
                        goal_handle->canceled(result);
                    }
                    RCLCPP_INFO(this->get_logger(), "Goal canceled");
                    return;
                }
            }
            positions.clear();
            positions.push_back(door_position);
            positions.push_back(actuator_position);
            goal_handle->publish_feedback(feedback);

            bool door_complete = false;
            bool actuator_complete = false;
            motor_messages::msg::Command door_msg;
            motor_messages::msg::Command actuator_msg;

            switch (commandedPosition)
            {
            case HOME:
                if (fabs(door_position - DOOR_HOME_POSITION) < DOOR_POSITION_THRESHOLD)
                {
                    door_complete = true;
                    door_msg.dutycycle.data = 0;
                }
                else
                {
                    door_msg.dutycycle.data = -DOOR_DUTY_CYCLE;
                }
                if (fabs(actuator_position - ACTUATOR_HOME_POSITION) < ACTUATOR_POSITION_THRESHOLD)
                {
                    actuator_complete = true;
                    actuator_msg.dutycycle.data = 0;
                }
                else
                {
                    actuator_msg.dutycycle.data = ACTUATOR_DUTY_CYCLE;
                }
                break;
            case DUMP:
                if (fabs(door_position - DOOR_DUMP_POSITION) < DOOR_POSITION_THRESHOLD)
                {
                    door_complete = true;
                    door_msg.dutycycle.data = 0;
                }
                else
                {
                    door_msg.dutycycle.data = DOOR_DUTY_CYCLE;
                }
                if ((fabs(actuator_position - ACTUATOR_DUMP_POSITION) < ACTUATOR_POSITION_THRESHOLD) || actuator_position > ACTUATOR_DUMP_POSITION)
                {
                    actuator_complete = true;
                    RCLCPP_INFO(this->get_logger(), "Actuator Done");
                    actuator_msg.dutycycle.data = 0;
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Actuator Running");
                    actuator_msg.dutycycle.data = -ACTUATOR_DUTY_CYCLE;
                }
                break;
            }
            door_duty_publisher->publish(door_msg);
            linear_actuator_duty_publisher->publish(actuator_msg);
            RCLCPP_INFO(this->get_logger(), "Actuator Running at %f", actuator_msg.dutycycle.data);

            RCLCPP_INFO(this->get_logger(), "Door running at %f", door_msg.dutycycle.data);
            if (door_complete && actuator_complete)
                break;

            loop_rate.sleep();
        }

        // Check if goal is done
        if (rclcpp::ok())
        {
            result->final_positions = positions;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }

    void dump_door_status_callback(motor_messages::msg::Feedback msg)
    {
        door_position = msg.position.data;
    }
    void dump_actuator_status_callback(motor_messages::msg::Feedback msg)
    {
        actuator_position = msg.position.data;
        RCLCPP_INFO(this->get_logger(), "Dump Position: %f", msg.position.data);
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(DumpAutoActionServer)