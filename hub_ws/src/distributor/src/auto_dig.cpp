#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "dig/action/motor_control.hpp"
#include "auto_dig.hpp"

void AutoDig::AutoDig(rclcpp::Node *owner_node)
{
    this->owner_node = owner_node;
    velocity_publisher_ = this->owner_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    dig_client_ = rclcpp_action::create_client<MotorControl>(owner_node, "motor_control");
}

void AutoDig::auto_dig(float drive_time_seconds)
{
    running = true;

    driver_thread = std::thread([this, drive_time_seconds]()
                                { run_auto_dig(drive_time_seconds); });
}

void AutoDig::cancel_dig()
{
    cancel_requested = true;
    geometry_msgs::msg::Twist stop; // all 0
    velocity_publisher_->publish(stop);

    if (active_goal_handle_)
    {
        dig_client_->async_cancel_goal(active_goal_handle_); // cancel current goal
    }
}

bool AutoDig::is_running()
{
    return running;
}

void AutoDig::run_auto_dig()
{
    RCLCPP_INFO(node_->get_logger(), "Starting autodig");
    send_dig_goal(0);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.2;
    velocity_publisher_->publish(cmd);

    auto start = std::chrono::steady_clock::now();

    while (!cancel_requested)
    {
        auto elapsed =
            std::chrono::duration<float>(
                std::chrono::steady_clock::now() - start)
                .count();

        if (elapsed >= drive_time_seconds)
        {
            break;
        }

        velocity_publisher_->publish(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    geometry_msgs::msg::Twist stop;
    velocity_publisher_->publish(stop);

    send_dig_goal(1);

    if (cancel_requested_)
    {
        running = false;
        RCLCPP_WARN(node_->get_logger(), "Autodig canceled during drive");
        return;
    }
    running = false;
}

void AutoDig::send_dig_goal(int target_position)
    {
        if (!dig_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(node_->get_logger(), "Dig action server unavailable");
            return;
        }
        MotorControl::Goal goal;
        goal.target_position = target_position;
        rclcpp_action::Client<MotorControl>::SendGoalOptions options;

        options.goal_response_callback =
            [this](GoalHandleMotor::SharedPtr goal_handle)
            {
                if (!goal_handle)
                {
                    RCLCPP_ERROR(node_->get_logger(), "Dig goal rejected");
                    return;
                }

                active_goal_handle_ = goal_handle;
                RCLCPP_INFO(node_->get_logger(), "Dig goal accepted");
            };

        options.result_callback =
            [this](const GoalHandleMotor::WrappedResult & result)
            {
                active_goal_handle_.reset();

                if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                {
                    RCLCPP_INFO(node_->get_logger(), "Dig goal succeeded");
                }
                else
                {
                    RCLCPP_WARN(node_->get_logger(), "Dig goal did not succeed");
                }
            };

        dig_client_->async_send_goal(goal, options);
    }
