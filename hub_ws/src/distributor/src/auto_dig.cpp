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

AutoDig::AutoDig(rclcpp::Node *owner_node)
{
    this->owner_node = owner_node;
    velocity_publisher_ = this->owner_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    dig_client_ = rclcpp_action::create_client<MotorControl>(owner_node, "motor_control");

    running = false;
    cancel_requested = false;
    dig_goal_succeeded = false;
}

AutoDig::~AutoDig()
{
    cancel_dig();

    if (driver_thread.joinable())
    {
        driver_thread.join();
    }
}

void AutoDig::auto_dig(float drive_time_seconds)
{
    if (running)
    {
        RCLCPP_INFO(owner_node->get_logger(), "Auto dig already running");
        return;
    }
    if (driver_thread.joinable())
    {
        driver_thread.join();
    }
    running = true;

    driver_thread = std::thread([this, drive_time_seconds]()
                                { run_auto_dig(drive_time_seconds); });
}

void AutoDig::cancel_dig()
{
    cancel_requested = true;
    geometry_msgs::msg::Twist stop_msg; // all 0
    velocity_publisher_->publish(stop_msg);

    GoalHandleMotor::SharedPtr goal;
    {
        std::lock_guard<std::mutex> lock(goal_mutex);
        goal = active_goal_handle_;
    }

    if (goal)
    {
        dig_client_->async_cancel_goal(goal); // cancel current goal
    }
}

bool AutoDig::is_running()
{
    return running;
}

void AutoDig::run_auto_dig(float drive_time_seconds)
{
    dig_goal_succeeded = false;
    cancel_requested = false;
    dig_goal_succeeded = false;

    RCLCPP_INFO(owner_node->get_logger(), "Starting autodig");
    bool dig_status = send_dig_goal(0);
    if (!dig_status)
    {
        running = false;
        return;
    }
    auto start = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(10);

    while (!dig_goal_succeeded)
    {
        if (cancel_requested)
        {
            running = false;
            return;
        }

        if (std::chrono::steady_clock::now() - start > timeout)
        {
            RCLCPP_WARN(owner_node->get_logger(), "Dig goal timeout");
            dig_goal_succeeded = false;
            running = false;
            return;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    dig_goal_succeeded = false;
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.2;
    velocity_publisher_->publish(cmd);

    start = std::chrono::steady_clock::now();

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
    dig_goal_succeeded = false;
    dig_status = send_dig_goal(1);
    if (!dig_status)
    {
        running = false;
        return;
    }
    start = std::chrono::steady_clock::now();
    while (!dig_goal_succeeded)
    {
        if (cancel_requested)
        {
            running = false;
            return;
        }
        if (std::chrono::steady_clock::now() - start > timeout)
        {
            RCLCPP_WARN(owner_node->get_logger(), "Dig goal timeout");
            dig_goal_succeeded = false;
            running = false;
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (cancel_requested)
    {
        running = false;
        RCLCPP_WARN(owner_node->get_logger(), "Autodig canceled during drive");
        return;
    }
    running = false;
}

bool AutoDig::send_dig_goal(int target_position)
{
    if (!dig_client_->wait_for_action_server(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(owner_node->get_logger(), "Dig action server unavailable");
        return false;
    }
    MotorControl::Goal goal;
    goal.target_position = target_position;
    rclcpp_action::Client<MotorControl>::SendGoalOptions options;

    options.goal_response_callback =
        [this](GoalHandleMotor::SharedPtr goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(owner_node->get_logger(), "Dig goal rejected");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(goal_mutex);
            active_goal_handle_ = goal_handle;
        }

        RCLCPP_INFO(owner_node->get_logger(), "Dig goal accepted");
    };

    options.result_callback =
        [this](const GoalHandleMotor::WrappedResult &result)
    {
        {
            std::lock_guard<std::mutex> lock(goal_mutex);
            active_goal_handle_.reset();
        }

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(owner_node->get_logger(), "Dig goal succeeded");
            dig_goal_succeeded = true;
        }
        else
        {
            RCLCPP_WARN(owner_node->get_logger(), "Dig goal did not succeed");
        }
    };

    dig_client_->async_send_goal(goal, options);
    return true;
}
