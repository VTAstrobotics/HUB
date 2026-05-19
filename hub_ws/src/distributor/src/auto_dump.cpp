#include "auto_dump.hpp"

AutoDump::AutoDump(rclcpp::Node *owner_node)
{
    this->owner_node = owner_node;
    dump_client_ = rclcpp_action::create_client<dump::action::Dump>(owner_node, "dump_auto");
    velocity_publisher_ = this->owner_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    dig_client_ = rclcpp_action::create_client<dig::action::MotorControl>(owner_node, "motor_control");

    running = false;
    cancel_requested = false;
    dump_goal_succeeded = false;
    dig_goal_succeeded = false;
}

AutoDump::~AutoDump()
{
    cancel_dump();
    if (driver_thread.joinable())
    {
        driver_thread.join();
    }
}

void AutoDump::auto_dump(int goal_position)
{
    if (running)
    {
        RCLCPP_INFO(owner_node->get_logger(), "Auto dump already running");
        return;
    }
    if (driver_thread.joinable())
    {
        driver_thread.join();
    }
    running = true;
    driver_thread = std::thread([this, goal_position]()
                                { run_auto_dump(goal_position); });
    RCLCPP_INFO(owner_node->get_logger(), "Auto dump thread created");
}

void AutoDump::cancel_dump()
{
    cancel_requested = true;

    rclcpp_action::ClientGoalHandle<dump::action::Dump>::SharedPtr goal;
    {
        std::lock_guard<std::mutex> lock(goal_mutex);
        goal = active_goal_handle_;
    }
    if (goal)
    {
        dump_client_->async_cancel_goal(goal);
    }

    rclcpp_action::ClientGoalHandle<dig::action::MotorControl>::SharedPtr dig_goal;
    {
        std::lock_guard<std::mutex> lock(goal_mutex);
        dig_goal = active_dig_goal_handle_;
    }

    if (dig_goal)
    {
        dig_client_->async_cancel_goal(dig_goal); // cancel current goal
    }
    running = false;
}

bool AutoDump::is_running()
{
    return running;
}

void AutoDump::run_auto_dump(int goal_position)
{
    cancel_requested = false;
    dig_goal_succeeded = false;
    dump_goal_succeeded = false;

    RCLCPP_INFO(owner_node->get_logger(), "Starting auto dump!");
    if (goal_position == 0) // move bucket out of way - this is not abstracted heavily enough
    {
        bool dig_status = send_dig_goal(2);
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
                cancel_dump();
                return;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    bool status = send_dump_goal(goal_position);
    if (!status)
    {
        running = false;
        return;
    }

    if (goal_position == 1)
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = -0.2;
        velocity_publisher_->publish(cmd);

        auto start_drive = std::chrono::steady_clock::now();

        float dump_time_seconds = 1.3;

        while (!cancel_requested)
        {
            auto elapsed =
                std::chrono::duration<float>(
                    std::chrono::steady_clock::now() - start_drive)
                    .count();

            if (elapsed >= dump_time_seconds)
            {
                break;
            }

            velocity_publisher_->publish(cmd);
            RCLCPP_INFO(owner_node->get_logger(), "Auto drive initiated");

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        geometry_msgs::msg::Twist stop;
        velocity_publisher_->publish(stop);
    }

    auto start = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(10);

    while (!dump_goal_succeeded)
    {
        if (cancel_requested)
        {
            running = false;
            return;
        }
        if (std::chrono::steady_clock::now() - start > timeout)
        {
            RCLCPP_WARN(owner_node->get_logger(), "Dump goal timeout");
            running = false;
            cancel_dump();
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    running = false;
}

bool AutoDump::send_dump_goal(int goal_position)
{
    if (!dump_client_->wait_for_action_server(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(owner_node->get_logger(), "DUMP ACTION SERVER NOT LAUNCHED");
        return false;
    }

    dump::action::Dump::Goal goal;
    goal.goal_position = goal_position;

    rclcpp_action::Client<dump::action::Dump>::SendGoalOptions options;

    options.goal_response_callback =
        [this](rclcpp_action::ClientGoalHandle<dump::action::Dump>::SharedPtr goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(owner_node->get_logger(), "Dump goal rejected");
            return;
        }
        {
            std::lock_guard<std::mutex> lock(goal_mutex);
            active_goal_handle_ = goal_handle;
        }
        RCLCPP_INFO(owner_node->get_logger(), "Dump goal accepted");
    };

    options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<dump::action::Dump>::WrappedResult &result)
    {
        {
            std::lock_guard<std::mutex> lock(goal_mutex);
            active_goal_handle_.reset();
        }
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(owner_node->get_logger(), "Dump goal succeeded");
            dump_goal_succeeded = true;
        }
        else
        {
            RCLCPP_WARN(owner_node->get_logger(), "Dump goal did not succeed");
        }
    };

    dump_client_->async_send_goal(goal, options);
    return true;
}

bool AutoDump::send_dig_goal(int target_position)
{
    if (!dig_client_->wait_for_action_server(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(owner_node->get_logger(), "DIG ACTION SERVER NOT LAUNCHED");
        return false;
    }
    dig::action::MotorControl::Goal goal;
    goal.target_position = target_position;
    rclcpp_action::Client<dig::action::MotorControl>::SendGoalOptions options;

    options.goal_response_callback =
        [this](rclcpp_action::ClientGoalHandle<dig::action::MotorControl>::SharedPtr goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(owner_node->get_logger(), "Dig goal rejected");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(goal_mutex);
            active_dig_goal_handle_ = goal_handle;
        }

        RCLCPP_INFO(owner_node->get_logger(), "Dig goal accepted");
    };

    options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<dig::action::MotorControl>::WrappedResult &result)
    {
        {
            std::lock_guard<std::mutex> lock(goal_mutex);
            active_dig_goal_handle_.reset();
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