#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "dump/action/dump.hpp"

class AutoDump
{
public:
    AutoDump(rclcpp::Node* owner_node);
    ~AutoDump();
    void auto_dump(int goal_position);
    void cancel_dump();
    bool is_running();

private:
    void run_auto_dump(int goal_position);
    bool send_dump_goal(int goal_position);

    rclcpp_action::Client<dump::action::Dump>::SharedPtr dump_client_;
    rclcpp_action::ClientGoalHandle<dump::action::Dump>::SharedPtr active_goal_handle_;
    std::thread driver_thread;
    rclcpp::Node* owner_node;
    std::atomic_bool running;
    std::atomic_bool cancel_requested;
    std::atomic_bool dump_goal_succeeded;

    std::mutex goal_mutex;
};