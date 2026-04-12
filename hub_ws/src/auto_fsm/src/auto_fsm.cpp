#include "rclcpp/rclcpp.hpp"

#include "action_msgs/msg/goal_status_array.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include <mutex>

enum AUTO_STATES
{
    STARTUP,
    LOCALIZE,
    HOME,
    DRIVE_TO_DIG,
    DIG,
    DRIVE_TO_DUMP,
    DUMP
};

enum NAVIGATION_STATES
{
    NAVIGATING = 2,
    SUCCEEDED = 4,
    CANCELED = 5,
    ABORTED = 6,

};

struct Nav2Coordinates
{
    float x;
    float y;
    float w;
};

    using std::placeholders::_1;
class AutoFSM : public rclcpp::Node 
{
public:
    AutoFSM() : Node("auto_fsm_node") 
    {

        timer_ptr_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&AutoFSM::fsm_callback, this));
        auto_state = STARTUP;

        subscription_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
            "/navigate_to_pose/_action/status", 10,
            std::bind(&AutoFSM::status_callback, this, std::placeholders::_1));

        nav_client =
            rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/navigate_to_pose");

        dig_coordinates.x = 0;
        dig_coordinates.y = 0;
        dig_coordinates.w = 0;

        dump_coordinates.x = 0;
        dump_coordinates.y = 0;
        dump_coordinates.w = 0;
    }

private:
    rclcpp::TimerBase::SharedPtr timer_ptr_;
    AUTO_STATES auto_state;
    std::mutex nav_state_mutex;
    Nav2Coordinates dig_coordinates;
    Nav2Coordinates dump_coordinates;

    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr subscription_;

    action_msgs::msg::GoalStatusArray::SharedPtr nav2_status;
    NAVIGATION_STATES navigation_status_code;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client;

    void
    fsm_callback()
    {
        switch (auto_state)
        {
        case (STARTUP):
            // button press
            break;
        case (LOCALIZE): // spin to win
            break;
        case (HOME):
            // send home goal
            break;
        case (DRIVE_TO_DIG):
            handle_drive_to_dig();
            break;
        case (DIG):
            break;
        case (DRIVE_TO_DUMP):
            if (navigation_status_code == ABORTED)
                if (navigation_status_code == SUCCEEDED)
                {
                    auto_state = DUMP;
                    // send dump goal;
                    break;
                }
            break;
        case (DUMP):
            break;
        default:
            break;
        }
        // fsm
    }

    void status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
    {
        // if (msg->status_list.empty()) {
        //     RCLCPP_INFO(this->get_logger(), "No active navigation goals.");
        //     return;
        // }
        nav2_status = msg;

        auto latest_status = msg->status_list.back();
        navigation_status_code = static_cast<NAVIGATION_STATES>(latest_status.status);

        switch (navigation_status_code)
        {
        case NAVIGATING: // STATUS_EXECUTING
            RCLCPP_INFO(this->get_logger(), "Still navigating");
            break;
        case SUCCEEDED: // STATUS_SUCCEEDED
            RCLCPP_INFO(this->get_logger(), "Goal reached");
            break;
        case ABORTED: // STATUS_ABORTED
            RCLCPP_ERROR(this->get_logger(), "Navigation failed.");
            break;
        case CANCELED: // STATUS_CANCELED
            RCLCPP_WARN(this->get_logger(), "canceled");
            break;
        }
    }

    void handle_drive_to_dig()
    {
        switch (navigation_status_code)
        {
        case NAVIGATING:
            break;
        case SUCCEEDED:
            auto_state = DIG;
            break;
        case ABORTED:
            auto_state = LOCALIZE;
            break;
        case CANCELED:
            break;
        }
    }
    void handle_drive_to_dump()
    {
        switch (navigation_status_code)
        {
        case NAVIGATING:
            break;
        case SUCCEEDED:
            auto_state = DIG;
            break;
        case ABORTED:
            auto_state = LOCALIZE;
            break;
        case CANCELED:
            break;
        }
    }

    void send_nav2_goal(float x, float y, float w)
    {
        nav2_msgs::action::NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = this->now();
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        goal.pose.pose.position.z = 0.0;
        goal.pose.pose.orientation.w = w;
        nav_client->async_send_goal(goal);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoFSM>());
    rclcpp::shutdown();
    return 0;
}
