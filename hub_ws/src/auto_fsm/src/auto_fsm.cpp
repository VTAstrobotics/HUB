    #include "rclcpp/rclcpp.hpp"

    #include "action_msgs/msg/goal_status_array.hpp"
    #include "rclcpp_action/rclcpp_action.hpp"
    #include "nav2_msgs/action/navigate_to_pose.hpp"
    // button press topic
    #include "std_msgs/msg/bool.hpp"   


    #include "dump/action/dump.hpp"

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

    enum DumpPosition
    {
        //renamed to avoid clash with AUTO_STATES::HOME
        HOME_POS,
        DUMP_POS
    };

    struct Nav2Coordinates
    {
        float x;
        float y;
        float w;
    };

    using std::placeholders::_1;
    using Dump = dump::action::Dump;

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
            
            dump_client = rclcpp_action::create_client<Dump>(this, "/dump");

            button_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                "/start_button", 10,
                std::bind(&AutoFSM::button_callback, this, _1));

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

        bool button_pressed_   = false;
        bool home_goal_sent_   = false;
        bool home_goal_done_   = false;

        rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr subscription_;

        action_msgs::msg::GoalStatusArray::SharedPtr nav2_status;
        NAVIGATION_STATES navigation_status_code;

        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client;
        rclcpp_action::Client<Dump>::SharedPtr dump_client;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr button_sub_;
    

        void
        fsm_callback()
        {
            switch (auto_state)
            {
            case (STARTUP):
                handle_startup();  
                break;
            case (LOCALIZE): // spin to win
                break;
            case (HOME):
                handle_home();
                break;
            case (DRIVE_TO_DIG):
                handle_drive_to_dig();
                break;
            case (DIG):
                break;
            case (DRIVE_TO_DUMP):
                handle_drive_to_dump();
                break;
            case (DUMP):
                break;
            default:
                break;
            
        }
    }
        
       void button_callback(const std_msgs::msg::Bool::SharedPtr msg)
       {
        if (msg->data)
        {
            button_pressed_ = true;
            RCLCPP_INFO(this->get_logger(), "Start button pressed.");
        }
      }
        
        
        // STARTUP: wait for button, then go HOME
        void handle_startup()
        {
            if (!button_pressed_)
                return;

            RCLCPP_INFO(this->get_logger(), "STARTUP to HOME");
            // consume the event
            button_pressed_  = false;   
            // reset for a clean Home entry
            home_goal_sent_  = false;   
            home_goal_done_  = false;
            auto_state       = HOME;
        }

        void handle_home()
        {
            if (!home_goal_sent_)
            {
                if (!dump_client->wait_for_action_server(std::chrono::seconds(0)))
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                        "Waiting for dump action server…");
                    return;
                }

                auto goal_msg = Dump::Goal();
                goal_msg.target_position = DumpPosition::HOME_POS;  // retract

                auto send_goal_options = rclcpp_action::Client<Dump>::SendGoalOptions();

                send_goal_options.result_callback =
                    [this](const rclcpp_action::ClientGoalHandle<Dump>::WrappedResult & result)
                    {
                        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                        {
                            RCLCPP_INFO(this->get_logger(), "Dump retracted — HOME complete.");
                            home_goal_done_ = true;
                        }
                        else
                        {
                            RCLCPP_ERROR(this->get_logger(),
                                        "Dump retract failed (code %d) — retrying.",
                                        static_cast<int>(result.code));
                            // allow a retry next tick
                            home_goal_sent_ = false;   
                        }
                    };

                dump_client->async_send_goal(goal_msg, send_goal_options);
                home_goal_sent_ = true;
                RCLCPP_INFO(this->get_logger(), "HOME: sent dump-retract goal.");
                return;
            }

            if (home_goal_done_)
            {
                RCLCPP_INFO(this->get_logger(), "HOME → DRIVE_TO_DIG");
                auto_state = DRIVE_TO_DIG;
                send_nav2_goal(dig_coordinates.x, dig_coordinates.y, dig_coordinates.w);
            }
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
                auto_state = DUMP;
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
            nav_client->async_send_goal(goal); //probaby need handle goal options - 
        }
    };

    int main(int argc, char **argv)
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<AutoFSM>());
        rclcpp::shutdown();
        return 0;
    }
