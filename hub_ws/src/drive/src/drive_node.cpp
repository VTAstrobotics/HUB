#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "motor_messages/msg/command.hpp"
#include "motor_control/sparkmax_controller.hpp"
#include "motor_control/motor_controller_base.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_broadcaster.h"

#include "motor.hpp"

#define reaper_wheelbase 0.58
#define anubis_wheelbase 1.2

using std::placeholders::_1;
class Drive : public rclcpp::Node
{
public:
  Drive()
      : Node("drive_node")
  {
    //------------------------Publishers and Subscribers
    cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&Drive::cmd_vel_callback, this, _1));
    odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    //--------------------------Config logic
    this->declare_parameter<std::string>("robot", "REAPER");
    const std::string robot_name = this->get_parameter("robot").as_string();

    this->declare_parameter<double>("wheelbase", anubis_wheelbase);
    this->wheelbase = this->get_parameter("wheelbase").as_double();

    this->declare_parameter<double>("motor_gear_ratio", 125.0);
      
    this->declare_parameter<double>("wheel_diameter", 0.31);
    this->wheel_diameter = this->get_parameter("wheel_diameter").as_double();

    this->declare_parameter<double>("odom_update_rate", 50.0); // Hz. Theoretically higher is better but our motors only update so quickly
    this->odom_update_rate = this->get_parameter("odom_update_rate").as_double();

    this->declare_parameter<std::vector<std::string>>("left_motor_names", std::vector<std::string>({"left_front", "left_back"}));
    std::vector<std::string> left_motors_names;
      left_motors_names = this->get_parameter("left_motor_names").as_string_array();
    for (auto &&motor_name : left_motors_names)
    {
      left_motors.push_back(
          std::make_shared<motor>(motor_name, this));

    }
    this->declare_parameter<std::vector<std::string>>("right_motor_names", std::vector<std::string>({"right_front", "right_back"}));
    std::vector<std::string> right_motors_names;
      right_motors_names = this->get_parameter("right_motor_names").as_string_array();
    for (auto &&motor_name : right_motors_names)
    {
      right_motors.push_back(
          std::make_shared<motor>(motor_name, this));

    }
    


    //------------------------Timers
    int64_t odom_period_ms = 1000 * (1.0 / odom_update_rate);
    this->odom_timer = this->create_wall_timer(
        std::chrono::milliseconds(odom_period_ms),
        std::bind(&Drive::update_odometry, this));
  }


  // /**
  //  * Add motor node instances to exec
  //  */
  // void add_motors(const std::shared_ptr<rclcpp::Executor> &exec)
  // {
  //   for (auto &n : left_motors)
  //   {
  //     exec->add_node(n);
  //   }
  //   for (auto &n : right_motors)
  //   {
  //     exec->add_node(n);
  //   }
  // }

private:
  struct pose2d // https://www.ros.org/reps/rep-0103.html
  {
    double x;          // meter
    double y;          // meter
    double theta;      // radian counter clockwise positive
    rclcpp::Time time; // seconds(what time was this pose recorded at)
  };

  struct velocity2d
  {
    double linear;    // meters per second
    double angular_z; //(yaw) radian per second
  };

  /*
  Converts velocity into REAPER RPM
  */
  double vel_to_rpm(double velocity)
  {
    return (velocity * 60.0  * motor_gear_ratio) / ( ( (wheel_diameter) / 2.0)  * 2 * M_PI);
  }

  /*
  Converts REAPER drive RPM to velocity
  */
  double rpm_to_vel(double rpm)
  {
    return (rpm * (2 * M_PI) * wheel_diameter / 2.0) / (60.0 * motor_gear_ratio);
  }

  /**
   * reacts to /cmd_velocity message, and distrubutes velocity to left_right topics
   */
  void cmd_vel_callback(geometry_msgs::msg::Twist::SharedPtr msg)
  {
    double lin_x = msg->linear.x;
    double ang_z = msg->angular.z;
    // RCLCPP_INFO(this->get_logger(), "Driving With cmd_vel/");

    // velocity control
    double left_vel = ((lin_x - 0.5 * ang_z * wheelbase));
    double right_vel = (-(lin_x + 0.5 * ang_z * wheelbase));

    // std::cout << "VELOCITY: " << left_vel << " | " << right_vel << std::endl;
    // double left_rpm = vel_to_rpm(left_vel);
    // double right_rpm = vel_to_rpm(right_vel);

    // std::cout << "RPM: " << left_rpm << " | " << right_rpm << "\n"
    //           << std::endl;


    motor_messages::msg::Command right_velocity_msg;
    motor_messages::msg::Command left_velocity_msg;

    left_velocity_msg.dutycycle.data = left_vel;
    right_velocity_msg.dutycycle.data = right_vel;

    for (auto &&i : left_motors)
    {
      i->send_command(left_velocity_msg);
    }
    for (auto &&i : right_motors)
    {
      i->send_command(right_velocity_msg);
    }
    
  }

  pose2d integrate_velocity(pose2d current_pose, velocity2d vel)
  {
    pose2d new_pose;
    new_pose.x = 0;
    new_pose.y = 0;
    new_pose.theta = 0;
    new_pose.time = this->get_clock()->now();
    new_pose.x = 0;
    new_pose.y = 0;
    new_pose.theta = 0;
    new_pose.time = this->get_clock()->now();
    double dt = (this->get_clock()->now() - current_pose.time).seconds();
    // RCLCPP_INFO(this->get_logger(), "Creating Drive REAPER");
    // RCLCPP_INFO(this->get_logger(), "Creating Drive REAPER");
    if (dt < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Time went backwards somehow in odometry integration");
      dt = 0; // just dont change the state estimate
    }
    if (dt > 2 * (1 / odom_update_rate))
    {
      RCLCPP_WARN(this->get_logger(), "Large dt detected in odometry integration: %f seconds", dt);
    }

    double linear_x = vel.linear * cos(current_pose.theta);
    double linear_y = vel.linear * sin(current_pose.theta);
    new_pose.x = current_pose.x + linear_x * dt;
    new_pose.y = current_pose.y + linear_y * dt;
    ;
    new_pose.theta = current_pose.theta + vel.angular_z * dt;
    return new_pose;
  }



  void update_odometry()
  {
    // RCLCPP_INFO(this->get_logger(), "Updating Odom");
    odom_mutex.lock();
    if ((last_left_feedback != nullptr) && (last_right_feedback != nullptr))
    {
      if ((last_left_feedback != nullptr) && (last_right_feedback != nullptr))
      {
        current_velocity.linear = (rpm_to_vel(last_left_feedback->velocity.data) + rpm_to_vel(-last_right_feedback->velocity.data)) / 2.0;
        current_velocity.angular_z = (rpm_to_vel (-last_right_feedback->velocity.data) - rpm_to_vel(last_left_feedback->velocity.data)) / wheelbase;
        current_pose = integrate_velocity(current_pose, current_velocity);
      }
      publish_odometry();
    }
    odom_mutex.unlock();
  }

  void publish_odometry()
  {
    // RCLCPP_INFO(this->get_logger(), "Publishing Odom");
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->get_clock()->now();
    // RCLCPP_WARN(this->get_logger(), "Time: %f", this->get_clock()->now().seconds());
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = current_pose.x;
    odom_msg.pose.pose.position.y = current_pose.y;
    odom_msg.pose.pose.position.z = 0.0; // no flying here

    odom_msg.pose.pose.orientation = tf2::toMsg(
        tf2::Quaternion(0, 0, sin(current_pose.theta / 2.0), cos(current_pose.theta / 2.0)));

    odom_msg.twist.twist.linear.x = current_velocity.linear * cos(current_pose.theta);
    odom_msg.twist.twist.linear.y = current_velocity.linear * sin(current_pose.theta);
    odom_msg.twist.twist.linear.z = 0.0; // no upwards movement
    odom_msg.twist.twist.angular.z = current_velocity.angular_z;
    odom_publisher->publish(odom_msg);
  }

  //----------------------publishers/subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
  rclcpp::Publisher<motor_messages::msg::Command>::SharedPtr left_velocity_publisher;
  rclcpp::Publisher<motor_messages::msg::Command>::SharedPtr right_velocity_publisher;

  std::vector<std::shared_ptr<motor>> left_motors;
  std::vector<std::shared_ptr<motor>> right_motors;



  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;


  //------------------------Timer
  rclcpp::TimerBase::SharedPtr odom_timer;

  //------------------------data variables

  std::unordered_map<std::string, double> left_velocity;  // currently unused but will be needed for multi motor
  std::unordered_map<std::string, double> right_velocity; // currently unused but will be needed for multi motor

  double wheelbase; // in meters
  double motor_gear_ratio;
  double wheel_diameter; // in meters

  /**
   * These are used to update the odometry and ensure that things are synchronized ish hopefully
   * IDK i'm not a multithreading expert
   */
  motor_messages::msg::Feedback::SharedPtr last_left_feedback; // temps for the single motor config
  motor_messages::msg::Feedback::SharedPtr last_right_feedback;

  std::mutex odom_mutex;
  pose2d current_pose{0.0, 0.0, 0.0, this->get_clock()->now()};
  velocity2d current_velocity{0.0, 0.0};
  double odom_update_rate; // Hz
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Drive>());
  rclcpp::shutdown();
  return 0;
}
