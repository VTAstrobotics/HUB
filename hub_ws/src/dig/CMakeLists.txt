cmake_minimum_required(VERSION 3.8)
project(dig)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate action
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MotorControl.action"
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(dig_action_server src/dig_action_server.cpp)
ament_target_dependencies(dig_action_server
  rclcpp
  rclcpp_action
)

rosidl_target_interfaces(dig_action_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS
  dig_action_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()