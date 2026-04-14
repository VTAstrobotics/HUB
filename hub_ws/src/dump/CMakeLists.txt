cmake_minimum_required(VERSION 3.8)
project(dump)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(motor_messages REQUIRED)
find_package(motor_control REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(motor_messages REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Dump.action"
)

add_library(action_server SHARED
  src/dump_auto_action_server.cpp)

add_executable(dump_teleop_node src/dump_teleop.cpp src/motor.cpp)


target_compile_definitions(action_server
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")

ament_target_dependencies(action_server
  rclcpp
  std_msgs
  motor_messages
)

ament_target_dependencies(action_server
  rclcpp
  rclcpp_action
  rclcpp_components
  motor_messages
  motor_control
)

ament_target_dependencies(dump_teleop_node
  rclcpp
  rclcpp_components
  motor_messages
  motor_control
)

rosidl_target_interfaces(action_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rclcpp_components_register_node(action_server
  PLUGIN "DumpAutoActionServer"
  EXECUTABLE dump_auto_action_server)

target_link_libraries(action_server
  motor_control::motor_control_nodes)

install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS dump_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
