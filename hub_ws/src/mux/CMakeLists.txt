cmake_minimum_required(VERSION 3.8)
project(mux)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)

find_package(motor_messages REQUIRED)

add_executable(dig_mux_node src/dig_mux.cpp src/motor.cpp)

add_executable(dump_mux_node src/dig_mux.cpp src/motor.cpp)


target_include_directories(dig_mux_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(dump_mux_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(dig_mux_node
  rclcpp
  std_msgs
  motor_messages
)

ament_target_dependencies(dump_mux_node
  rclcpp
  std_msgs
  motor_messages
)

install(TARGETS dig_mux_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS dump_mux_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
