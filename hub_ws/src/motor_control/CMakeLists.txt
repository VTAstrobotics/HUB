cmake_minimum_required(VERSION 3.8)
project(motor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(motor_messages REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(phoenix6 REQUIRED)
add_definitions(-DUNIT_LIB_DISABLE_FMT -DUNIT_LIB_ENABLE_IOSTREAM)

add_library(motor_control_nodes SHARED
  src/kraken_controller.cpp
  src/sparkmax_controller.cpp
  src/spark_base.cpp
  src/spark_max.cpp
)

ament_target_dependencies(motor_control_nodes rclcpp motor_messages rclcpp_components phoenix6)

target_include_directories(motor_control_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(kraken_control_node src/kraken_controller_executable.cpp src/kraken_controller.cpp)
add_executable(sparkmax_control_node src/sparkmax_controller_executable.cpp src/sparkmax_controller.cpp src/spark_base.cpp src/spark_max.cpp)

ament_target_dependencies(kraken_control_node rclcpp motor_messages rclcpp_components)
target_include_directories(kraken_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(sparkmax_control_node rclcpp motor_messages rclcpp_components)
target_include_directories(kraken_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(sparkmax_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(kraken_control_node phoenix6 motor_control_nodes)
target_link_libraries(sparkmax_control_node motor_control_nodes)

install(TARGETS
  motor_control_nodes
  EXPORT export_motor_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  kraken_control_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  sparkmax_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_targets(export_motor_control HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_components motor_messages phoenix6)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
