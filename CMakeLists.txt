cmake_minimum_required(VERSION 3.8)
project(kinova_moveit_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(kinova_wrapper REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bridge_integration test/test_bridge_integration.cpp src/kinova_moveit_bridge_node.cpp src/kinova_gripper_bridge.cpp)
  ament_target_dependencies(test_bridge_integration
  rclcpp rclcpp_action control_msgs trajectory_msgs kinova_wrapper)

target_include_directories(test_bridge_integration PRIVATE include)
endif()

# ----- Single executable hosting both bridges -----
add_executable(kinova_bridge_node
  src/main.cpp
  src/kinova_moveit_bridge_node.cpp
  src/kinova_gripper_bridge.cpp
)

ament_target_dependencies(kinova_bridge_node
  rclcpp
  trajectory_msgs
  control_msgs
  action_msgs
  rclcpp_action
  kinova_wrapper
)

target_include_directories(kinova_bridge_node PUBLIC
  include include/${PROJECT_NAME}
)

install(TARGETS kinova_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
