cmake_minimum_required(VERSION 3.8)
project(robotcan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
include_directories(include)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robotcan_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(test_can src/test.cpp)
ament_target_dependencies(test_can rclcpp)

add_executable(robot_can src/robot_test.cpp)
ament_target_dependencies(robot_can rclcpp)

add_executable(robot_srv src/robot_srv.cpp)
ament_target_dependencies(robot_srv rclcpp robotcan_interfaces)

add_executable(robot_uart src/robot_test_uart.cpp)
ament_target_dependencies(robot_uart rclcpp)

target_link_libraries(test_can)
target_link_libraries(robot_can)
target_link_libraries(robot_uart)

install(TARGETS
  test_can
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  robot_can
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  robot_uart
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
