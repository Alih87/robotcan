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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(robot_can src/robot_test.cpp)
ament_target_dependencies(robot_can rclcpp)

add_executable(robot_uart src/robot_test_uart.cpp)
ament_target_dependencies(robot_uart rclcpp)

add_executable(robot_srv src/robot_srv.cpp)
ament_target_dependencies(robot_srv rclcpp robotcan_interfaces)

add_executable(robot_pub src/robot_pub.cpp)
ament_target_dependencies(robot_pub rclcpp robotcan_interfaces geometry_msgs)

add_executable(robot_odom src/robot_odom.cpp)
ament_target_dependencies(robot_odom rclcpp geometry_msgs nav_msgs sensor_msgs)

target_link_libraries(robot_can)
target_link_libraries(robot_uart)
target_link_libraries(robot_srv)
target_link_libraries(robot_pub)
target_link_libraries(robot_odom)

install(TARGETS
  robot_can
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  robot_uart
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  robot_srv
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  robot_pub
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  robot_odom
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}
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
