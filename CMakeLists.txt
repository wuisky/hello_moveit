cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  control_toolbox
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  pluginlib
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  tf2_eigen
  trajectory_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msgs/srv/PlanExecutePoses.srv"
  DEPENDENCIES
  builtin_interfaces
  moveit_msgs
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(moveit_sample src/hello_moveit.cpp)
# target_include_directories(hello_moveit PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_sample
  moveit_ros_planning_interface
  moveit_visual_tools
  rclcpp
)

add_executable(hello_ur src/hello_ur.cpp)
ament_target_dependencies(
  hello_ur
  moveit_ros_planning_interface
  moveit_visual_tools
  rclcpp
)


add_executable(moveit_client src/moveit_client.cpp)
ament_target_dependencies(
  moveit_client
  moveit_ros_planning_interface
  moveit_visual_tools
  rclcpp
  lifecycle_msgs
  rclcpp_lifecycle
  std_msgs
)
rosidl_target_interfaces(moveit_client ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Add executable for using a controller
set(SERVO_CONTROLLER_INPUT servo_controller_input)
add_library(${SERVO_CONTROLLER_INPUT} SHARED src/teleop_demo/joystick_servo_example.cpp)
ament_target_dependencies(${SERVO_CONTROLLER_INPUT} ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(${SERVO_CONTROLLER_INPUT} "moveit_servo::JoyToServoPub")
# Install Libraries
install(
  TARGETS
    ${SERVO_CONTROLLER_INPUT}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  moveit_sample
  hello_ur
  moveit_client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz config
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
