cmake_minimum_required(VERSION 3.8)
project(turtlebot_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(turtlebot_cosmo_interface REQUIRED)


add_executable(turtlebot_moveit src/turtlebot_moveit.cpp)
add_executable(get_eef_pose src/get_eef_pose.cpp)
add_executable(turtlebot_arm_controller src/turtlebot_arm_controller.cpp)


target_include_directories(turtlebot_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(turtlebot_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  turtlebot_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
)
ament_target_dependencies(
  get_eef_pose
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
  "tf2_geometry_msgs"
)
ament_target_dependencies(
  turtlebot_arm_controller
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
  "tf2_geometry_msgs"
  "turtlebot_cosmo_interface"
)


install(TARGETS turtlebot_moveit get_eef_pose turtlebot_arm_controller
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(scripts)  # Point to the scripts directory

# Install Python scripts
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/srv_call_test.py
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/task.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
