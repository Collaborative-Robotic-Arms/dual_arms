cmake_minimum_required(VERSION 3.8)
project(dual_arms)

# ==============================
# 🔹 Dependencies
# ==============================
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(xacro REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# MoveIt2 Dependencies
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_visual_tools REQUIRED)

# Custom/Other Packages
find_package(ros2srrc_data REQUIRED)
find_package(linkattacher_msgs REQUIRED)
find_package(abb_robot_msgs REQUIRED)
find_package(ros2_robotiqgripper REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros2srrc_execution REQUIRED)
find_package(ros2srrc_robots REQUIRED)

# ==============================
# 🔹 Compiler Settings
# ==============================
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==============================
# 🔹 Include Directories
# ==============================
include_directories(
  ${CMAKE_SOURCE_DIR}/../ros2_SimRealRobotControl/ros2srrc_execution/include
)

# ==============================
# 🔹 Executables
# ==============================
add_executable(robmove ../ros2_SimRealRobotControl/ros2srrc_execution/src/robmove.cpp)
ament_target_dependencies(robmove rclcpp moveit_ros_planning_interface yaml-cpp ros2srrc_data)
add_executable(dual_arms_controller src/dual_arms_controller.cpp)
add_executable(move
  ../ros2_SimRealRobotControl/ros2srrc_execution/src/moveg.cpp
  ../ros2_SimRealRobotControl/ros2srrc_execution/src/moverp.cpp
  ../ros2_SimRealRobotControl/ros2srrc_execution/src/moverot.cpp
  ../ros2_SimRealRobotControl/ros2srrc_execution/src/mover.cpp
  ../ros2_SimRealRobotControl/ros2srrc_execution/src/movel.cpp
  ../ros2_SimRealRobotControl/ros2srrc_execution/src/movej.cpp
  ../ros2_SimRealRobotControl/ros2srrc_execution/src/move.cpp
)
ament_target_dependencies(move rclcpp moveit_ros_planning_interface yaml-cpp ros2srrc_data)
target_link_libraries(move yaml-cpp)

add_executable(robpose ../ros2_SimRealRobotControl/ros2srrc_execution/src/robpose.cpp)
ament_target_dependencies(robpose rclcpp moveit_ros_planning_interface yaml-cpp ros2srrc_data)

# ==============================
# 🔹 Install Targets
# ==============================
install(TARGETS
  robmove
  move
  robpose
  DESTINATION lib/${PROJECT_NAME}
)
ament_target_dependencies(dual_arms_controller
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)


install(DIRECTORY config launch srdf urdf meshes DESTINATION share/${PROJECT_NAME})
install(TARGETS dual_arms_controller
  DESTINATION lib/${PROJECT_NAME})
# ==============================
# 🔹 Export
# ==============================
ament_export_dependencies(Boost)
ament_package()
