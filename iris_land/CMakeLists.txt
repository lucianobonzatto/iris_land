cmake_minimum_required(VERSION 3.8)
project(iris_land)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(iris_land_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(iris_mng
  src/manager_node.cpp
  src/drone_control.cpp
  src/follow_controller.cpp
  src/general.cpp
  src/land_controller.cpp
  # src/manager.cpp
  src/PIDlib.cpp
  src/ros_client.cpp
  src/state_machine.cpp
)
target_include_directories(iris_mng PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/iris_land/
)
ament_target_dependencies(iris_mng
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  iris_land_msgs
  mavros_msgs
  tf2_ros
  cv_bridge
  OpenCV
)

install(TARGETS iris_mng
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
