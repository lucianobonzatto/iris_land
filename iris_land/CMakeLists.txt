cmake_minimum_required(VERSION 3.0.2)
project(iris_land)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  cv_bridge
  image_transport
  message_filters
  message_generation
)

find_package(OpenCV REQUIRED)

## Encontra o GStreamer usando pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)


add_message_files(
  FILES
    controllers_gain.msg
    PD_controller.msg
    PD_gain.msg
    PDPI_controller.msg
    PDPI_gain.msg
    PID_controller.msg
    PID_gain.msg
    PI_gain.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2_ros
    cv_bridge
    image_transport
    message_runtime
)

include_directories(
  include
  include/iris_land
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)

## ------------------------------------------------------------------
## Executável “iris_mng”
## ------------------------------------------------------------------
add_executable(iris_mng
  src/manager_node.cpp
  src/drone_control.cpp
  src/follow_controller.cpp
  src/general.cpp
  src/land_controller.cpp
  src/manager.cpp
  src/PIDlib.cpp
  src/ros_client.cpp
  src/state_machine.cpp
)

target_link_libraries(iris_mng
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  
)
add_dependencies(iris_mng ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## ------------------------------------------------------------------
## Executável aruco_stereo
## ------------------------------------------------------------------
add_executable(aruco_stereo
  src/aruco_stereo.cpp
)

target_link_libraries(aruco_stereo
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
add_dependencies(aruco_stereo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## ------------------------------------------------------------------
## Executável stereo_camera_publisher_node
## ------------------------------------------------------------------
add_executable(stereo_camera_publisher_node src/camera_publisher.cpp)

target_link_libraries(stereo_camera_publisher_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GST_LIBRARIES}
)

## ------------------------------------------------------------------
## Executaveis Python
## ------------------------------------------------------------------
catkin_install_python(PROGRAMS
  scripts/parameters_node.py
  scripts/joy_node.py
  scripts/aruco_rgb.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
