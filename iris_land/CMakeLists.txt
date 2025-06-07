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
  ${GST_INCLUDE_DIRS}  # Inclui os diretórios do GStreamer
)

## ------------------------------------------------------------------
## Executável principal “iris_mng” (manager_node + demais módulos)
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
## Executável separado “aruco_node” (apenas aruco_node.cpp)
## ------------------------------------------------------------------
add_executable(aruco_node
  scripts/aruco_node.cpp
)

target_link_libraries(aruco_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
add_dependencies(aruco_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Adiciona o executável (nome: stereo_camera_publisher_node, arquivo: scripts/camera_publisher.cpp)
add_executable(stereo_camera_publisher_node src/camera_publisher.cpp)

## Linka o executável com as bibliotecas necessárias
target_link_libraries(stereo_camera_publisher_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GST_LIBRARIES}  # Linka com as bibliotecas do GStreamer
)



## Instalar scripts Python
catkin_install_python(PROGRAMS
  scripts/parameters_node.py
  scripts/joy_node.py
  # remova aruco_node (copy).py se não for usar.
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(TARGETS iris_mng aruco_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

