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

find_package(OpenCV 4.5.4 REQUIRED
  COMPONENTS
    core
    imgproc
    imgcodecs
    highgui
    videoio
    calib3d
)

# Some Ubuntu/Jetson OpenCV 4.5.4 packages install the C++ ArUco header and
# library without advertising "aruco" as a CMake component. Requiring that
# component makes configuration fail even though ArUco is usable. Verify the
# real compile/link capability instead, and add the standalone library when
# OpenCV_LIBS does not already contain it (for example via opencv_world).
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_INCLUDES ${OpenCV_INCLUDE_DIRS})
set(CMAKE_REQUIRED_LIBRARIES ${OpenCV_LIBS})
check_cxx_source_compiles(
  "
  #include <opencv2/aruco.hpp>
  int main()
  {
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    return dictionary ? 0 : 1;
  }
  "
  OPENCV_ARUCO_LINKS_FROM_EXPORTED_LIBS
)

if(NOT OPENCV_ARUCO_LINKS_FROM_EXPORTED_LIBS)
  find_library(IRIS_OPENCV_ARUCO_LIBRARY NAMES opencv_aruco)
  if(IRIS_OPENCV_ARUCO_LIBRARY)
    list(APPEND OpenCV_LIBS ${IRIS_OPENCV_ARUCO_LIBRARY})
    set(CMAKE_REQUIRED_LIBRARIES ${OpenCV_LIBS})
    check_cxx_source_compiles(
      "
      #include <opencv2/aruco.hpp>
      int main()
      {
        cv::Ptr<cv::aruco::Dictionary> dictionary =
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
        return dictionary ? 0 : 1;
      }
      "
      OPENCV_ARUCO_LINKS_WITH_STANDALONE_LIBRARY
    )
  endif()
endif()

if(NOT OPENCV_ARUCO_LINKS_FROM_EXPORTED_LIBS AND
   NOT OPENCV_ARUCO_LINKS_WITH_STANDALONE_LIBRARY)
  message(FATAL_ERROR
    "OpenCV ${OpenCV_VERSION} was found, but its C++ ArUco module cannot be "
    "compiled and linked. Install the OpenCV-contrib development package "
    "matching the OpenCV used by ROS/cv_bridge (commonly "
    "'libopencv-contrib-dev'), or rebuild OpenCV 4.5.4 with opencv_contrib. "
    "A Python cv2.aruco installation alone is not sufficient for this C++ node."
  )
endif()

unset(CMAKE_REQUIRED_INCLUDES)
unset(CMAKE_REQUIRED_LIBRARIES)
list(REMOVE_DUPLICATES OpenCV_LIBS)

message(STATUS "========== OpenCV Configuration ==========")
message(STATUS "OpenCV Version: ${OpenCV_VERSION}")
message(STATUS "OpenCV Libraries: ${OpenCV_LIBS}")
message(STATUS "OpenCV Include Dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV Install Path: ${OpenCV_INSTALL_PATH}")
message(STATUS "==========================================")

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

link_directories(/usr/local/lib)

include_directories(
  include
  include/iris_land
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)

## ------------------------------------------------------------------
## Executável "iris_mng"
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
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

set_target_properties(iris_mng PROPERTIES
  INSTALL_RPATH "/usr/local/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

add_dependencies(iris_mng ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## ------------------------------------------------------------------
## Executável aruco_stereo
## ------------------------------------------------------------------
add_executable(aruco_stereo
  src/aruco_stereo.cpp
)

target_link_libraries(aruco_stereo
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

set_target_properties(aruco_stereo PROPERTIES
  INSTALL_RPATH "/usr/local/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

add_dependencies(aruco_stereo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## ------------------------------------------------------------------
## Executável stereo_camera_publisher_node
## ------------------------------------------------------------------
add_executable(stereo_camera_publisher_node src/camera_publisher.cpp)

target_link_libraries(stereo_camera_publisher_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${GST_LIBRARIES}
)

set_target_properties(stereo_camera_publisher_node PROPERTIES
  INSTALL_RPATH "/usr/local/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

add_dependencies(stereo_camera_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## ------------------------------------------------------------------
## Executaveis Python
## ------------------------------------------------------------------
catkin_install_python(PROGRAMS
  scripts/parameters_node.py
  scripts/joy_node.py
  scripts/aruco_rgb.py
  scripts/aruco_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
