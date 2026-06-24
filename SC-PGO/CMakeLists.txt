cmake_minimum_required(VERSION 3.8)
project(aloam_velodyne)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(GTSAM QUIET CONFIG)
if(NOT GTSAM_FOUND)
  find_package(gtsam REQUIRED CONFIG)
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

add_executable(alaserPGO
  src/laserPosegraphOptimization.cpp
  include/scancontext/Scancontext.cpp
)

set_target_properties(alaserPGO PROPERTIES
  INSTALL_RPATH "$ORIGIN/../../../gtsam/lib"
)

target_compile_options(alaserPGO PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(alaserPGO
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  OpenMP::OpenMP_CXX
  gtsam
)
ament_target_dependencies(alaserPGO
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  pcl_conversions
  cv_bridge
)

install(TARGETS alaserPGO
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
