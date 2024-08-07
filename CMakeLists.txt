cmake_minimum_required(VERSION 3.10)
project(rmcs_location)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

set(PACKAGES
    rclcpp
    geometry_msgs
    nav_msgs
    std_srvs
    tf2_ros
    pcl_conversions
    pcl_ros
    Eigen
    OpenCV
    fast_gicp
)

# rmeove pcl warning
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# FIND PACKAGES
foreach(package ${PACKAGES})
  find_package(${package} REQUIRED)
  if(!${package}_FOUND)
    message("not found package: ${package}")
  endif()
endforeach()

aux_source_directory(src/match SOURCE)
aux_source_directory(src/ros2 SOURCE)

# BUILD EXE
add_executable(
  ${PROJECT_NAME}_exe 
  ${SOURCE} 
  src/main.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}_exe 
  ${PACKAGES}
)

# BUILD GICP TEST EXE
add_executable(
  ${PROJECT_NAME}_gicp
  ${SOURCE} 
  src/gicp.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}_gicp 
  ${PACKAGES}
)

# INSTALL BUILD FILE
install(
  TARGETS ${PROJECT_NAME}_exe ${PROJECT_NAME}_gicp 
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

include_directories(${PACKAGES}_INCLUDE_DIR)
include_directories(src)

ament_package()