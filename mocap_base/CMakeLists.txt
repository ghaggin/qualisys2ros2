cmake_minimum_required(VERSION 3.10)
project(mocap_base)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall")

add_library(mocap_kalman_filter SHARED
  src/KalmanFilter.cpp
)
target_include_directories(mocap_kalman_filter PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:include>")

add_library(mocap_base SHARED
  src/MoCapDriverBase.cpp
)
target_include_directories(mocap_base PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(mocap_base "geometry_msgs" "rclcpp" "nav_msgs")
target_link_libraries(mocap_base
  mocap_kalman_filter
)

ament_export_include_directories(include)
ament_export_libraries(mocap_base mocap_kalman_filter)
ament_export_interfaces(export_mocap_base HAS_LIBRARY_TARGET)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS mocap_base mocap_kalman_filter
  EXPORT export_mocap_base
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()