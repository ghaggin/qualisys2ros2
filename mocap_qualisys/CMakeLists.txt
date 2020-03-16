cmake_minimum_required(VERSION 3.10)
project(mocap_qualisys)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(rclcpp REQUIRED)
find_package(mocap_base REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall")

# QUALISYS
add_library(mocap_qualisys_driver
    src/QualisysDriver.cpp
    include/mocap_qualisys/RTProtocol.cpp
    include/mocap_qualisys/NBC_Markup.cpp
    include/mocap_qualisys/RTPacket.cpp
    include/mocap_qualisys/Network.cpp
)
target_include_directories(mocap_qualisys_driver PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                               "$<INSTALL_INTERFACE:include>"
                                               /usr/include/eigen3)
target_link_libraries(mocap_qualisys_driver ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})                                            
ament_target_dependencies(mocap_qualisys_driver
    "rclcpp"
    "mocap_base"
    "tf2_geometry_msgs"
    "tf2_eigen"
)

add_executable(mocap_qualisys_node
    src/qualisys.cpp
)
target_link_libraries(mocap_qualisys_node
    mocap_qualisys_driver
)
target_include_directories(mocap_qualisys_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mocap_qualisys_node "rclcpp")

install(
  TARGETS mocap_qualisys_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()