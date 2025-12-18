cmake_minimum_required(VERSION 3.5)
project(micro_epsilon_scancontrol_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(micro_epsilon_scancontrol_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(SCANCONTROL REQUIRED
  aravis-0.8
  glib-2.0
  llt
  mescan
)

# Component library
add_library(driver_component SHARED src/driver.cpp)

target_include_directories(driver_component PUBLIC
  include
  ${SCANCONTROL_INCLUDE_DIRS}
)

ament_target_dependencies(driver_component
  micro_epsilon_scancontrol_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_srvs
  pcl_conversions
  pcl_ros
)

target_link_libraries(driver_component
  ${SCANCONTROL_LIBRARIES}
)

rclcpp_components_register_nodes(driver_component
  "scancontrol_driver::ScanControlDriver"
)

# Standalone node
add_executable(driver_node src/node.cpp)

target_link_libraries(driver_node
  driver_component
  rclcpp::rclcpp
)

# Install
install(TARGETS
  driver_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
