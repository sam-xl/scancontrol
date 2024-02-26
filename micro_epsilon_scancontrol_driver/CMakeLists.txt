cmake_minimum_required(VERSION 3.5)
project(micro_epsilon_scancontrol_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(micro_epsilon_scancontrol_msgs REQUIRED)

pkg_check_modules(SCANCONTROL REQUIRED glib-2.0 aravis-0.8 mescan llt)

include_directories(include ${SCANCONTROL_INCLUDE_DIRS}) 

add_executable(driver_node src/node.cpp src/driver.cpp)
ament_target_dependencies(driver_node 
  rclcpp
  std_srvs
  sensor_msgs
  pcl_ros
  pcl_conversions
  rclcpp_components
  micro_epsilon_scancontrol_msgs
)
target_link_libraries(driver_node
 "${SCANCONTROL_LIBRARIES}")

install(TARGETS driver_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_export_include_directories(include)
ament_export_libraries(driver_node ) #driver_nodelet

ament_package()
