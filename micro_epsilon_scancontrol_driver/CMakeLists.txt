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
# find_package(PCL 1.8 REQUIRED)

pkg_check_modules(SCANCONTROL REQUIRED glib-2.0 aravis-0.6 mescan llt)

# catkin_package(
#   INCLUDE_DIRS include
#   CATKIN_DEPENDS roscpp pcl_ros nodelet micro_epsilon_scancontrol_msgs
#   DEPENDS scancontrol
# )

include_directories(include ${SCANCONTROL_INCLUDE_DIRS}) 

add_executable(driver_node src/node.cpp src/driver.cpp)
# target_link_libraries(driver_node ${catkin_LIBRARIES} ${scancontrol_LIBRARIES})
ament_target_dependencies(driver_node 
  rclcpp
  std_srvs
  sensor_msgs
  pcl_ros
  pcl_conversions
  rclcpp_components
  micro_epsilon_scancontrol_msgs
)
# target_include_directories(driver_node PUBLIC
#  "${SCANCONTROL_INCLUDE_DIRS}")
target_link_libraries(driver_node
 "${SCANCONTROL_LIBRARIES}")

# add_library(driver_nodelet src/nodelet.cpp src/driver.cpp)
# # target_link_libraries(driver_nodelet ${catkin_LIBRARIES} ${scancontrol_LIBRARIES})
# ament_target_dependencies(driver_nodelet 
#   rclcpp
#   rclcpp_components
#   micro_epsilon_scancontrol_msgs
# )

ament_export_include_directories(include)
ament_export_libraries(driver_node ) #driver_nodelet
# ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
